"""sim_mjlidar_bridge node (§11.2): simulated Mid360 source.

Subscribes /sim/mj_state (BestEffort depth 1), mirrors the compiled sim model
(mj_kinematics, poses only), raycasts the Livox mid360 rosette with the
MuJoCo-LiDAR CPU backend at scan_rate on the freshest state, and publishes:

  /livox/lidar   sensor_msgs/PointCloud2 (x,y,z float32, frame mid360_link,
                 stamped with SIM time from the state message — never wall time)
  /odom          nav_msgs/Odometry (ground truth pose; twist not populated —
                 MjState carries no qvel by design, §7.1)
  TF             odom -> base_link

Head-shell self-hits are excluded via the geom-group mask (H-4): group 2 is
masked out of the raycast. Runs on system Python 3.10 + rclpy (R-11) with
use_sim_time:=true.
"""
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from sim_msgs.msg import MjState

from mujoco_lidar.lidar_wrapper import MjLidarWrapper
from mujoco_lidar.scan_gen import LivoxGenerator

from .mirror import MirrorModel


def to_time_msg(sim_time: float) -> TimeMsg:
    sec = int(sim_time)
    return TimeMsg(sec=sec, nanosec=int(round((sim_time - sec) * 1e9)))


class SimMjLidarBridge(Node):
    def __init__(self):
        super().__init__('sim_mjlidar_bridge')
        self.mirror_path = self.declare_parameter(
            'mirror_model_path', '/tmp/unitree_mujoco_mirror_model.xml').value
        self.lidar_site = self.declare_parameter('lidar_site', 'mid360_link').value
        self.base_body = self.declare_parameter('base_body', 'pelvis').value
        self.scan_rate = self.declare_parameter('scan_rate', 10.0).value
        self.cutoff = self.declare_parameter('cutoff_dist', 40.0).value
        self.min_range = self.declare_parameter('min_range', 0.1).value
        self.masked_group = int(self.declare_parameter('masked_geom_group', 2).value)
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'mid360_link').value

        self.mirror = MirrorModel(self.mirror_path)
        self.base_qadr = self.mirror.free_body_qpos_addr(self.base_body)

        geomgroup = np.ones(6, dtype=np.uint8)
        if 0 <= self.masked_group < 6:
            geomgroup[self.masked_group] = 0
        self.lidar = MjLidarWrapper(
            self.mirror.model, site_name=self.lidar_site, backend='cpu',
            cutoff_dist=self.cutoff, args={'geomgroup': geomgroup})
        self.livox = LivoxGenerator('mid360')

        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.state_sub = self.create_subscription(
            MjState, '/sim/mj_state', self.on_state, state_qos)

        cloud_qos = qos_profile_sensor_data
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar', cloud_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)

        self.state = None
        self.state_errors = 0
        self.scan_count = 0
        self.raycast_s = []  # per-frame raycast wall time (benchmark §6)
        self.points_per_frame = []

        # The raycast (tens of ms on the full G1 scene) must not run on the
        # executor thread — it would starve the 100 Hz odom/TF path. A worker
        # thread paces scans by SIM time (period 1/scan_rate in d->time), so
        # scan density is invariant to the simulator's realtime factor and
        # scans stop while the simulation is paused.
        self._cv = threading.Condition()
        self._stop = False
        self._worker = threading.Thread(target=self.scan_worker, daemon=True)
        self._worker.start()

        self.stats_timer = self.create_timer(10.0, self.log_stats)
        self.get_logger().info(
            f"mirror={self.mirror_path} nq={self.mirror.model.nq} "
            f"nmocap={self.mirror.model.nmocap} site={self.lidar_site} "
            f"backend=cpu masked_group={self.masked_group}")

    # 100 Hz path: ground-truth odom + TF from every state message
    def on_state(self, msg: MjState):
        with self._cv:
            self.state = msg
            self._cv.notify()
        a = self.base_qadr
        stamp = to_time_msg(msg.sim_time)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = msg.qpos[a + 0]
        odom.pose.pose.position.y = msg.qpos[a + 1]
        odom.pose.pose.position.z = msg.qpos[a + 2]
        # MuJoCo quat (w x y z) -> ROS (x y z w)
        odom.pose.pose.orientation.w = msg.qpos[a + 3]
        odom.pose.pose.orientation.x = msg.qpos[a + 4]
        odom.pose.pose.orientation.y = msg.qpos[a + 5]
        odom.pose.pose.orientation.z = msg.qpos[a + 6]
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def scan_worker(self):
        period = 1.0 / self.scan_rate
        next_due = None
        while True:
            with self._cv:
                self._cv.wait(timeout=0.5)
                if self._stop:
                    return
                msg = self.state
            if msg is None:
                continue
            if next_due is None:
                next_due = msg.sim_time
            if msg.sim_time + 1e-9 < next_due:
                if msg.sim_time < next_due - 2 * period:  # sim reset
                    next_due = msg.sim_time
                continue
            next_due = max(next_due + period, msg.sim_time - period)
            self.scan_once(msg)

    # 10 Hz (sim time) path: raycast on the freshest mirrored state
    def scan_once(self, msg):
        try:
            self.mirror.set_state(msg.qpos, msg.mocap_pos, msg.mocap_quat)
        except ValueError as e:
            self.state_errors += 1
            if self.state_errors % 50 == 1:
                self.get_logger().error(str(e))
            return

        theta, phi = self.livox.sample_ray_angles()
        t0 = time.perf_counter()
        self.lidar.trace_rays(self.mirror.data, theta, phi)
        dt = time.perf_counter() - t0

        dist = self.lidar.get_distances()
        pts = self.lidar.get_hit_points()
        valid = (dist > self.min_range) & (dist < self.cutoff)
        pts = np.ascontiguousarray(pts[valid], dtype=np.float32)

        cloud = PointCloud2()
        cloud.header.stamp = to_time_msg(msg.sim_time)
        cloud.header.frame_id = self.lidar_frame
        cloud.height = 1
        cloud.width = pts.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self.cloud_pub.publish(cloud)

        self.scan_count += 1
        self.raycast_s.append(dt)
        self.points_per_frame.append(pts.shape[0])

    def log_stats(self):
        if not self.raycast_s:
            return
        r = np.array(self.raycast_s[-100:])
        p = np.array(self.points_per_frame[-100:])
        nrays = self.livox.samples
        self.get_logger().info(
            f"scans={self.scan_count} raycast mean={r.mean()*1e3:.2f}ms "
            f"p95={np.percentile(r, 95)*1e3:.2f}ms "
            f"({nrays / r.mean() / 1e6:.2f} Mrays/s) "
            f"points/frame mean={p.mean():.0f}")


def main(args=None):
    rclpy.init(args=args)
    node = SimMjLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node._cv:
            node._stop = True
            node._cv.notify()
        node._worker.join(timeout=2.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # context already shut down by the launch system


if __name__ == '__main__':
    main()
