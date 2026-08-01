#!/usr/bin/python3
"""livox_ros_driver2 OUTPUT EMULATOR for bench testing (Phase 5A).

This is not a simulator of a Mid360. It replays the geometry of a recorded
sim fixture bag but re-wraps it in exactly the wire format, frame, QoS,
clock and topic set that the pinned livox_ros_driver2 (tag 1.2.6, patch 0005)
produces on hardware — all verified against that checkout, not its README:

  /livox/lidar  PointCloud2, 7 fields, point_step 26 (#pragma pack(1)):
                x,y,z,intensity FLOAT32 @0/4/8/12; tag,line UINT8 @16/17;
                timestamp FLOAT64 @18  (xfer_format 0; the only PointCloud2
                format the ROS2 build actually implements)
                frame_id from the frame_id param (mid360_link, §7.1)
                QoS: RELIABLE, depth 256  <- the driver's, not SensorData
  /livox/imu    sensor_msgs/Imu @200 Hz, same frame (patch 0005 makes the
                driver honour frame_id here; upstream hardcodes livox_frame)
                QoS: RELIABLE, depth 256
  stamps        WALL clock. On hardware there is no /clock; the driver stamps
                from the LiDAR when it is PTP/GPS-synced and from the host
                clock otherwise (pub_handler.cpp GetEthPacketTimestamp).

The IMU stream is synthetic: a stationary specific-force vector (gravity
expressed in the sensor frame through the H-1 roll=pi mount) plus optional
noise. It exists because DLIO will not initialise without IMU and the sim
sidecar never published one (§7.1 marks it "sidecar (optional)"). Odometry
computed against it is meaningless — these tests assert wiring, QoS and TF
tree shape, never odometry quality.

Optionally (odom_hz > 0) it also plays the part of the odometry source,
publishing a motionless /odom + dynamic TF odom->base_link at 100 Hz with
wall-clock stamps. That exists so the cloud seam can be gated WITHOUT DLIO in
the loop. A `static_transform_publisher` cannot do this job: base_footprint_
publisher deduplicates by stamp, so a static odom->base_link makes it emit
base_footprint exactly once, at time 0, and every later lookup extrapolates
into the future and fails. Real odometry (sidecar or DLIO) is dynamic.

Params: bag (path), rate (playback speed), loop (bool), imu_hz, odom_hz,
frame_id, imu_noise.
"""
import math
import os
import struct
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

# Driver-identical publisher QoS: kMinEthPacketQueueSize(32) * 8.
DRIVER_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=256)

POINT_STEP = 26
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
    PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]

# Gravity in the sensor frame: R(base->mid360)^T @ [0,0,g]. The mount is
# roll=pi (H-1), so an accelerometer at rest reads ~-9.8 on the sensor z.
_G = 9.80665
_PITCH = 0.000892
GRAVITY_SENSOR = np.array([-_G * math.sin(_PITCH), 0.0, -_G * math.cos(_PITCH)])


def read_clouds(bag_path, topic='/livox/lidar'):
    """Return [(t_rel_s, [(x,y,z), ...]), ...] from a rosbag2 fixture."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f'{topic} not in bag {bag_path}: {sorted(types)}')
    msg_type = get_message(types[topic])
    out, t0 = [], None
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if t0 is None:
            t0 = t_ns
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        out.append(((t_ns - t0) * 1e-9, pts))
    return out


class HwSourceStub(Node):
    def __init__(self):
        super().__init__('hw_source_stub')
        bag = self.declare_parameter('bag', '').value
        self.rate = float(self.declare_parameter('rate', 1.0).value)
        self.loop = bool(self.declare_parameter('loop', True).value)
        self.imu_hz = float(self.declare_parameter('imu_hz', 200.0).value)
        self.frame = self.declare_parameter('frame_id', 'mid360_link').value
        self.imu_noise = float(self.declare_parameter('imu_noise', 0.0).value)
        self.odom_hz = float(self.declare_parameter('odom_hz', 0.0).value)

        if not bag or not os.path.isdir(bag):
            raise RuntimeError(f'hw_source_stub: bag not found: {bag!r}')
        self.frames = read_clouds(bag)
        self.get_logger().info(
            f'loaded {len(self.frames)} clouds from {bag} '
            f'({self.frames[-1][0]:.1f} s)')

        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar',
                                               DRIVER_QOS)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', DRIVER_QOS)
        self.rng = np.random.default_rng(0)

        self.odom_pub = None
        self.tf_broadcaster = None
        if self.odom_hz > 0.0:
            from nav_msgs.msg import Odometry
            from tf2_ros import TransformBroadcaster
            self._Odometry = Odometry
            self.odom_pub = self.create_publisher(Odometry, '/odom',
                                                  QoSProfile(depth=10))
            self.tf_broadcaster = TransformBroadcaster(self)

        self._stop = threading.Event()
        threading.Thread(target=self._cloud_worker, daemon=True).start()
        threading.Thread(target=self._imu_worker, daemon=True).start()
        if self.odom_pub is not None:
            threading.Thread(target=self._odom_worker, daemon=True).start()

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()

    # --- driver-format cloud ------------------------------------------------
    def _pack(self, pts, stamp_s):
        buf = bytearray(POINT_STEP * len(pts))
        off = 0
        for x, y, z in pts:
            struct.pack_into('<ffffBBd', buf, off,
                             float(x), float(y), float(z), 0.0, 0, 0, stamp_s)
            off += POINT_STEP
        msg = PointCloud2()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = len(pts)
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * len(pts)
        msg.is_dense = True
        msg.data = bytes(buf)
        return msg

    def _cloud_worker(self):
        while not self._stop.is_set():
            t_start = time.monotonic()
            for t_rel, pts in self.frames:
                if self._stop.is_set():
                    return
                due = t_start + t_rel / self.rate
                delay = due - time.monotonic()
                if delay > 0:
                    if self._stop.wait(delay):
                        return
                now = self.get_clock().now()
                msg = self._pack(pts, now.nanoseconds * 1e-9)
                msg.header.stamp = now.to_msg()
                self.cloud_pub.publish(msg)
            if not self.loop:
                return

    # --- synthetic stationary IMU ------------------------------------------
    def _imu_worker(self):
        period = 1.0 / self.imu_hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0 and self._stop.wait(delay):
                return
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.frame
            a = GRAVITY_SENSOR
            if self.imu_noise:
                a = a + self.rng.normal(0.0, self.imu_noise, 3)
            m.linear_acceleration.x = float(a[0])
            m.linear_acceleration.y = float(a[1])
            m.linear_acceleration.z = float(a[2])
            if self.imu_noise:
                g = self.rng.normal(0.0, self.imu_noise * 0.1, 3)
                m.angular_velocity.x = float(g[0])
                m.angular_velocity.y = float(g[1])
                m.angular_velocity.z = float(g[2])
            m.orientation_covariance[0] = -1.0   # orientation not provided
            self.imu_pub.publish(m)


    # --- stand-in odometry (odom_hz > 0), motionless but DYNAMIC ------------
    def _odom_worker(self):
        from geometry_msgs.msg import TransformStamped
        period = 1.0 / self.odom_hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0 and self._stop.wait(delay):
                return
            stamp = self.get_clock().now().to_msg()
            odom = self._Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init(args=sys.argv)
    node = HwSourceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
