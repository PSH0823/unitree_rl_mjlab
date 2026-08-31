"""Synthetic scene for the 3D-pipeline launch test (test_tracking_3d).

Publishes, on wall time:
  * /tf_static  odom -> mid360_link (sensor 0.6 m above the floor)
  * /livox/lidar (PointCloud2, mid360_link, 10 Hz):
      - a floor grid at z_odom = 0.02  — must be REMOVED by the detector's
        z band (z_min 0.10); if it leaks, it clusters with everything and the
        test's shape assertions fail, which is the point
      - a moving pillar (r 0.15 m, height 0.1–1.2 m) starting at
        (2.0, -1.5) in odom, moving +y at SPEED until Y_STOP, then holding —
        the tracked velocity target

No bag, no MuJoCo: the scene IS the ground truth (SPEED below), so the test
asserts against constants instead of a fixture.
"""
import math
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

SPEED = 0.5        # [m/s] +y in odom — the value the test asserts against
Y_START = -1.5     # [m]
Y_STOP = 2.5       # [m]
PILLAR_X = 2.0     # [m]
SENSOR_Z = 0.6     # [m] odom z of mid360_link
RATE = 10.0        # [Hz]


def _cloud(points, stamp, frame):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


class Scene3dSource(Node):
    def __init__(self):
        super().__init__('scene_3d_source')
        self._static = StaticTransformBroadcaster(self)
        tfs = []
        # sensor frame, and the ego frame the tracker looks up (ego_source:
        # tf) — without base_link every measurement logs "Failed to get ego
        # pose" and ego-relative processing degrades.
        for child, z in (('mid360_link', SENSOR_Z), ('base_link', 0.0)):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = child
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.0
            tfs.append(tf)
        self._static.sendTransform(tfs)

        self._pub = self.create_publisher(
            PointCloud2, '/livox/lidar',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(1.0 / RATE, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        y = min(Y_START + SPEED * t, Y_STOP)

        pts = []
        # floor grid (odom z = 0.02), 1..4 m ahead, +/-3 m across
        for gx in range(4, 17):
            for gy in range(-12, 13):
                pts.append((gx * 0.25, gy * 0.25, 0.02 - SENSOR_Z))
        # pillar: rings every 0.1 m of height, r = 0.15 m — SENSOR-FACING
        # HALF only, like a real single-viewpoint scan (this also exercises
        # the detector's depth completion in the e2e gate).
        dx, dy = PILLAR_X, y  # sensor is at the odom origin in xy
        dn = math.hypot(dx, dy) or 1.0
        for k in range(1, 13):
            z = k * 0.1
            for a in range(16):
                th = 2.0 * math.pi * a / 16.0
                nx, ny = math.cos(th), math.sin(th)
                if nx * dx / dn + ny * dy / dn >= 0.0:
                    continue  # back half: surface normal points away
                pts.append((PILLAR_X + 0.15 * nx,
                            y + 0.15 * ny,
                            z - SENSOR_Z))
        self._pub.publish(_cloud(pts, now.to_msg(), 'mid360_link'))


def main():
    rclpy.init()
    node = Scene3dSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
