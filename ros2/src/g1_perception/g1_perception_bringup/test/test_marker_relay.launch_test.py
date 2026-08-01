"""obstacles_marker_relay unit-level test (§6.3/§14.4, Phase 3): runs the
relay executable (g1_perception_utils) and checks the Obstacles→MarkerArray
mapping — circle→cylinder scale, velocity arrow presence, uid text labels,
frame/stamp passthrough, and the empty-input DELETEALL contract.

Lives in bringup because the relay is exercised as a node (its class is not
exported as a library) and bringup already carries the launch_testing
infrastructure.

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_marker_relay.launch_test.py
"""
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='g1_perception_utils',
            executable='obstacles_marker_relay',
            name='test_marker_relay',
            parameters=[{'topic': '/relay_test_in',
                         'cylinder_height': 1.5,
                         'show_ids': True}],
            output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMarkerRelay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        from rclpy.qos import QoSProfile
        from obstacle_detector.msg import Obstacles
        from visualization_msgs.msg import MarkerArray

        rclpy.init()
        cls.node = rclpy.create_node('marker_relay_probe')
        cls.lock = threading.Lock()
        cls.out = []
        cls.pub = cls.node.create_publisher(
            Obstacles, '/relay_test_in', QoSProfile(depth=5))
        cls.node.create_subscription(
            MarkerArray, '/test_marker_relay/markers',
            lambda m: cls._push(m), QoSProfile(depth=1))
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()
        time.sleep(2.0)  # discovery

    @classmethod
    def _push(cls, m):
        with cls.lock:
            cls.out.append(m)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        cls.executor.shutdown()
        rclpy.shutdown()

    def _send_and_wait(self, msg, timeout=5.0):
        with self.lock:
            self.out.clear()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pub.publish(msg)
            time.sleep(0.2)
            with self.lock:
                if self.out:
                    return self.out[-1]
        self.fail('no MarkerArray received from relay')

    @staticmethod
    def _make_obstacles():
        from obstacle_detector.msg import Obstacles, CircleObstacle
        m = Obstacles()
        m.header.frame_id = 'odom'
        m.header.stamp.sec = 123
        c = CircleObstacle()
        c.uid = 42
        c.center.x, c.center.y = 2.0, -1.0
        c.velocity.x, c.velocity.y = 0.5, 0.0
        c.radius, c.true_radius = 0.32, 0.15
        m.circles.append(c)
        s = CircleObstacle()          # static circle: no arrow expected
        s.uid = 7
        s.center.x, s.center.y = -1.0, 1.0
        s.radius, s.true_radius = 0.2, 0.1
        m.circles.append(s)
        return m

    def test_mapping_frames_and_ids(self):
        out = self._send_and_wait(self._make_obstacles())
        by_ns = {}
        for mk in out.markers:
            by_ns.setdefault(mk.ns, []).append(mk)
        # DELETEALL header marker + 2 cylinders + 1 arrow + 2 id labels
        self.assertEqual(len(by_ns.get('circles', [])), 2)
        self.assertEqual(len(by_ns.get('velocities', [])), 1)
        self.assertEqual(len(by_ns.get('ids', [])), 2)
        cyl = by_ns['circles'][0]
        self.assertEqual(cyl.header.frame_id, 'odom')
        self.assertEqual(cyl.header.stamp.sec, 123)
        self.assertAlmostEqual(cyl.scale.x, 2.0 * 0.32, places=6)
        labels = sorted(mk.text for mk in by_ns['ids'])
        self.assertEqual(labels, ['42', '7'])

    def test_empty_input_clears(self):
        from obstacle_detector.msg import Obstacles
        m = Obstacles()
        m.header.frame_id = 'odom'
        out = self._send_and_wait(m)
        from visualization_msgs.msg import Marker
        self.assertEqual(len(out.markers), 1)
        self.assertEqual(out.markers[0].action, Marker.DELETEALL)
