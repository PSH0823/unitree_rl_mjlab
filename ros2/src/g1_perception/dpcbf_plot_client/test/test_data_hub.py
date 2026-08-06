"""DataHub integration: in-process rclpy pub -> hub -> snapshot.

No GUI here — this is the data path the GUIs draw from. Verifies message
consumption on both topics, series accumulation, snapshot consistency and
the age bookkeeping the stale display is built on.
"""
import math
import time
import unittest

import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from dpcbf_viz_msgs.msg import DpcbfPlotSample, PlotObstacle
from nav_msgs.msg import Odometry

from dpcbf_plot_client.data_hub import DataHub


def _spin_until(executor, pred, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if pred():
            return True
    return pred()


class TestDataHub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.hub = DataHub()
        self.pub_node = rclpy.create_node('data_hub_test_pub')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.hub)
        self.executor.add_node(self.pub_node)

    def tearDown(self):
        self.hub.destroy_node()
        self.pub_node.destroy_node()

    def _sample(self, tick, intervention=False):
        msg = DpcbfPlotSample()
        msg.header.stamp = self.pub_node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.tick = tick
        msg.t_ctrl = 0.001 * tick
        msg.mode = DpcbfPlotSample.MODE_ESTIMATED
        msg.robot_x = 1.0
        msg.robot_y = 2.0
        msg.robot_phi = 0.3
        msg.nominal.sagittal = 0.5
        msg.scaled.sagittal = 0.5
        msg.safe.sagittal = 0.2 if intervention else 0.5
        msg.command_scale = 1.0
        msg.intervention = intervention
        msg.solved = True
        msg.staleness_state = DpcbfPlotSample.STALENESS_FRESH
        msg.obstacle_age_s = 0.05
        msg.obstacle_total = 1
        o = PlotObstacle()
        o.id = 3
        o.x = 2.5
        o.y = 2.0
        o.radius = 0.3
        o.distance = 1.5
        o.h = -0.1 if intervention else 0.7
        msg.obstacles.append(o)
        msg.min_h_valid = True
        msg.min_h = o.h
        msg.min_clearance = o.distance - o.radius
        return msg

    def test_plot_sample_flow(self):
        pub = self.pub_node.create_publisher(
            DpcbfPlotSample, '/dpcbf/plot',
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=5))
        for tick in range(1, 21):
            pub.publish(self._sample(tick, intervention=tick > 10))
            self.executor.spin_once(timeout_sec=0.02)
        ok = _spin_until(self.executor,
                         lambda: self.hub.snapshot()['counts']['plot'] >= 15)
        self.assertTrue(ok, 'hub never received the plot samples')

        snap = self.hub.snapshot()
        self.assertIsNotNone(snap['sample'])
        self.assertEqual(snap['sample'].robot_x, 1.0)
        self.assertTrue(snap['sample'].intervention)
        ts, vs = snap['series']['nominal_sagittal']
        self.assertEqual(len(ts), len(vs))
        self.assertGreaterEqual(len(vs), 15)
        self.assertTrue(all(v == 0.5 for v in vs))
        _, interv = snap['series']['intervention']
        self.assertIn(1.0, interv)
        self.assertIn(0.0, interv)
        _, min_h = snap['series']['min_h']
        self.assertAlmostEqual(min_h[-1], -0.1)
        # x-axes are "seconds ago": monotone nondecreasing, ending <= 0
        self.assertLessEqual(ts[-1], 0.0 + 1e-6)
        self.assertLess(snap['ages']['plot'], 5.0)
        self.assertIsNone(snap['ages']['odom'])

    def test_odom_flow_and_trail(self):
        pub = self.pub_node.create_publisher(Odometry, '/odom',
                                             QoSProfile(depth=10))
        yaw = 0.5
        for i in range(10):
            msg = Odometry()
            msg.header.stamp = self.pub_node.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.pose.position.x = float(i)
            msg.pose.pose.position.y = -float(i)
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.twist.twist.linear.x = 0.4
            pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.02)
        ok = _spin_until(self.executor,
                         lambda: self.hub.snapshot()['counts']['odom'] >= 5)
        self.assertTrue(ok, 'hub never received odometry')

        snap = self.hub.snapshot()
        self.assertIsNotNone(snap['odom'])
        self.assertAlmostEqual(snap['odom']['yaw'], yaw, places=5)
        self.assertAlmostEqual(snap['odom']['vx'], 0.4)
        xs, ys = snap['trail']
        self.assertGreaterEqual(len(xs), 5)
        self.assertEqual(xs[-1], -ys[-1])

    def test_ages_grow_when_source_stops(self):
        pub = self.pub_node.create_publisher(
            DpcbfPlotSample, '/dpcbf/plot',
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=5))
        for tick in range(1, 6):
            pub.publish(self._sample(tick))
            self.executor.spin_once(timeout_sec=0.02)
        ok = _spin_until(self.executor,
                         lambda: self.hub.snapshot()['counts']['plot'] >= 1)
        self.assertTrue(ok)
        age0 = self.hub.snapshot()['ages']['plot']
        time.sleep(0.5)
        age1 = self.hub.snapshot()['ages']['plot']
        self.assertGreater(age1, age0 + 0.4,
                           'age must keep growing after the source stops')


if __name__ == '__main__':
    unittest.main()
