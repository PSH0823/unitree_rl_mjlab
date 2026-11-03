"""launch_testing smoke (§16.1): bringup.launch.py source:=sim must produce
every sim-side §7.1 topic at its expected rate (±20%).

Integration-gated: needs the ROS2-enabled simulator already running (it owns
/clock, /sim/mj_state and the mirror-model dump). Without it the test SKIPS
so `colcon test` stays green on a bare machine.

Standalone run:
  launch_test src/g1_perception/g1_perception_bringup/test/test_bringup_sim.launch_test.py
"""
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    from ament_index_python.packages import get_package_share_directory
    bringup = os.path.join(get_package_share_directory('g1_perception_bringup'),
                           'launch', 'bringup.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments=[('source', 'sim'), ('viz', 'off'), ('record', 'off')],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimTopics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('bringup_smoke_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _count(self, topic_type, topic, qos, seconds):
        counts = []
        sub = self.node.create_subscription(topic_type, topic,
                                            lambda m: counts.append(1), qos)
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return len(counts)

    def test_topics_and_rates(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sim_msgs.msg import MjState

        best_effort = QoSProfile(depth=5,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # gate on the simulator being up
        if self._count(MjState, '/sim/mj_state',
                       QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
                       5.0) == 0:
            self.skipTest('simulator with ROS2 module not running')

        t0 = time.time()
        cloud = self._count(PointCloud2, '/livox/lidar', best_effort, 6.0)
        elapsed = time.time() - t0
        cloud_hz = cloud / elapsed
        self.assertGreaterEqual(cloud_hz, 8.0,   # 10 Hz -20%
                                f'/livox/lidar at {cloud_hz:.1f} Hz')
        self.assertLessEqual(cloud_hz, 12.5)

        t0 = time.time()
        odom = self._count(Odometry, '/odom', QoSProfile(depth=10), 4.0)
        odom_hz = odom / (time.time() - t0)
        self.assertGreaterEqual(odom_hz, 50.0,   # >=50 Hz contract (§7.1)
                                f'/odom at {odom_hz:.1f} Hz')
