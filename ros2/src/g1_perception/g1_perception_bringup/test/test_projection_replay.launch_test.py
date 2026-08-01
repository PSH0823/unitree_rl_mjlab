"""Bag-driven node integration (§16.1, Phase 2): the perception container
replays the s1_static_reference fixture and must produce
/points_self_filtered and /scan at the contract rate (10 Hz, <20% drops,
§18 Phase-2 gate) with T9 checked live against the replayed TF tree.

SKIPs when the fixture bag is absent (it is gitignored — regenerate per
test_fixtures/README.md). The replay discipline established here (recorded
/clock replayed as a topic, use_sim_time everywhere, stamps compared in sim
time) is what Phase 3 inherits for T4/T5/T8.

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_projection_replay.launch_test.py
"""

# Concurrent launch tests share topic names; give this one a private DDS
# domain before anything else touches ROS (see isolate_domain.py).
# launch_test loads this file by path without putting its directory on
# sys.path, so the sibling import needs the insert first.
import os as _os, sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E402
import isolate_domain  # noqa: E402
isolate_domain.isolate(isolate_domain.PROJECTION_REPLAY)  # noqa: E402
import math
import os
import threading
import time
import unittest

import launch
import launch_testing.actions
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
BAG = os.path.join(WS, 'test_fixtures', 's1_static_reference')

SKIP_REASON = None
if not os.path.isdir(BAG):
    SKIP_REASON = f'fixture bag missing: {BAG} (gitignored; see README)'

BAG_SCAN_RATE = 10.0  # §7.1
MAX_DROP_FRACTION = 0.20  # §18 Phase-2 gate


@pytest.mark.launch_test
def generate_test_description():
    if SKIP_REASON:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_dir = os.path.join(
        get_package_share_directory('g1_perception_bringup'), 'launch')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'perception.launch.py'))),
        # give the container + probe subscriptions time to discover
        launch.actions.TimerAction(
            period=4.0,
            actions=[launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'play', BAG], output='screen')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestProjectionReplay(unittest.TestCase):
    COLLECT_S = 36.0  # bag is 27.87 s + startup margin

    @classmethod
    def setUpClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        import tf2_ros

        rclpy.init()
        cls.node = rclpy.create_node('replay_probe')
        cls.lock = threading.Lock()
        cls.scans = []
        cls.filtered = []          # (stamp_sec, width)
        cls.clouds = []            # (stamp_msg, t_recv, width)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.node.create_subscription(
            LaserScan, '/scan', lambda m: cls._push(cls.scans, m),
            qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, '/points_self_filtered',
            lambda m: cls._push(
                cls.filtered,
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.width)),
            qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, '/livox/lidar',
            lambda m: cls._push(cls.clouds,
                                (m.header.stamp, time.monotonic(), m.width)),
            qos_profile_sensor_data)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin,
                                           daemon=True)
        cls.spin_thread.start()

        cls.t9_checked = 0
        cls.t9_misses = 0
        cls._t9_stop = False
        cls.t9_thread = threading.Thread(target=cls._t9_worker, daemon=True)
        cls.t9_thread.start()

        time.sleep(cls.COLLECT_S)
        cls._t9_stop = True
        cls.t9_thread.join(timeout=2.0)

    @classmethod
    def _push(cls, store, item):
        with cls.lock:
            store.append(item)

    @classmethod
    def _t9_worker(cls):
        # same discipline as the wall gate: count from first resolution
        # (pre-discovery TF is unobservable to this probe), then 0 misses.
        from rclpy.time import Time
        done = 0
        warm = False
        while not cls._t9_stop:
            with cls.lock:
                pending = cls.clouds[done:]
            if not pending:
                time.sleep(0.005)
                continue
            for stamp, t_recv, _ in pending:
                deadline = t_recv + 0.050  # §16.2-T9
                ok = cls.tf_buffer.can_transform(
                    'odom', 'base_footprint', Time.from_msg(stamp))
                while not ok and time.monotonic() < deadline:
                    time.sleep(0.002)
                    ok = cls.tf_buffer.can_transform(
                        'odom', 'base_footprint', Time.from_msg(stamp))
                if not warm:
                    warm = ok
                    continue
                cls.t9_checked += 1
                cls.t9_misses += 0 if ok else 1
            done += len(pending)

    @classmethod
    def tearDownClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        if SKIP_REASON:
            self.skipTest(SKIP_REASON)
        with self.lock:
            self._scans = list(self.scans)
            self._filtered = list(self.filtered)
            self._clouds = list(self.clouds)

    def _drop_fraction(self, stamps):
        """1 - received/expected over the observed sim-time span."""
        expected = (stamps[-1] - stamps[0]) * BAG_SCAN_RATE + 1
        return 1.0 - len(stamps) / expected

    def test_scan_rate(self):
        stamps = [s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
                  for s in self._scans]
        self.assertGreater(len(stamps), 100, 'too few /scan frames from bag')
        drop = self._drop_fraction(stamps)
        span = stamps[-1] - stamps[0]
        print(f'/scan: {len(stamps)} frames over {span:.1f} s sim time '
              f'-> {(len(stamps) - 1) / span:.2f} Hz, drop {drop * 100:.1f}%')
        self.assertLess(drop, MAX_DROP_FRACTION)

    def test_filtered_rate_and_cropbox(self):
        self.assertGreater(len(self._filtered), 100,
                           'too few /points_self_filtered frames')
        stamps = [t for t, _ in self._filtered]
        drop = self._drop_fraction(stamps)
        self.assertLess(drop, MAX_DROP_FRACTION)
        # CropBox pass-through accounting (sim masks the head shell already,
        # H-4, so removal should be small — measured, not assumed)
        by_stamp = {t: w for t, w in self._filtered}
        removed = []
        for stamp, _, w_in in self._clouds:
            t = stamp.sec + stamp.nanosec * 1e-9
            if t in by_stamp:
                removed.append(w_in - by_stamp[t])
        self.assertTrue(removed, 'no matching cloud/filtered stamp pairs')
        removed.sort()
        print(f'CropBox removed points/frame: median '
              f'{removed[len(removed) // 2]}, max {removed[-1]} '
              f'({len(removed)} matched frames)')
        for r in removed:
            self.assertGreaterEqual(r, 0, 'filtered cloud larger than input')

    def test_scan_sanity(self):
        first = self._scans[0]
        self.assertEqual(first.header.frame_id, 'base_footprint')
        self.assertAlmostEqual(first.angle_increment, 0.0058, places=9)
        prev = None
        for s in self._scans:
            self.assertEqual(len(s.ranges), len(first.ranges))
            t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
            if prev is not None:
                self.assertGreater(t, prev)
            prev = t
            for r in s.ranges:
                self.assertFalse(math.isnan(r))

    def test_t9_tf_availability(self):
        self.assertGreater(self.t9_checked, 0, 'no clouds T9-checked')
        print(f'T9 (replay): {self.t9_checked} clouds, '
              f'{self.t9_misses} misses')
        self.assertEqual(self.t9_misses, 0)
