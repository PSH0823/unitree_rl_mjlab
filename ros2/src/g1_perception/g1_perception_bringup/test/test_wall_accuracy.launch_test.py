"""Measured-wall gate (§18 Phase 2): /scan range at surveyed wall bearings
within ±2 cm at 1/2/4 m, grounded standing pose, full production chain
(sidecar raycast → CropBox → pointcloud_to_laserscan), plus inline T9 and
the /scan sanity checks (H-5 uniform increments, inf handling, frame_id,
monotonic stamps). Cylinder targets at 1/2/3 m are measured and printed as
the Q-3 bin-occupancy baseline, not gated.

Self-contained: wall_state_source.py replaces the simulate binary (static
grounded qpos; the mirror is kinematic), so this runs headless on a bare
machine with python mujoco + the pinned MuJoCo-LiDAR checkout. SKIPs if
those are missing. Phase 5 reruns the same assertions against a real wall.

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_wall_accuracy.launch_test.py
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
import unittest

import launch
import launch_testing.actions
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
GEN = os.path.join(WS, 'test_fixtures', 'wall_scene', 'make_wall_scene.py')
OUT = '/tmp/wall_scene_phase2'
GT_JSON = os.path.join(OUT, 'wall_scene_gt.json')
MIRROR = os.path.join(OUT, 'wall_scene_mirror.xml')

SKIP_REASON = None
try:
    r = subprocess.run([sys.executable, GEN, '--out', OUT],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        SKIP_REASON = f'wall scene generation failed: {r.stderr[-400:]}'
except Exception as e:  # noqa: BLE001 — any env problem means SKIP, not fail
    SKIP_REASON = f'wall scene generation failed: {e}'


@pytest.mark.launch_test
def generate_test_description():
    if SKIP_REASON:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_dir = os.path.join(
        get_package_share_directory('g1_perception_bringup'), 'launch')
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[sys.executable, os.path.join(HERE, 'wall_state_source.py'),
                 '--gt-json', GT_JSON],
            output='screen'),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'source_sim.launch.py')),
            launch_arguments=[('mirror_model_path', MIRROR)]),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'description.launch.py'))),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'perception.launch.py'))),
        launch_testing.actions.ReadyToTest(),
    ])


class TestWallAccuracy(unittest.TestCase):
    N_SCANS = 25
    T9_TIMEOUT = 0.050          # §16.2-T9
    WALL_TOL = 0.02             # §18 Phase-2 gate
    WINDOW = math.radians(2.0)  # bins evaluated around each target bearing

    @classmethod
    def setUpClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        import tf2_ros

        rclpy.init()
        cls.node = rclpy.create_node('wall_gate_probe')
        cls.scans = []
        cls.cloud_stamps = []
        cls.lock = threading.Lock()
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.node.create_subscription(
            LaserScan, '/scan',
            lambda m: cls._push(cls.scans, m), qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, '/livox/lidar',
            lambda m: cls._push(cls.cloud_stamps,
                                (m.header.stamp, time.monotonic())),
            qos_profile_sensor_data)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin,
                                           daemon=True)
        cls.spin_thread.start()

        # T9 runs LIVE: a checker thread polls each cloud's transform with a
        # 50 ms deadline from receipt (not post-hoc when TF has caught up).
        cls.t9_checked = 0
        cls.t9_misses = 0
        cls._t9_stop = False
        cls.t9_thread = threading.Thread(target=cls._t9_worker, daemon=True)
        cls.t9_thread.start()

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            with cls.lock:
                if len(cls.scans) >= cls.N_SCANS:
                    break
            time.sleep(0.2)
        cls._t9_stop = True
        cls.t9_thread.join(timeout=2.0)

    @classmethod
    def _t9_worker(cls):
        # Clouds that arrive before this probe's /tf subscription has finished
        # DDS discovery can never resolve (volatile TF published pre-match is
        # lost to us though it WAS on the wire) — so counting starts at the
        # first successful resolution and every cloud after that must resolve
        # within 50 ms of receipt. Zero misses in steady state is the gate.
        from rclpy.time import Time
        done = 0
        warm = False
        while not cls._t9_stop:
            with cls.lock:
                pending = cls.cloud_stamps[done:]
            if not pending:
                time.sleep(0.005)
                continue
            for stamp, t_recv in pending:
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
    def _push(cls, store, item):
        with cls.lock:
            store.append(item)

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
            self.assertGreaterEqual(
                len(self.scans), self.N_SCANS,
                f'only {len(self.scans)} scans arrived — chain not running')
            self._scans = list(self.scans)
            self._clouds = list(self.cloud_stamps)
        with open(GT_JSON) as f:
            self._gt = json.load(f)

    def _target_error(self, scan, bearing_deg, expected):
        """Median (measured − expected-at-bin) over finite bins in WINDOW."""
        b = math.radians(bearing_deg)
        errs = []
        n = len(scan.ranges)
        for i in range(n):
            theta = scan.angle_min + (i + 0.5) * scan.angle_increment
            d = math.atan2(math.sin(theta - b), math.cos(theta - b))
            if abs(d) > self.WINDOW:
                continue
            r = scan.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            errs.append(r - expected / math.cos(d))
        errs.sort()
        return errs[len(errs) // 2] if errs else None

    def test_scan_sanity(self):
        """H-5: uniform increments; frame; monotonic stamps; inf handling."""
        first = self._scans[0]
        self.assertEqual(first.header.frame_id, 'base_footprint')
        self.assertAlmostEqual(first.angle_increment, 0.0058, places=9)
        prev = None
        for s in self._scans:
            self.assertEqual(len(s.ranges), len(first.ranges))
            self.assertAlmostEqual(s.angle_increment, first.angle_increment,
                                   places=12)
            t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
            if prev is not None:
                self.assertGreater(t, prev, 'non-monotonic /scan stamps')
            prev = t
            n_inf = 0
            for r in s.ranges:
                self.assertFalse(math.isnan(r), 'NaN in /scan')
                if math.isinf(r):
                    n_inf += 1
                else:
                    self.assertGreaterEqual(r, s.range_min)
                    self.assertLessEqual(r, s.range_max)
            self.assertGreater(n_inf, 0, 'use_inf: empty bins must be +inf')

    def test_wall_accuracy(self):
        """±2 cm at surveyed walls, 1/2/4 m (the Phase-2 gate)."""
        report = []
        for tgt in self._gt['targets']:
            per_scan = [e for s in self._scans
                        if (e := self._target_error(
                            s, tgt['bearing_deg'], tgt['range_m'])) is not None]
            self.assertTrue(per_scan, f"{tgt['name']}: never seen in /scan")
            per_scan.sort()
            med = per_scan[len(per_scan) // 2]
            report.append((tgt['name'], tgt['kind'], tgt['range_m'], med,
                           len(per_scan)))
            if tgt['kind'] == 'wall':
                self.assertLessEqual(
                    abs(med), self.WALL_TOL,
                    f"{tgt['name']}: median error {med * 100:.2f} cm "
                    f'exceeds ±{self.WALL_TOL * 100:.0f} cm')
        print('\n=== measured-wall gate ===')
        for name, kind, rng, med, n in report:
            print(f'{name:8s} {kind:8s} GT {rng:.3f} m  '
                  f'median err {med * 1000:+7.2f} mm  ({n} scans)')

    def test_t9_tf_availability(self):
        """T9: odom->base_footprint resolves within 50 ms of every cloud."""
        self.assertGreater(self.t9_checked, 0,
                           'no /livox/lidar clouds were T9-checked')
        print(f'T9: {self.t9_checked} clouds checked live, '
              f'{self.t9_misses} misses')
        self.assertEqual(self.t9_misses, 0,
                         f'T9 FAIL: {self.t9_misses} TF misses')
