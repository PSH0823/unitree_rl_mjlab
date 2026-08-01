"""T4 static-obstacle accuracy gate (§16.2, Phase 3) + extractor bag-driven
integration: replays the s1_surveyed fixture (3 surveyed cylinders at
1/2/3 m, grounded pose) through the full perception container and asserts,
on /raw_obstacles and /tracked_obstacles:

  - frames are odom; circle counts sane vs GT (3 matched, no extras in p_max)
  - T4: center error <= 0.10 m, true_radius error <= 0.05 m (pre-inflation —
    measured on /tracked_obstacles; /obstacles_safe does not exist until
    Phase 4, recorded as a T4 adaptation in the §21 log)
  - detection latency <= 2 scan frames

SKIPs when the fixture bag is absent (gitignored; regenerate per
test_fixtures/README.md).

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_detection_static.launch_test.py
"""

# Concurrent launch tests share topic names; give this one a private DDS
# domain before anything else touches ROS (see isolate_domain.py).
# launch_test loads this file by path without putting its directory on
# sys.path, so the sibling import needs the insert first.
import os as _os, sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E402
import isolate_domain  # noqa: E402
isolate_domain.isolate(isolate_domain.DETECTION_STATIC)  # noqa: E402
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
BAG = os.path.join(WS, 'test_fixtures', 's1_surveyed')

SKIP_REASON = None

# --- one harness, two worlds (D4; Phase 5A) --------------------------------
# By default this is the sim T4 gate on the committed s1_surveyed fixture.
# Point it at a hardware session instead with:
#
#   T4_BAG=<bag dir> T4_LAYOUT=<layout.yaml> T4_USE_SIM_TIME=false \
#     launch_test test/test_detection_static.launch_test.py
#
# layout.yaml (surveyed, odom frame — see doc/phase5b_checklists.md block 5):
#   match_radius: 0.5          # optional, defaults to §17.2's 0.5 m
#   targets:
#     - {name: cyl_1m, x: 0.813, y: 0.813, r: 0.15}
#     - {name: blocker_2m, x: -1.52, y: 1.52, r: 0.30}
#
# Per-target radii are honoured, so a layout mixing r=0.15 with r>=0.30 props
# measures the Phase-4 "radius bias grows with radius" finding directly: the
# per-target true_radius error is printed for every target, not just gated.
BAG = os.environ.get('T4_BAG', BAG)
USE_SIM_TIME = os.environ.get('T4_USE_SIM_TIME', 'true')
_LAYOUT = os.environ.get('T4_LAYOUT', '')

# Default GT: cylinders r=0.15, faces at 1/2/3 m, bearings 45/135/-45 deg
# (test_fixtures/scenarios/make_scenario_scene.py)
GT = {
    's1_cyl_1m': (1.15 * math.cos(math.radians(45)),
                  1.15 * math.sin(math.radians(45))),
    's1_cyl_2m': (2.15 * math.cos(math.radians(135)),
                  2.15 * math.sin(math.radians(135))),
    's1_cyl_3m': (3.15 * math.cos(math.radians(-45)),
                  3.15 * math.sin(math.radians(-45))),
}
GT_R = {name: 0.15 for name in GT}
MATCH_R = 0.5        # §17.2 NN matching radius

if _LAYOUT:
    import yaml as _yaml
    with open(_LAYOUT) as _f:
        _spec = _yaml.safe_load(_f)
    GT = {t['name']: (float(t['x']), float(t['y'])) for t in _spec['targets']}
    GT_R = {t['name']: float(t['r']) for t in _spec['targets']}
    MATCH_R = float(_spec.get('match_radius', MATCH_R))

if not os.path.isdir(BAG):
    SKIP_REASON = f'fixture bag missing: {BAG} (gitignored; see README)'
CENTER_TOL = 0.10    # T4
RADIUS_TOL = 0.05    # T4, pre-inflation (true_radius)
LATENCY_FRAMES = 2   # T4
P_MAX = 3.0


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
                os.path.join(launch_dir, 'perception.launch.py')),
            launch_arguments=[('use_sim_time', USE_SIM_TIME)]),
        launch.actions.TimerAction(
            period=4.0,
            actions=[launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'play', BAG], output='screen')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDetectionStatic(unittest.TestCase):
    COLLECT_S = 40.0  # bag is ~30 s + startup margin

    @classmethod
    def setUpClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        from rclpy.qos import QoSProfile
        from obstacle_detector.msg import Obstacles

        rclpy.init()
        cls.node = rclpy.create_node('t4_probe')
        cls.lock = threading.Lock()
        cls.raw = []
        cls.tracked = []
        cls.node.create_subscription(
            Obstacles, '/raw_obstacles',
            lambda m: cls._push(cls.raw, m), QoSProfile(depth=5))
        cls.node.create_subscription(
            Obstacles, '/tracked_obstacles',
            lambda m: cls._push(cls.tracked, m), QoSProfile(depth=5))
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()
        deadline = time.monotonic() + cls.COLLECT_S
        while time.monotonic() < deadline:
            time.sleep(0.5)

    @classmethod
    def _push(cls, buf, m):
        with cls.lock:
            buf.append(m)

    @classmethod
    def tearDownClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        cls.executor.shutdown()
        rclpy.shutdown()

    def _skip_if_needed(self):
        if SKIP_REASON:
            self.skipTest(SKIP_REASON)

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _match_stats(self, msgs):
        """Per-GT center/true_radius errors + off-GT circles inside p_max."""
        err = {n: [] for n in GT}
        extras = 0
        for m in msgs:
            for c in m.circles:
                best, bd = None, MATCH_R
                for name, (gx, gy) in GT.items():
                    d = math.hypot(c.center.x - gx, c.center.y - gy)
                    if d < bd:
                        best, bd = name, d
                if best:
                    err[best].append((bd, c.true_radius - GT_R[best]))
                elif math.hypot(c.center.x, c.center.y) < P_MAX:
                    extras += 1
        return err, extras

    def test_raw_obstacles_frame_and_counts(self):
        self._skip_if_needed()
        with self.lock:
            raw = list(self.raw)
        self.assertGreater(len(raw), 200, 'too few /raw_obstacles messages')
        for m in raw:
            self.assertEqual(m.header.frame_id, 'odom')
        err, extras = self._match_stats(raw)
        for name in GT:
            frac = len(err[name]) / len(raw)
            self.assertGreater(
                frac, 0.98, f'{name} missing from raw circles ({frac:.2%})')
        self.assertEqual(extras, 0,
                         'unexpected raw circles inside p_max (phantom?)')

    def test_t4_accuracy_on_tracked(self):
        self._skip_if_needed()
        with self.lock:
            tracked = list(self.tracked)
        self.assertGreater(len(tracked), 200)
        err, extras = self._match_stats(tracked)
        for name in GT:
            self.assertTrue(err[name], f'{name} never tracked')
            ce = [e[0] for e in err[name]]
            re = [e[1] for e in err[name]]
            mean_ce = sum(ce) / len(ce)
            mean_re = sum(re) / len(re)
            # Printed per target so a mixed-radius layout shows the Phase-4
            # radius-dependent bias directly (r>=0.30 props, 5B block 5).
            print(f'T4 {name} (r_gt {GT_R[name]:.2f}): center mean '
                  f'{mean_ce * 1e3:+.1f} mm max {max(ce) * 1e3:.1f} mm; '
                  f'true_radius mean {mean_re * 1e3:+.1f} mm '
                  f'({len(ce)} observations)')
            self.assertLess(mean_ce, CENTER_TOL,
                            f'{name} center error {mean_ce:.3f} m')
            self.assertLess(abs(mean_re), RADIUS_TOL,
                            f'{name} true_radius error {mean_re:+.3f} m')
        self.assertEqual(extras, 0,
                         'unexpected tracked circles inside p_max')

    def test_t4_detection_latency(self):
        self._skip_if_needed()
        with self.lock:
            raw = list(self.raw)
            tracked = list(self.tracked)
        t0 = self._stamp(raw[0])
        for name, (gx, gy) in GT.items():
            first = None
            for m in tracked:
                if any(math.hypot(c.center.x - gx, c.center.y - gy) < MATCH_R
                       for c in m.circles):
                    first = self._stamp(m)
                    break
            self.assertIsNotNone(first, f'{name} never tracked')
            frames = round((first - t0) * 10)  # 10 Hz scans
            self.assertLessEqual(frames, LATENCY_FRAMES,
                                 f'{name} latency {frames} frames')
