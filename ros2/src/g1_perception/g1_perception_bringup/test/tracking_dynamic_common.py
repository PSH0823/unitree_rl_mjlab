"""Shared T5 machinery (§16.2, Phase 3): replay an S2 crosser fixture through
the perception container and assert velocity RMSE / ID stability against the
in-bag /sim/gt_obstacles ground truth.

Confirmation boundary: H-7's "confirm after 3 hits" — velocity is scored from
3 measurement frames after the crosser is first matched (uid identity is not
gated on it; DPCBF treats id as informational, §2.3-6).
"""
import math
import os
import threading
import time
import unittest

import launch
import launch_testing.actions

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))

MATCH_R = 0.5
CONFIRM_HITS = 3          # H-7 / Appendix A
VEL_RMSE_GATE = 0.1       # T5 / H-11
MAX_SWAPS_PER_10S = 1.0   # T5
CROSSER_UID_GT = 3        # mocap index of s2_crosser in the scenario json


def make_description(bag, skip_reason):
    if skip_reason:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_dir = os.path.join(
        get_package_share_directory('g1_perception_bringup'), 'launch')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'perception.launch.py'))),
        launch.actions.TimerAction(
            period=4.0,
            actions=[launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag], output='screen')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TrackingDynamicBase(unittest.TestCase):
    SKIP_REASON = None     # set by subclass
    SPEED = None           # commanded crossing speed [m/s]
    COLLECT_S = None       # bag duration + margin

    @classmethod
    def setUpClass(cls):
        if cls.SKIP_REASON:
            return
        import rclpy
        from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                               QoSHistoryPolicy)
        from obstacle_detector.msg import Obstacles

        rclpy.init()
        cls.node = rclpy.create_node('t5_probe')
        cls.lock = threading.Lock()
        cls.tracked = []
        cls.gt = []
        cls.node.create_subscription(
            Obstacles, '/tracked_obstacles',
            lambda m: cls._push(cls.tracked, m), QoSProfile(depth=5))
        cls.node.create_subscription(
            Obstacles, '/sim/gt_obstacles',
            lambda m: cls._push(cls.gt, m),
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
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
        if cls.SKIP_REASON:
            return
        import rclpy
        cls.executor.shutdown()
        rclpy.shutdown()

    # -- GT interpolation ---------------------------------------------------
    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _gt_samples(self):
        out = []
        with self.lock:
            for m in self.gt:
                for c in m.circles:
                    if c.uid == CROSSER_UID_GT:
                        out.append((self._stamp(m), c.center.x, c.center.y,
                                    c.velocity.x, c.velocity.y))
        out.sort()
        return out

    def _gt_at(self, gts, t):
        if not gts or t < gts[0][0] or t > gts[-1][0]:
            return None
        lo, hi = 0, len(gts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if gts[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, vx0, vy0 = gts[lo]
        t1, x1, y1, _, _ = gts[hi]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), vx0, vy0)

    def _matches(self):
        gts = self._gt_samples()
        self.assertTrue(gts, 'no GT crosser samples in bag replay')
        out = []
        with self.lock:
            tracked = list(self.tracked)
        for m in tracked:
            g = self._gt_at(gts, self._stamp(m))
            if g is None:
                continue
            gx, gy, gvx, gvy = g
            best, bd = None, MATCH_R
            for c in m.circles:
                d = math.hypot(c.center.x - gx, c.center.y - gy)
                if d < bd:
                    best, bd = c, d
            if best:
                out.append((self._stamp(m), best.uid,
                            best.velocity.x - gvx, best.velocity.y - gvy,
                            math.hypot(gvx, gvy) > 1e-6))
        return out

    # -- assertions ---------------------------------------------------------
    def _skip_if_needed(self):
        if self.SKIP_REASON:
            self.skipTest(self.SKIP_REASON)

    def test_tracker_confirms_and_velocity_sane(self):
        self._skip_if_needed()
        matches = self._matches()
        self.assertGreater(len(matches), 50,
                           'crosser rarely matched — tracker broken?')
        # sign/magnitude sanity while GT moves along +y at SPEED
        moving = [m for m in matches if m[4]][len(matches) // 4:]
        self.assertTrue(moving)
        # velocity error already relative to GT; the estimate itself must be
        # non-trivial (not stuck at zero) — check via error magnitude bound
        stuck = sum(1 for m in moving
                    if math.hypot(m[2], m[3]) > 0.9 * self.SPEED)
        self.assertLess(stuck / len(moving), 0.1,
                        'velocity estimate stuck near zero for the crosser')

    def test_t5_velocity_rmse_after_confirmation(self):
        self._skip_if_needed()
        matches = self._matches()
        conf_t = matches[0][0] + CONFIRM_HITS * 0.1
        scored = [m for m in matches if m[0] >= conf_t and m[4]]
        self.assertGreater(len(scored), 50)
        rmse = math.sqrt(sum(m[2] ** 2 + m[3] ** 2
                             for m in scored) / len(scored))
        print(f'[T5] speed {self.SPEED}: velocity RMSE {rmse:.3f} m/s '
              f'over {len(scored)} frames')
        self.assertLess(rmse, VEL_RMSE_GATE,
                        f'T5 velocity RMSE {rmse:.3f} >= {VEL_RMSE_GATE}')

    def test_t5_id_stability(self):
        self._skip_if_needed()
        matches = self._matches()
        uids = []
        for m in matches:
            if not uids or uids[-1] != m[1]:
                uids.append(m[1])
        dur = matches[-1][0] - matches[0][0]
        swaps_per_10s = (len(uids) - 1) / dur * 10.0 if dur > 0 else 0.0
        print(f'[T5] uid sequence {uids} over {dur:.1f} s '
              f'({swaps_per_10s:.2f} swaps/10 s)')
        self.assertLessEqual(swaps_per_10s, MAX_SWAPS_PER_10S)
