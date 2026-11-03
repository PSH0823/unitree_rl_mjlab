#!/usr/bin/env python3
"""Closed-loop walking A/B probe — the §17.3 safety metrics (workstream B).

The Phase-4 offline harness (`phase4_ab_run.sh`) pins the robot at (0,0,0)
and can only score command tracking; collision rate and min-clearance are
undefined for a robot that never moves. This probe measures them on the
LIVE walking closed loop, where the simulator owns both ground truths:

  /sim/mj_state      robot base pose (qpos free joint)
  /sim/gt_obstacles  exact obstacle centres/radii/velocities (odom frame)
  /tracked_obstacles perception's estimate of the same field
  /obstacles_safe    what the adapter actually hands DPCBF

Reported per run:
  * clearance(t) = min_i(|p_rob - p_i| - r_rob - r_i), the §10.1 safety
    margin DPCBF is defending; negative = the robot's safety disc is inside
    an obstacle's.
  * collision rate as BOTH a time fraction and a count of distinct entries
    into clearance < 0 (a single long contact is one event, not thousands).
  * min-clearance distribution (p0/p1/p5/p50).
  * tracked -> nearest-GT distances (the shadow-mode delta, re-priced on a
    walking base instead of the suspension rig).

Only obstacles within `--pmax` of the robot are scored: a filter that never
sees an obstacle cannot be blamed for it, and §10 culls beyond p_max.

  walk_ab_probe.py <seconds> <out_prefix> [--settle T0] [--pmax 3.0]
                   [--rrob 0.30]
"""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sim_msgs.msg import MjState
from obstacle_detector.msg import Obstacles


def arg(name, default):
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


def pct(xs, q):
    if not xs:
        return float('nan')
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


class Probe(Node):
    def __init__(self):
        super().__init__('walk_ab_probe')
        self.base = None
        self.gt = []
        self.tracked = []
        self.safe = []
        self.rows = []
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=5)
        self.create_subscription(MjState, '/sim/mj_state', self.on_state, best)
        self.create_subscription(Obstacles, '/sim/gt_obstacles', self.on_gt, rel)
        self.create_subscription(Obstacles, '/tracked_obstacles',
                                 self.on_tracked, rel)
        self.create_subscription(Obstacles, '/obstacles_safe', self.on_safe, rel)

    @staticmethod
    def circles(msg):
        return [(c.center.x, c.center.y, c.radius) for c in msg.circles]

    def on_gt(self, m):
        self.gt = self.circles(m)

    def on_tracked(self, m):
        self.tracked = self.circles(m)

    def on_safe(self, m):
        self.safe = self.circles(m)

    def on_state(self, m):
        q = list(m.qpos)
        if len(q) < 7:
            return
        w, x, y, z = q[3], q[4], q[5], q[6]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        self.rows.append(dict(t=m.sim_time, x=q[0], y=q[1], z=q[2], yaw=yaw,
                              tilt=math.hypot(roll, math.asin(s)),
                              gt=list(self.gt), tr=list(self.tracked),
                              sf=list(self.safe)))


def main():
    secs = float(sys.argv[1])
    prefix = sys.argv[2]
    settle = arg('--settle', 0.0)
    pmax = arg('--pmax', 3.0)
    r_rob = arg('--rrob', 0.30)

    rclpy.init()
    node = Probe()
    import time
    t_end = time.time() + secs
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    # Detect the fall over the WHOLE trace, before the settle filter — a robot
    # that went down during bring-up must be reported with the time it fell,
    # not with the time the scoring window happened to open.
    fell_t = next((r['t'] for r in rows if r['tilt'] > 0.6 or r['z'] < 0.5), None)

    # Always dump the raw base trace, scored or not: a run that fell during
    # bring-up is exactly the one whose trace is worth reading, and filtering
    # it away leaves nothing to diagnose.
    with open(prefix + '_base.jsonl', 'w') as f:
        for r in rows:
            f.write(json.dumps(dict(t=r['t'], x=r['x'], y=r['y'], z=r['z'],
                                    yaw=r['yaw'], tilt=r['tilt'],
                                    n_gt=len(r['gt']), n_tracked=len(r['tr']),
                                    n_safe=len(r['sf']))) + '\n')
    rows = [r for r in rows if r['t'] >= settle]
    if len(rows) < 10:
        print('too few samples after settle')
        return 1

    # A fall ends the run for scoring purposes. A prone robot's pelvis is at
    # z~0.1 and its "clearance" is a number about a body lying on the floor,
    # not about a safety margin; averaging the two together would quietly
    # dilute both. The fall itself is reported as its own outcome.
    scored_rows = [r for r in rows if fell_t is None or r['t'] < fell_t]
    if not scored_rows:
        # The robot went down before the scoring window opened. That is an
        # outcome, not an error: report it as one rather than as a run with no
        # violations, which is what silently skipping it would look like.
        out = {'samples': 0, 'window_s': 0.0, 'fell_at_s': fell_t,
               'outcome': 'FELL BEFORE THE SCORING WINDOW OPENED',
               'first_sample_s': round(rows[0]['t'], 2),
               'last_sample_s': round(rows[-1]['t'], 2)}
        with open(prefix + '_metrics.json', 'w') as f:
            json.dump(out, f, indent=2)
        for k, v in out.items():
            print(f'{k:24s} {v}')
        return 1

    clearances, deltas, in_scope = [], [], []
    events, in_collision = 0, False
    n_coll = 0
    with open(prefix + '_trace.jsonl', 'w') as f:
        for r in scored_rows:
            px, py = r['x'], r['y']
            near = [(math.hypot(cx - px, cy - py), cx, cy, cr)
                    for cx, cy, cr in r['gt']
                    if math.hypot(cx - px, cy - py) <= pmax + cr]
            in_scope.append(len(near))
            cl = min((d - r_rob - cr for d, _, _, cr in near), default=float('nan'))
            if near:
                clearances.append(cl)
                if cl < 0.0:
                    n_coll += 1
                    if not in_collision:
                        events += 1
                    in_collision = True
                else:
                    in_collision = False
            # tracked -> nearest GT, scoped the same way (shadow delta)
            for tx, ty, _tr in r['tr']:
                if math.hypot(tx - px, ty - py) > pmax:
                    continue
                d = min((math.hypot(tx - gx, ty - gy) for gx, gy, _ in r['gt']),
                        default=None)
                if d is not None and d <= 0.5:      # Phase-4 NN cap
                    deltas.append(d)
            f.write(json.dumps(dict(t=r['t'], x=px, y=py, z=r['z'],
                                    yaw=r['yaw'], tilt=r['tilt'],
                                    clearance=cl, n_gt_in_scope=len(near),
                                    n_tracked=len(r['tr']),
                                    n_safe=len(r['sf']))) + '\n')

    dt = scored_rows[-1]['t'] - scored_rows[0]['t']
    path = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(scored_rows, scored_rows[1:]))
    out = {
        'samples': len(scored_rows),
        'window_s': round(dt, 2),
        'fell_at_s': round(fell_t, 2) if fell_t is not None else None,
        'path_m': round(path, 3),
        'mean_speed_mps': round(path / max(1e-6, dt), 3),
        'pelvis_z_min': round(min(r['z'] for r in scored_rows), 3),
        'tilt_max_rad': round(max(r['tilt'] for r in scored_rows), 3),
        'scored_samples': len(clearances),
        'in_scope_frac': round(len(clearances) / max(1, len(scored_rows)), 4),
        'gt_in_scope_mean': round(sum(in_scope) / len(in_scope), 2),
        # clearance < 0 means the pelvis disc (r_rob) overlaps an obstacle
        # disc: a DPCBF MARGIN violation. It is not necessarily body contact —
        # the limbs reach well outside r_rob, and a body contact shows up as
        # `fell_at_s`. Report both; do not conflate them.
        'margin_violation_time_frac': round(n_coll / max(1, len(clearances)), 6),
        'margin_violation_events': events,
        'clearance_min_m': round(min(clearances), 4) if clearances else None,
        'clearance_p01_m': round(pct(clearances, 0.01), 4) if clearances else None,
        'clearance_p05_m': round(pct(clearances, 0.05), 4) if clearances else None,
        'clearance_p50_m': round(pct(clearances, 0.50), 4) if clearances else None,
        'tracked_to_gt_n': len(deltas),
        'tracked_to_gt_p50_mm': round(1000 * pct(deltas, 0.50), 1) if deltas else None,
        'tracked_to_gt_p90_mm': round(1000 * pct(deltas, 0.90), 1) if deltas else None,
        'tracked_to_gt_p99_mm': round(1000 * pct(deltas, 0.99), 1) if deltas else None,
    }
    with open(prefix + '_metrics.json', 'w') as f:
        json.dump(out, f, indent=2)
    for k, v in out.items():
        print(f'{k:24s} {v}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
