#!/usr/bin/env python3
"""Walking visual evidence: LiDAR cloud + tracked overlay (carried since Phase 1).

The interactive RViz screenshot has been blocked since Phase 1 — this machine
runs the simulator against Xvfb with software GL and there is no one at a
display to drive RViz. This is the honest substitute and it shows strictly
more: an OFFSCREEN render (matplotlib Agg, no display, no RViz) of the same
three layers RViz would show, sampled live from the running walking stack —

  * /scan          the projected LiDAR return, in the odom frame
  * /tracked_obstacles  perception's circles (what DPCBF is told)
  * /sim/gt_obstacles   the simulator's exact circles (what is really there)
  * /sim/mj_state       the robot base, with its p_max horizon

so the overlay is also the tracked-vs-GT error, visible per obstacle rather
than only as a percentile. Panels are taken at even intervals through the
walking window, plus a whole-run trajectory panel.

  walk_overlay.py <seconds> <out.png> [--settle 45] [--panels 4] [--pmax 3.0]
"""
import math
import sys
import time

import matplotlib
matplotlib.use('Agg')            # offscreen: no display, no RViz, no GL
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as MplCircle

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from sim_msgs.msg import MjState
from obstacle_detector.msg import Obstacles


def arg(name, default):
    return type(default)(sys.argv[sys.argv.index(name) + 1]) \
        if name in sys.argv else default


class Collector(Node):
    def __init__(self):
        super().__init__('walk_overlay')
        self.frames = []
        self.scan = None
        self.gt = []
        self.tr = []
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, best)
        self.create_subscription(Obstacles, '/sim/gt_obstacles',
                                 lambda m: setattr(self, 'gt', self.circles(m)), rel)
        self.create_subscription(Obstacles, '/tracked_obstacles',
                                 lambda m: setattr(self, 'tr', self.circles(m)), rel)
        self.create_subscription(MjState, '/sim/mj_state', self.on_state, best)

    @staticmethod
    def circles(m):
        return [(c.center.x, c.center.y, c.true_radius or c.radius)
                for c in m.circles]

    def on_scan(self, m):
        self.scan = m

    def on_state(self, m):
        q = list(m.qpos)
        if len(q) < 7 or self.scan is None:
            return
        w, x, y, z = q[3], q[4], q[5], q[6]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # /scan is in base_footprint; put it in odom with the base pose so all
        # four layers share one frame (that is what makes the overlay legible).
        pts = []
        s = self.scan
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            a = s.angle_min + i * s.angle_increment + yaw
            pts.append((q[0] + r * math.cos(a), q[1] + r * math.sin(a)))
        self.frames.append(dict(t=m.sim_time, x=q[0], y=q[1], yaw=yaw,
                                pts=pts, gt=list(self.gt), tr=list(self.tr)))


def main():
    secs = float(sys.argv[1])
    out = sys.argv[2]
    settle = arg('--settle', 45.0)
    panels = arg('--panels', 4)
    pmax = arg('--pmax', 3.0)

    rclpy.init()
    node = Collector()
    t_end = time.time() + secs
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames = [f for f in node.frames if f['t'] >= settle and f['pts']]
    node.destroy_node()
    rclpy.shutdown()

    if len(frames) < panels:
        print(f'only {len(frames)} usable frames after settle={settle}')
        return 1

    fig, axes = plt.subplots(1, panels + 1, figsize=(5 * (panels + 1), 5.4))
    picks = [frames[int(i * (len(frames) - 1) / max(1, panels - 1))]
             for i in range(panels)]
    for ax, f in zip(axes[:panels], picks):
        if f['pts']:
            ax.scatter([p[0] for p in f['pts']], [p[1] for p in f['pts']],
                       s=1.2, c='#888888', label='/scan')
        for cx, cy, cr in f['gt']:
            if math.hypot(cx - f['x'], cy - f['y']) < pmax + 1.5:
                ax.add_patch(MplCircle((cx, cy), cr, fill=False,
                                       ec='#1f77b4', lw=1.6, ls='--'))
        for cx, cy, cr in f['tr']:
            if math.hypot(cx - f['x'], cy - f['y']) < pmax + 1.5:
                ax.add_patch(MplCircle((cx, cy), cr, fill=False,
                                       ec='#d62728', lw=1.6))
        ax.add_patch(MplCircle((f['x'], f['y']), pmax, fill=False,
                               ec='#2ca02c', lw=1.0, ls=':'))
        ax.plot([f['x']], [f['y']], marker='o', ms=7, color='#2ca02c')
        ax.arrow(f['x'], f['y'], 0.6 * math.cos(f['yaw']),
                 0.6 * math.sin(f['yaw']), width=0.04, color='#2ca02c')
        ax.set_xlim(f['x'] - pmax - 1.2, f['x'] + pmax + 1.2)
        ax.set_ylim(f['y'] - pmax - 1.2, f['y'] + pmax + 1.2)
        ax.set_aspect('equal')
        ax.set_title(f't = {f["t"]:.1f} s   ({len(f["tr"])} tracked, '
                     f'{len(f["gt"])} GT)', fontsize=10)
        ax.set_xlabel('odom x [m]')

    ax = axes[panels]
    ax.plot([f['x'] for f in frames], [f['y'] for f in frames],
            color='#2ca02c', lw=1.5)
    for cx, cy, cr in frames[-1]['gt']:
        ax.add_patch(MplCircle((cx, cy), cr, fill=False, ec='#1f77b4', lw=1.0,
                               ls='--'))
    ax.set_aspect('equal')
    ax.set_title(f'base trajectory, {frames[0]["t"]:.0f}-{frames[-1]["t"]:.0f} s',
                 fontsize=10)
    ax.set_xlabel('odom x [m]')
    ax.set_ylabel('odom y [m]')

    axes[0].set_ylabel('odom y [m]')
    fig.suptitle('G1 walking under policy — /scan (grey), tracked obstacles '
                 '(red, solid), GT obstacles (blue, dashed), robot + p_max '
                 '(green).  Offscreen matplotlib render, no RViz.',
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out, dpi=110)
    print(f'wrote {out} from {len(frames)} frames '
          f'({frames[0]["t"]:.1f}..{frames[-1]["t"]:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
