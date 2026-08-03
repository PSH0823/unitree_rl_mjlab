#!/usr/bin/env python3
"""Base-motion probe for the walking gate (workstream B).

Subscribes /sim/mj_state (the §11.2 mirror state) and reports what the robot
base actually did, in the terms the walking bar is written in: "stable
locomotion under a scripted command profile, long enough to run the A/B" —
standing, twitching or being dragged is not walking.

qpos[0:3] is the pelvis free-joint translation and qpos[3:7] its (w x y z)
quaternion, so heading, height and tilt come straight out of the mirror
without touching the simulator.

  walk_state_probe.py <seconds> <out.jsonl> [--settle T0]

--settle skips the first T0 sim-seconds (spawn + FixStand) from the summary
statistics; the trace always covers everything.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sim_msgs.msg import MjState


def rpy(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


class Probe(Node):
    def __init__(self, out):
        super().__init__('walk_state_probe')
        self.rows = []
        self.out = out
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(MjState, '/sim/mj_state', self.cb, qos)

    def cb(self, m):
        q = list(m.qpos)
        if len(q) < 7:
            return
        r, p, y = rpy(q[3], q[4], q[5], q[6])
        self.rows.append(dict(t=m.sim_time, x=q[0], y=q[1], z=q[2],
                              roll=r, pitch=p, yaw=y))


def main():
    secs = float(sys.argv[1])
    out = sys.argv[2]
    settle = 0.0
    if '--settle' in sys.argv:
        settle = float(sys.argv[sys.argv.index('--settle') + 1])

    rclpy.init()
    node = Probe(out)
    t_end = time.time() + secs
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    with open(out, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')

    w = [r for r in rows if r['t'] >= settle]
    print(f'samples {len(rows)} total, {len(w)} after settle={settle:.1f}s')
    if len(w) < 10:
        print('WALKING: NO — too few samples after settle')
        return 1

    dt = w[-1]['t'] - w[0]['t']
    dx = w[-1]['x'] - w[0]['x']
    dy = w[-1]['y'] - w[0]['y']
    net = math.hypot(dx, dy)
    path = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(w, w[1:]))
    zs = [r['z'] for r in w]
    tilt = [math.hypot(r['roll'], r['pitch']) for r in w]
    # Per-sample forward speed, for the "did it actually keep moving" question
    # (a robot dragged once and then stuck has net displacement but no speed).
    v = [math.hypot(b['x'] - a['x'], b['y'] - a['y']) / max(1e-6, b['t'] - a['t'])
         for a, b in zip(w, w[1:])]
    v.sort()

    print(f'window          {w[0]["t"]:.2f} .. {w[-1]["t"]:.2f} s  ({dt:.2f} s)')
    print(f'net displacement {net:.3f} m  (dx {dx:+.3f}, dy {dy:+.3f})')
    print(f'path length      {path:.3f} m   straightness {net / max(1e-6, path):.3f}')
    print(f'mean speed       {path / max(1e-6, dt):.3f} m/s'
          f'   v p50 {v[len(v)//2]:.3f}  p90 {v[int(len(v)*0.9)]:.3f}')
    print(f'pelvis height    mean {sum(zs)/len(zs):.3f}  min {min(zs):.3f}'
          f'  max {max(zs):.3f} m')
    print(f'tilt |roll,pitch| max {max(tilt):.3f} rad  '
          f'(bad_orientation threshold 1.0)')

    walking = (min(zs) > 0.5 and max(tilt) < 0.5 and
               path / max(1e-6, dt) > 0.05 and dt > 5.0)
    print('WALKING: ' + ('YES' if walking else 'NO'))
    return 0 if walking else 1


if __name__ == '__main__':
    sys.exit(main())
