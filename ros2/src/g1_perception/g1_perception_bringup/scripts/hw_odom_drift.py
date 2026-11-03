#!/usr/bin/python3
"""Stationary DLIO drift check (Phase 5C, stage 5), live on the console.

The stage-5 criterion is `‖p(t) − p(0)‖ / t < 1 cm/min` with the robot NOT
moving, plus "no pose jumps". Until now that number could only be produced
offline from the bag — which means the operator standing next to a robot had
no way to tell a healthy initialisation from a diverging one while it was
happening. This node computes it live:

    t+ 45.0 s   drift 0.004 m (0.5 cm/min)   yaw 0.06 deg   jump_max 0.001 m
                /odom 100.2 Hz   stamps ok

Two traps it is built around:

  * **`/odom` existing is not odometry.** DLIO publishes it as soon as it
    starts and stamps it with the IMU stamp, which is 0 until the first IMU
    message arrives. Samples with a zero stamp are counted and excluded, and
    the summary says so — otherwise "0 cm/min drift" can mean "perfect" or
    "nothing is running", which are not the same result.
  * **the first seconds are not representative.** DLIO calibrates IMU bias
    over `odom/imu/calibration/time: 3.0` s and drops its first ~30 clouds
    against an unfilled transform cache. `settle` (default 5 s) excludes that
    window from the reference pose, so a normal startup transient does not
    read as drift.

Publishes nothing; subscribes to /odom only. Cannot move the robot.

Usage
  ros2 run g1_perception_bringup hw_odom_drift.py --ros-args \
      -p duration:=60.0 -p json:=$SESSION/stage5_drift.json

Parameters
  topic       odometry topic (default /odom)
  duration    measurement window in seconds AFTER settle (default 60.0)
  settle      seconds ignored before taking the reference pose (default 5.0)
  max_drift   pass threshold in cm/min (default 1.0 — the stage-5 criterion)
  max_jump    pass threshold on a single sample-to-sample step, m (default 0.05)
  json        summary JSON path; '' = off

Exit 0 pass, 1 threshold violated, 2 no usable /odom.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry


def _yaw(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class OdomDrift(Node):
    def __init__(self):
        super().__init__('hw_odom_drift')
        topic = self.declare_parameter('topic', '/odom').value
        self.duration = float(self.declare_parameter('duration', 60.0).value)
        self.settle = float(self.declare_parameter('settle', 5.0).value)
        self.max_drift = float(self.declare_parameter('max_drift', 1.0).value)
        self.max_jump = float(self.declare_parameter('max_jump', 0.05).value)
        self.json_path = self.declare_parameter('json', '').value

        self.t0 = time.time()
        self.ref = None          # (x, y, z, yaw) reference pose
        self.ref_t = None
        self.cur = None
        self.prev = None
        self.jump_max = 0.0
        self.n = 0               # samples with a usable stamp
        self.n_zero_stamp = 0
        self.drift_max = 0.0     # worst instantaneous rate seen, cm/min

        # BEST_EFFORT/sensor QoS: DLIO's /odom publisher is RELIABLE, but a
        # sensor-data subscription is compatible with both and never blocks
        # the producer — this node must not be able to slow odometry down.
        self.create_subscription(Odometry, topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            'watching %s: settle %.1f s, then %.1f s stationary — DO NOT '
            'TOUCH THE ROBOT' % (topic, self.settle, self.duration))

    def _on_odom(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.n_zero_stamp += 1
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, p.z, _yaw(q))
        self.n += 1
        now = time.time()
        if self.ref is None:
            if now - self.t0 < self.settle:
                return
            self.ref, self.ref_t = pose, now
            self.get_logger().info(
                'reference pose (%.3f, %.3f, %.3f) yaw %.2f deg'
                % pose)
        if self.prev is not None:
            step = math.dist(pose[:3], self.prev[:3])
            self.jump_max = max(self.jump_max, step)
        self.prev = pose
        self.cur = pose

    def _stats(self):
        if self.ref is None or self.cur is None:
            return None
        dt = max(time.time() - self.ref_t, 1e-6)
        d = math.dist(self.cur[:3], self.ref[:3])
        return {'elapsed': dt, 'drift_m': d,
                'drift_cm_per_min': d * 100.0 / (dt / 60.0),
                'dyaw_deg': self.cur[3] - self.ref[3],
                'jump_max_m': self.jump_max,
                'hz': self.n / max(time.time() - self.t0, 1e-6),
                'samples': self.n, 'zero_stamp_samples': self.n_zero_stamp}

    def _tick(self):
        s = self._stats()
        if s is None:
            waited = time.time() - self.t0
            print('  waiting for %s ... %.0f s (%d sample(s), %d with stamp 0)'
                  % ('/odom', waited, self.n, self.n_zero_stamp), flush=True)
            if waited > self.settle + self.duration:
                raise SystemExit(2)
            return
        # After ~10 s the instantaneous rate stops being dominated by the
        # first sample's quantisation; before that it swings wildly and a max
        # taken over it means nothing.
        if s['elapsed'] > 10.0:
            self.drift_max = max(self.drift_max, s['drift_cm_per_min'])
        print('  t+%5.1f s   drift %.4f m (%.2f cm/min)   yaw %+.2f deg   '
              'jump_max %.3f m   %s %.1f Hz%s'
              % (s['elapsed'], s['drift_m'], s['drift_cm_per_min'],
                 s['dyaw_deg'], s['jump_max_m'], '/odom', s['hz'],
                 '' if not self.n_zero_stamp
                 else '   [%d stamp-0 dropped]' % self.n_zero_stamp),
              flush=True)
        if s['elapsed'] >= self.duration:
            raise SystemExit(self._report(s))

    def _report(self, s):
        s['drift_cm_per_min_max'] = self.drift_max
        s['thresholds'] = {'max_drift_cm_per_min': self.max_drift,
                           'max_jump_m': self.max_jump}
        bad = []
        if s['drift_cm_per_min'] > self.max_drift:
            bad.append('drift %.2f cm/min > %.2f'
                       % (s['drift_cm_per_min'], self.max_drift))
        if s['jump_max_m'] > self.max_jump:
            bad.append('pose jump %.3f m > %.3f'
                       % (s['jump_max_m'], self.max_jump))
        if s['samples'] < 10:
            bad.append('only %d stamped sample(s)' % s['samples'])
        s['verdict'] = 'PASS' if not bad else 'FAIL'
        s['problems'] = bad
        print('\n=== stationary odometry summary (%.0f s)' % s['elapsed'])
        print('  total drift          %.4f m' % s['drift_m'])
        print('  drift rate           %.2f cm/min (worst %.2f)'
              % (s['drift_cm_per_min'], self.drift_max))
        print('  yaw drift            %+.2f deg' % s['dyaw_deg'])
        print('  largest single step  %.4f m' % s['jump_max_m'])
        print('  samples              %d stamped, %d with stamp 0'
              % (s['samples'], s['zero_stamp_samples']))
        print('  %s%s' % (s['verdict'],
                          '' if not bad else ': ' + '; '.join(bad)))
        if self.json_path:
            with open(self.json_path, 'w') as f:
                json.dump(s, f, indent=2)
            print('  wrote %s' % self.json_path)
        return 0 if not bad else 1


def main():
    rclpy.init()
    node = OdomDrift()
    code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = getattr(exc, 'code', 0) or 0
    finally:
        if node.n == 0:
            print('\nNO STAMPED /odom — DLIO is not producing odometry '
                  '(stage 5 territory).', file=sys.stderr)
            code = code or 2
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
