#!/usr/bin/env python3
"""Circle-fit bias sweep: radius x range x visible-arc extent (interim phase).

Phase 3 measured the extractor's circle-fit error at r=0.15 only; Phase 4 found
r=0.30 gives +83 mm centre bias and +33 mm radius over-estimate and sized
`fixed_inflation` to cover it. This harness characterises that systematic
properly, in isolation from odometry, projection and tracking: it publishes
ANALYTIC LaserScan rays against a known cylinder straight into the real
`obstacle_extractor` and reads `/raw_obstacles` back. Ground truth is exact by
construction, so every millimetre of the difference is the fit.

Geometry (why an analytic model is expected at all): with
`circles_from_visibles`, the extractor fits a LINE to the visible arc and then
builds the circle as the circumcircle of the equilateral triangle on that chord
(`utilities/circle.h`) — radius = (sqrt(3)/3)*L, centre = chord midpoint
- (radius/2) * segment normal, and that normal points away from the sensor, so
the centre lands r_fit/2 TOWARDS the sensor.  For a cylinder of true radius r at
range d the visible arc ends at the two tangent points, so the chord would be
L = 2*r*sqrt(1-(r/d)^2) and the true centre sits r^2/d behind its midpoint.
That predicts, before any measurement:

    r_fit  = (1/sqrt(3)) * 2 * r * sqrt(1-(r/d)^2)  ~= 1.1547 * r   (d >> r)
    e_c    = -(r_fit/2) - r^2/d                                     (radial)

i.e. a radius over-estimate and a centre pulled towards the sensor, BOTH scaling
with r — which is the "bias grows with radius" Phase 4 saw. The two errors
partly cancel for containment: what safety actually cares about is

    F_req  = |e_c| - e_r

(the inflation the pair needs, phase4_containment.py's quantity with the
velocity term zero on a static target). The sweep measures all three and tests
the prediction's breakdown under arc truncation — a neighbour occluding one
side, i.e. the S3/S4 regime — where the chord shortens and the sign of the
radius error can flip to UNDER-estimate.

Usage (workspace sourced, extractor built):
    python3 circle_fit_sweep.py --out /tmp/sweep.csv
    python3 circle_fit_sweep.py --radii 0.15,0.30 --ranges 1,2,3 --arcs 1.0
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from obstacle_detector.msg import Obstacles

# Projection-stage geometry (§9.4 config, verbatim) — the scan the extractor
# actually sees in this stack.
ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
ANGLE_INCREMENT = 0.0058
RANGE_MIN = 0.3
RANGE_MAX = 5.0
SCAN_FRAME = 'base_footprint'
ODOM_FRAME = 'odom'


def synth_scan(cx, cy, r, arc_frac, one_sided, stamp, noise=0.0, rng=None):
    """Analytic ray-cast of one cylinder; returns a LaserScan.

    arc_frac < 1 truncates the visible silhouette to that fraction of its
    angular half-width (`one_sided`: all of the loss on one side, the shape a
    neighbouring obstacle actually produces; otherwise symmetric).
    """
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = SCAN_FRAME
    scan.angle_min = ANGLE_MIN
    scan.angle_max = ANGLE_MAX
    scan.angle_increment = ANGLE_INCREMENT
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = RANGE_MIN
    scan.range_max = RANGE_MAX

    d = math.hypot(cx, cy)
    theta_c = math.atan2(cy, cx)
    alpha = math.asin(min(1.0, r / d))          # silhouette angular half-width
    lo, hi = -alpha * arc_frac, alpha * arc_frac
    if one_sided:
        lo, hi = -alpha, -alpha + 2.0 * alpha * arc_frac

    n = int(round((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT)) + 1
    ranges = []
    cc = cx * cx + cy * cy - r * r
    for i in range(n):
        th = ANGLE_MIN + i * ANGLE_INCREMENT
        dth = math.atan2(math.sin(th - theta_c), math.cos(th - theta_c))
        if dth < lo or dth > hi:
            ranges.append(float('inf'))
            continue
        b = math.cos(th) * cx + math.sin(th) * cy
        disc = b * b - cc
        if disc <= 0.0:
            ranges.append(float('inf'))
            continue
        t = b - math.sqrt(disc)
        if noise and rng is not None:
            t += rng.gauss(0.0, noise)
        ranges.append(t if RANGE_MIN <= t <= RANGE_MAX else float('inf'))
    scan.ranges = ranges
    return scan


def _arc_angle(r, d, dtheta):
    """Surface arc angle (from the near point) hit by a ray dtheta off centre.

    Sine rule in the sensor-centre-hit triangle; the near intersection makes the
    angle at the hit point obtuse, so phi = asin(d*sin(dtheta)/r) - dtheta.
    At dtheta = asin(r/d) (the tangent ray) this returns acos(r/d), the full
    visible-arc half-angle, as it must.
    """
    s = min(1.0, d * math.sin(dtheta) / r)
    return math.asin(s) - dtheta


def analytic_prediction(r, d, arc_frac=1.0, one_sided=True):
    """Closed form for the equilateral-triangle construction of circle.h.

    Returns (r_fit, radial centre error). Sensor at the origin, cylinder centre
    on +x at distance d; a surface point at signed arc angle psi from the near
    point is at C + r*(-cos psi, sin psi).
    """
    alpha = math.asin(min(1.0, r / d))
    beta = _arc_angle(r, d, alpha)              # = acos(r/d)
    beta_cut = _arc_angle(r, d, alpha * arc_frac)
    if arc_frac >= 1.0:
        psi1, psi2 = -beta, beta
    elif one_sided:                             # neighbour occludes one side
        psi1, psi2 = -beta, beta_cut
    else:
        psi1, psi2 = -beta_cut, beta_cut

    p1 = (d - r * math.cos(psi1), r * math.sin(psi1))
    p2 = (d - r * math.cos(psi2), r * math.sin(psi2))
    chord = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    r_fit = chord / math.sqrt(3.0)
    mx, my = 0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1])
    m_norm = math.hypot(mx, my)
    # circle.h: centre = midpoint - (r_fit/2) * segment normal. The segment's
    # point order comes from the scan sweep, which makes that normal point AWAY
    # from the sensor — so the constructed centre lands r_fit/2 TOWARDS the
    # sensor of the chord midpoint, not behind it. (Measured, then read back out
    # of segment.h: normal() = perpendicular(last-first).)
    cx = mx - 0.5 * r_fit * mx / m_norm
    cy = my - 0.5 * r_fit * my / m_norm
    return r_fit, math.hypot(cx, cy) - d


class Sweeper(Node):
    def __init__(self):
        super().__init__('circle_fit_sweeper')
        self.pub = self.create_publisher(LaserScan, 'scan',
                                         qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Obstacles, 'raw_obstacles', self._on_obs, 10)
        self.latest = None
        self.stb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = SCAN_FRAME
        t.transform.rotation.w = 1.0            # identity: odom coords == scan coords
        self.stb.sendTransform(t)

    def _on_obs(self, msg):
        self.latest = msg

    def _spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def measure(self, cx, cy, r, arc_frac, one_sided, warmups=4):
        """One configuration -> the reply to a single, isolated scan.

        The extractor stamps /raw_obstacles with now(), not the scan stamp, so a
        reply cannot be matched to its scan by header. Instead: warm up, DRAIN
        the subscription completely, publish exactly one scan, and take what
        comes back. Without the drain the harness reads the previous
        configuration's reply — which is what produced a first pass full of
        impossible half-metre centre errors and identical rows for different arc
        fractions.
        """
        scan = synth_scan(cx, cy, r, arc_frac, one_sided,
                          self.get_clock().now().to_msg())
        for _ in range(warmups):
            self.pub.publish(scan)
            self._spin(0.06)
        # drain: spin until nothing has arrived for 250 ms
        while True:
            self.latest = None
            self._spin(0.25)
            if self.latest is None:
                break
        self.latest = None
        scan.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(scan)
        deadline = time.time() + 1.5
        while self.latest is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        got = self.latest
        if got is None or not got.circles:
            return None
        # nearest circle to ground truth
        best = min(got.circles,
                   key=lambda c: math.hypot(c.center.x - cx, c.center.y - cy))
        return best, len(got.circles)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radii', default='0.10,0.15,0.20,0.25,0.30,0.40,0.50')
    ap.add_argument('--ranges', default='1.0,1.5,2.0,2.5,3.0,4.0')
    ap.add_argument('--arcs', default='1.0,0.75,0.5')
    ap.add_argument('--one-sided', action='store_true', default=True)
    ap.add_argument('--out', default='circle_fit_sweep.csv')
    args = ap.parse_args()

    radii = [float(x) for x in args.radii.split(',')]
    ranges = [float(x) for x in args.ranges.split(',')]
    arcs = [float(x) for x in args.arcs.split(',')]

    rclpy.init()
    node = Sweeper()
    rows = []
    for r in radii:
        for d in ranges:
            if r / d >= 0.95:
                continue
            for a in arcs:
                res = node.measure(d, 0.0, r, a, args.one_sided)
                pr, pe = analytic_prediction(r, d, a, args.one_sided)
                if res is None:
                    rows.append(dict(r_gt=r, d=d, arc=a, n_circles=0,
                                     r_meas='', e_r='', e_c='', f_req='',
                                     r_pred=round(pr, 5), e_c_pred=round(pe, 5),
                                     note='NO_DETECTION'))
                    print(f'r={r} d={d} arc={a}: NO DETECTION')
                    continue
                c, ncirc = res
                # radial (line-of-sight) centre error, + = reported too far
                e_c = math.hypot(c.center.x, c.center.y) - d
                e_r = c.true_radius - r
                # The safety-relevant combination (phase4_containment.py's
                # F_req with the velocity term zero on a static target):
                # inflation the pair would need for the true circle to be
                # contained. F_req = |c_est - c_gt| + r_gt - r_est.
                f_req = (math.hypot(c.center.x - d, c.center.y) + r
                         - max(c.true_radius, 0.20))
                rows.append(dict(r_gt=r, d=d, arc=a, n_circles=ncirc,
                                 r_meas=round(c.true_radius, 5),
                                 e_r=round(e_r, 5), e_c=round(e_c, 5),
                                 f_req=round(f_req, 5),
                                 r_pred=round(pr, 5), e_c_pred=round(pe, 5),
                                 note=''))
                print(f'r={r:.2f} d={d:.1f} arc={a:.2f}: '
                      f'r_meas={c.true_radius:.4f} (pred {pr:.4f}) '
                      f'e_r={e_r*1000:+6.1f}mm e_c={e_c*1000:+7.1f}mm '
                      f'(pred {pe*1000:+7.1f}) F_req={f_req*1000:+6.1f}mm n={ncirc}')
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'\nwrote {args.out} ({len(rows)} rows)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
