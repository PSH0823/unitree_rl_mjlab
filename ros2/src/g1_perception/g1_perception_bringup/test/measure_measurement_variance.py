#!/usr/bin/env python3
"""Derive the tracker's `measurement_variance` from data, in m^2 (gap G1).

The tracker's Kalman filter is textbook: C = [1 0], y = the extractor's circle
centre in METRES, so R(0,0) has units of m^2, P(0,0) has units of m^2, and
sqrt(P(0,0)) — the sigma `safety_obstacle_filter` inflates by — is metres.
The maths was never unitless. What was wrong is the VALUE: Appendix A's
`measurement_variance: 1.0` asserts a 1-metre 1-sigma position measurement,
which is why sigma came out at 583 mm and why k_sigma silently absorbed the
scale. Nothing downstream can notice that, because a variance of 1.0 m^2 is a
perfectly well-typed number.

R is the variance of the MEASUREMENT NOISE — the random scatter of the
extractor's centre estimate — and explicitly NOT the systematic circle-fit
bias (e_c = -0.278*r, e_r = +0.084*r), which is deterministic, is what
`fixed_inflation` already covers, and would be double-counted if folded in
here. So this script measures scatter about a per-target MEAN on a static
scene, where the mean absorbs the bias by construction.

    measure_measurement_variance.py <jsonl from phase4_obstacles_dump.py>
                                    [--topic /raw_obstacles] [--assoc 0.30]

Prints the per-axis scatter and the recommended `measurement_variance`. On
hardware this is 5B block 1: the same script against a surveyed static bag.
"""
import json
import math
import sys
from collections import defaultdict


def main():
    path = sys.argv[1]
    topic = sys.argv[sys.argv.index('--topic') + 1] if '--topic' in sys.argv \
        else '/raw_obstacles'
    assoc = float(sys.argv[sys.argv.index('--assoc') + 1]) if '--assoc' in sys.argv \
        else 0.30

    frames = []
    for line in open(path):
        r = json.loads(line)
        if r['topic'] == topic:
            frames.append(r)
    if not frames:
        print(f'no {topic} frames in {path}')
        return 2

    # Cluster detections across frames by proximity: on a static scene a
    # cluster is one physical target, and its spread IS the measurement noise.
    clusters = []   # list of [sum_x, sum_y, sum_r, n, [(x,y,r)...]]
    for f in frames:
        for c in f['circles']:
            # phase4_obstacles_dump.py's schema: x/y/r/tr/vx/vy/uid/cov.
            x, y, r = c['x'], c['y'], c['tr']
            best, bestd = None, assoc
            for cl in clusters:
                d = math.hypot(cl[0] / cl[3] - x, cl[1] / cl[3] - y)
                if d < bestd:
                    best, bestd = cl, d
            if best is None:
                clusters.append([x, y, r, 1, [(x, y, r)]])
            else:
                best[0] += x; best[1] += y; best[2] += r; best[3] += 1
                best[4].append((x, y, r))

    clusters = [c for c in clusters if c[3] >= 20]
    if not clusters:
        print('no cluster with >=20 detections; need a static scene')
        return 2

    vx = vy = vr = 0.0
    n = 0
    print(f'{"target":>7} {"n":>5} {"mean x":>9} {"mean y":>9} '
          f'{"sd x mm":>8} {"sd y mm":>8} {"sd r mm":>8}')
    for i, cl in enumerate(clusters):
        mx, my, mr, k = cl[0] / cl[3], cl[1] / cl[3], cl[2] / cl[3], cl[3]
        sx = sum((x - mx) ** 2 for x, _, _ in cl[4]) / (k - 1)
        sy = sum((y - my) ** 2 for _, y, _ in cl[4]) / (k - 1)
        sr = sum((r - mr) ** 2 for _, _, r in cl[4]) / (k - 1)
        print(f'{i:>7} {k:>5} {mx:9.3f} {my:9.3f} {1000*math.sqrt(sx):8.1f} '
              f'{1000*math.sqrt(sy):8.1f} {1000*math.sqrt(sr):8.1f}')
        vx += sx * (k - 1); vy += sy * (k - 1); vr += sr * (k - 1)
        n += k - 1

    pooled = (vx + vy) / (2 * n)
    print(f'\npooled position variance  {pooled:.3e} m^2  '
          f'(1-sigma {1000*math.sqrt(pooled):.1f} mm)')
    print(f'pooled radius   variance  {vr/n:.3e} m^2  '
          f'(1-sigma {1000*math.sqrt(vr/n):.1f} mm)')
    print(f'\nrecommended obstacle_tracker measurement_variance: '
          f'{pooled:.2e}    # m^2')
    print('Shipped value 1.0 asserts a 1 m 1-sigma measurement, i.e. it is '
          f'{pooled and 1.0/pooled:.0f}x too large.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
