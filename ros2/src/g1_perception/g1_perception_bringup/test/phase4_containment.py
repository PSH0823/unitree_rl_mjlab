#!/usr/bin/python3
"""§9.6 containment calibration (the Q-2 measurement).

Input: phase4_obstacles_dump.py JSONL files (gt + tracked streams) from the
S1–S4 fixture replays. For every /tracked_obstacles circle matched to a GT
circle (nearest-neighbor ≤ 0.5 m, GT extrapolated to the tracked stamp by
its commanded velocity), computes the fixed-inflation term that pair would
REQUIRE for the true circle to be contained in the safety circle:

    safe_r(F) = max(tr_est, min_radius) + F + min(|v_est|, v_max)·latency_horizon
    contained ⇔ |c_est − c_gt| + r_gt ≤ safe_r(F)
    F_req      = |c_est − c_gt| + r_gt − max(tr_est, min_radius)
                 − min(|v_est|, v_max)·latency_horizon

The calibrated fixed term is the 99.9th percentile of F_req pooled over
S1–S4 (and must also give ≥99.9 % per scenario). Circles the filter's own
gates drop (tr > max_circle_radius) are excluded, mirroring the filter.

Scope: only pairs whose GT circle is within p_max (3.0 m) of the robot at
(0,0) count — DPCBF pre-selects to p_max (§2.3-2), so containment outside
that horizon has no safety meaning (track birth at range_max and
beyond-range coasting would otherwise dominate the tail with pairs the
filter never consumes). Matching is globally greedy by pair distance.

Usage: phase4_containment.py <name=dump.jsonl> [...] [--horizon 0.1]
"""
import json
import math
import sys

MIN_R = 0.20
MAX_CIRCLE_R = 0.60
V_MAX = 1.5
LAT_HORIZON = 0.12
GATE = 0.999
P_MAX = 3.0


def load(path):
    gt, tracked = [], []
    for line in open(path):
        r = json.loads(line)
        if r['topic'] == '/sim/gt_obstacles':
            gt.append(r)
        elif r['topic'] == '/tracked_obstacles':
            tracked.append(r)
    return gt, tracked


def f_req_list(gt, tracked, horizon=0.0):
    """Required fixed term per matched pair; optionally both sides
    extrapolated `horizon` seconds forward (sensitivity check)."""
    out = []
    misses = 0
    gi = 0
    for frame in tracked:
        t = frame['t']
        while gi + 1 < len(gt) and abs(gt[gi + 1]['t'] - t) <= abs(gt[gi]['t'] - t):
            gi += 1
        g = gt[gi]
        dt = t - g['t']
        gt_circles = [
            (c['x'] + c['vx'] * (dt + horizon),
             c['y'] + c['vy'] * (dt + horizon), c['tr']) for c in g['circles']]
        est = [c for c in frame['circles'] if c['tr'] <= MAX_CIRCLE_R]
        # Globally greedy matching (all pairs sorted by distance): stable in
        # dense scenes where per-circle greedy in message order mismatches.
        pairs = []
        for j, c in enumerate(est):
            ex = c['x'] + c['vx'] * horizon
            ey = c['y'] + c['vy'] * horizon
            for i, (gx, gy, gr) in enumerate(gt_circles):
                d2 = (ex - gx) ** 2 + (ey - gy) ** 2
                if d2 <= 0.25:
                    pairs.append((d2, j, i))
        pairs.sort()
        used_e, used_g = set(), set()
        for d2, j, i in pairs:
            if j in used_e or i in used_g:
                continue
            used_e.add(j)
            used_g.add(i)
            gx, gy, gr = gt_circles[i]
            if math.hypot(gx, gy) > P_MAX:
                continue  # outside the DPCBF selection horizon (§2.3-2)
            c = est[j]
            speed = min(math.hypot(c['vx'], c['vy']), V_MAX)
            ex = c['x'] + c['vx'] * horizon
            ey = c['y'] + c['vy'] * horizon
            dist = math.hypot(ex - gx, ey - gy)
            f = dist + gr - max(c['tr'], MIN_R) - speed * LAT_HORIZON
            out.append(f)
        misses += sum(1 for j in range(len(est)) if j not in used_e
                      and math.hypot(est[j]['x'], est[j]['y']) <= P_MAX)
    return out, misses


def pct_at(f_reqs, f):
    if not f_reqs:
        return float('nan')
    return sum(1 for v in f_reqs if v <= f) / len(f_reqs)


def quantile(sorted_v, q):
    if not sorted_v:
        return float('nan')
    idx = min(len(sorted_v) - 1, int(math.ceil(q * len(sorted_v))) - 1)
    return sorted_v[max(idx, 0)]


def main():
    args = [a for a in sys.argv[1:] if '=' in a]
    horizon = 0.0
    if '--horizon' in sys.argv:
        horizon = float(sys.argv[sys.argv.index('--horizon') + 1])
    pooled = []
    per = {}
    for a in args:
        name, path = a.split('=', 1)
        gt, tracked = load(path)
        f_reqs, unmatched = f_req_list(gt, tracked, horizon)
        per[name] = sorted(f_reqs)
        pooled.extend(f_reqs)
        print(f'{name}: pairs={len(f_reqs)} est_unmatched={unmatched} '
              f'F_req p50={quantile(per[name],0.5)*1000:.1f}mm '
              f'p99={quantile(per[name],0.99)*1000:.1f}mm '
              f'p99.9={quantile(per[name],0.999)*1000:.1f}mm '
              f'max={max(f_reqs)*1000:.1f}mm' if f_reqs else f'{name}: no pairs')
    pooled.sort()
    print(f'POOLED: pairs={len(pooled)} '
          f'p99.9={quantile(pooled,GATE)*1000:.2f}mm max={pooled[-1]*1000:.2f}mm')
    for f_mm in (0, 5, 10, 15, 20, 25, 30, 40, 50):
        f = f_mm / 1000.0
        line = f'F={f_mm:3d}mm pooled={pct_at(pooled,f)*100:7.3f}%'
        for name in per:
            line += f'  {name}={pct_at(per[name],f)*100:7.3f}%'
        print(line)
    f999 = quantile(pooled, GATE)
    worst = max(quantile(v, GATE) for v in per.values() if v)
    pick = max(f999, worst, 0.0)
    print(f'calibrated fixed_inflation (>= {GATE*100}% pooled AND per-scenario): '
          f'{max(pick,0)*1000:.1f} mm  (horizon={horizon}s)')


if __name__ == '__main__':
    main()
