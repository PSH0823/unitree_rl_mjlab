#!/usr/bin/python3
"""k_sigma calibration harness (H-8 / §9.6 sigma term; P-3 remediation of Q-2).

Phase 4 calibrated `fixed_inflation` = 0.051 m as the 99.9th percentile of the
per-pair required inflation F_req, and recorded plainly what a fixed term cannot
buy: the occlusion-coast and merged-arc transients (S3 15% of pairs, S4 crosser
coast up to ~0.38 m). Those are exactly the samples where the tracker's own
posterior variance is large — which is the premise of the sigma term. This
script tests that premise and, if it holds, derives k_sigma.

Input is the SAME capture the containment calibration uses
(`phase4_obstacles_dump.py` JSONL), which now records `cov` per circle. So a 5B
robot session is a data drop, not a development task:

    phase4_obstacles_dump.py 120 hw_s1.jsonl        # on the robot
    calibrate_k_sigma.py s1=hw_s1.jsonl --fixed 0.051

Definitions (identical to phase4_containment.py, so the two are comparable):

    F_req  = |c_est - c_gt| + r_gt - max(tr_est, min_radius)
             - min(|v_est|, v_max) * latency_horizon           [m]
    sigma  = sqrt(max(var_x, var_y) + var_r)                   [m]
             (safety_obstacle_filter::SurfaceSigma, gating.h)

Containment with the sigma term holds for a pair iff  fixed + k*sigma >= F_req,
so the smallest k covering a quantile q of pairs is

    k(q, fixed) = quantile_q( (F_req - fixed) / sigma ),  over pairs with sigma>0

Outputs: the sigma distribution, the correlation between sigma and F_req (the
premise — a term uncorrelated with the residual buys margin, not safety), the
coverage of fixed-only vs fixed+k*sigma, and a (fixed, k) grid with the mean
inflation each costs. NOTHING here is authoritative on sim data: sim covariances
come from sim measurement noise, so a k derived here is a rehearsal, not a
calibration. Say which when quoting a number.

UNITS (gap G1, corrected). The KF is textbook — C = [1 0] and y is a circle
centre in metres — so R(0,0), P(0,0) and this sigma have always been m^2/m.
The earlier reading that "sigma is not in metres" was half right: the
dimensions were never wrong, the VALUE was. `measurement_variance: 1.0` seeds
P(0,0) directly and asserts a 1-metre 1-sigma LiDAR measurement, which is why
sigma read 583 mm and why k_sigma silently absorbed the scale. A variance of
1.0 m^2 is a well-typed number, so nothing downstream could notice. This
script therefore REFUSES to emit a k_sigma when the sigma distribution is
implausible for a LiDAR-derived centre estimate (SIGMA_PLAUSIBLE_MAX): derive
`measurement_variance` from data first with measure_measurement_variance.py.
"""
import json
import math
import sys

MIN_R = 0.20
MAX_CIRCLE_R = 0.60
V_MAX = 1.5
LAT_HORIZON = 0.12
P_MAX = 3.0
GATE = 0.999
# Largest 1-sigma obstacle-centre uncertainty that can plausibly come out of a
# Mid-360 + circle fit. The measured systematic fit error at r=0.30 is ~103 mm
# and the RANDOM part is far smaller, so anything past a quarter of a metre
# means the tracker's R was never set from data (gap G1).
SIGMA_PLAUSIBLE_MAX = 0.25   # m


def load(path):
    gt, tracked = [], []
    for line in open(path):
        r = json.loads(line)
        if r['topic'] == '/sim/gt_obstacles':
            gt.append(r)
        elif r['topic'] == '/tracked_obstacles':
            tracked.append(r)
    return gt, tracked


def sigma_of(circle):
    cov = circle.get('cov') or []
    if len(cov) < 3:
        return 0.0
    var = max(cov[0], cov[1]) + cov[2]
    return math.sqrt(var) if var > 0.0 else 0.0


def pairs_with_sigma(gt, tracked):
    """(F_req, sigma) per matched pair — matching identical to
    phase4_containment.f_req_list (globally greedy, p_max-scoped)."""
    out = []
    gi = 0
    for frame in tracked:
        t = frame['t']
        while gi + 1 < len(gt) and abs(gt[gi + 1]['t'] - t) <= abs(gt[gi]['t'] - t):
            gi += 1
        g = gt[gi]
        dt = t - g['t']
        gt_circles = [(c['x'] + c['vx'] * dt, c['y'] + c['vy'] * dt, c['tr'])
                      for c in g['circles']]
        est = [c for c in frame['circles'] if c['tr'] <= MAX_CIRCLE_R]
        cand = []
        for j, c in enumerate(est):
            for i, (gx, gy, _) in enumerate(gt_circles):
                d2 = (c['x'] - gx) ** 2 + (c['y'] - gy) ** 2
                if d2 <= 0.25:
                    cand.append((d2, j, i))
        cand.sort()
        used_e, used_g = set(), set()
        for _, j, i in cand:
            if j in used_e or i in used_g:
                continue
            used_e.add(j)
            used_g.add(i)
            gx, gy, gr = gt_circles[i]
            if math.hypot(gx, gy) > P_MAX:
                continue
            c = est[j]
            speed = min(math.hypot(c['vx'], c['vy']), V_MAX)
            f = (math.hypot(c['x'] - gx, c['y'] - gy) + gr
                 - max(c['tr'], MIN_R) - speed * LAT_HORIZON)
            out.append((f, sigma_of(c)))
    return out


def quantile(sorted_v, q):
    if not sorted_v:
        return float('nan')
    idx = min(len(sorted_v) - 1, int(math.ceil(q * len(sorted_v))) - 1)
    return sorted_v[max(idx, 0)]


def pearson(xs, ys):
    n = len(xs)
    if n < 2:
        return float('nan')
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return float('nan')
    return sxy / math.sqrt(sxx * syy)


def k_for(pairs, fixed, q):
    """Smallest k covering quantile q of pairs, given the fixed term."""
    need = sorted((f - fixed) / s for f, s in pairs if s > 0.0)
    if not need:
        return float('nan')
    return max(0.0, quantile(need, q))


def coverage(pairs, fixed, k):
    if not pairs:
        return float('nan')
    ok = sum(1 for f, s in pairs if fixed + k * s >= f)
    return ok / len(pairs)


def main():
    args = [a for a in sys.argv[1:] if '=' in a and not a.startswith('--')]
    fixed = 0.051
    if '--fixed' in sys.argv:
        fixed = float(sys.argv[sys.argv.index('--fixed') + 1])
    q = GATE
    if '--quantile' in sys.argv:
        q = float(sys.argv[sys.argv.index('--quantile') + 1])

    pooled, per = [], {}
    for a in args:
        name, path = a.split('=', 1)
        gt, tracked = load(path)
        p = pairs_with_sigma(gt, tracked)
        per[name] = p
        pooled.extend(p)
        sig = sorted(s for _, s in p)
        nz = sum(1 for s in sig if s > 0)
        print(f'{name}: pairs={len(p)} sigma>0={nz} '
              f'sigma p50={quantile(sig,0.5)*1000:.2f}mm '
              f'p99={quantile(sig,0.99)*1000:.2f}mm '
              f'max={(sig[-1]*1000 if sig else float("nan")):.2f}mm')

    if not pooled:
        print('no pairs — is the dump from a run with the tracker publishing?')
        return
    nz = [(f, s) for f, s in pooled if s > 0.0]
    print(f'\nPOOLED pairs={len(pooled)}  with sigma>0: {len(nz)}')
    if not nz:
        print('ALL covariances are zero: the capture predates P-3, or the '
              'obstacle source is the oracle. k_sigma is underivable from it.')
        return

    # --- G1 tripwire: is sigma a plausible position uncertainty at all? ------
    # sqrt(P(0,0)) is metres by construction (C = [1 0], y is a centre
    # coordinate in metres), so a sigma of half a metre is not a units bug —
    # it is a tracker that has been TOLD, via measurement_variance, that its
    # LiDAR measures obstacle centres to +/-1 m. k_sigma then silently absorbs
    # the scale and comes out looking authoritative. Refuse to emit one.
    sig_all = sorted(s for _, s in nz)
    sig_p50 = quantile(sig_all, 0.5)
    print(f'\nsigma p50 = {sig_p50*1000:.1f} mm  (sqrt of the KF position '
          f'variance; metres by construction)')
    if sig_p50 > SIGMA_PLAUSIBLE_MAX:
        print(f'*** REFUSING TO CALIBRATE k_sigma ***')
        print(f'    sigma p50 {sig_p50*1000:.0f} mm exceeds the plausibility '
              f'bound {SIGMA_PLAUSIBLE_MAX*1000:.0f} mm for a LiDAR-derived')
        print(f'    obstacle-centre estimate. That is `measurement_variance` '
              f'(m^2) not being set from data:')
        print(f'    it seeds P(0,0) directly, so R = 1.0 means a 1 m 1-sigma '
              f'measurement. Derive it first with')
        print(f'    measure_measurement_variance.py on a static capture, then '
              f'come back. Any k_sigma from')
        print(f'    this data would be a scale factor for the wrong quantity '
              f'and would LOOK calibrated.')
        return

    r = pearson([f for f, _ in nz], [s for _, s in nz])
    print(f'corr(F_req, sigma) = {r:+.3f}   '
          f'(the premise: a sigma uncorrelated with the residual buys margin, '
          f'not safety)')

    f_sorted = sorted(f for f, _ in pooled)
    print(f'F_req p50={quantile(f_sorted,0.5)*1000:.1f}mm '
          f'p99={quantile(f_sorted,0.99)*1000:.1f}mm '
          f'p{q*100:g}={quantile(f_sorted,q)*1000:.1f}mm '
          f'max={f_sorted[-1]*1000:.1f}mm')

    k = k_for(pooled, fixed, q)
    print(f'\nGiven fixed={fixed*1000:.1f}mm, k_sigma covering {q*100:g}% '
          f'of pairs = {k:.3f}')
    print(f'  coverage fixed-only          : {coverage(pooled, fixed, 0.0)*100:7.3f}%')
    print(f'  coverage fixed + {k:.3f}*sigma : {coverage(pooled, fixed, k)*100:7.3f}%')
    print(f'  coverage fixed + 2.748*sigma : {coverage(pooled, fixed, 2.748)*100:7.3f}%'
          '   (old branch H-8 placeholder)')

    print('\n(fixed mm, k) grid — coverage%% / mean inflation mm')
    ks = [0.0, 1.0, 2.0, 2.748, 4.0, 6.0]
    print('  fixed  ' + ''.join(f'{("k=%.3g" % kk):>18}' for kk in ks))
    for f_mm in (0, 10, 20, 30, 40, 51):
        fx = f_mm / 1000.0
        row = f'  {f_mm:5d}  '
        for kk in ks:
            cov = coverage(pooled, fx, kk) * 100
            mean_infl = (fx + kk * sum(s for _, s in pooled) / len(pooled)) * 1000
            row += f'{cov:9.3f}/{mean_infl:6.1f}  '
        print(row)

    print('\nper-scenario coverage at the pooled pick:')
    for name, p in per.items():
        if p:
            print(f'  {name}: fixed-only={coverage(p, fixed, 0.0)*100:7.3f}%  '
                  f'fixed+{k:.3f}s={coverage(p, fixed, k)*100:7.3f}%')


if __name__ == '__main__':
    main()
