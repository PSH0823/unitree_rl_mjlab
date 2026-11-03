#!/usr/bin/env python3
"""Collate walk_ab_probe metrics into the §17.3 walking A/B table.

  walk_ab_report.py <outdir>

The §17.3 rule for the safety metrics is asymmetric and worth restating:
oracle mode MUST be zero collisions (it sees exact ground truth, so any
collision there is a DPCBF/controller fault, not a perception one), and
estimated mode must MATCH it. A non-zero oracle number invalidates the
estimated comparison rather than excusing it.
"""
import glob
import json
import os
import sys


def main():
    outdir = sys.argv[1]
    runs = {}
    for p in sorted(glob.glob(os.path.join(outdir, '*_metrics.json'))):
        tag = os.path.basename(p)[:-len('_metrics.json')]
        runs[tag] = json.load(open(p))

    scen = sorted({t.rsplit('_', 1)[0] for t in runs})
    print('=== §17.3 walking A/B — collision rate and min clearance ===\n')
    hdr = (f'{"scenario":10s} {"mode":10s} {"walk s":>7s} {"fell@":>7s} '
           f'{"path m":>7s} {"in-scope":>8s} {"GTinPmax":>8s} {"viol ev":>7s} '
           f'{"viol t%":>8s} {"min cl":>8s} {"p01 cl":>8s} {"p50 cl":>8s}')
    print(hdr)
    print('-' * len(hdr))
    for s in scen:
        for m in ('oracle', 'estimated', 'rig'):
            r = runs.get(f'{s}_{m}')
            if not r:
                continue
            if r.get('outcome'):
                print(f'{s:10s} {m:10s}   {r["outcome"]} '
                      f'(fell at {r["fell_at_s"]:.1f} s)')
                continue
            fell = f'{r["fell_at_s"]:7.1f}' if r['fell_at_s'] else '      -'
            print(f'{s:10s} {m:10s} {r["window_s"]:7.1f} {fell} '
                  f'{r["path_m"]:7.2f} {100*r["in_scope_frac"]:7.1f}% '
                  f'{r["gt_in_scope_mean"]:8.2f} '
                  f'{r["margin_violation_events"]:7d} '
                  f'{100*r["margin_violation_time_frac"]:8.3f} '
                  f'{r["clearance_min_m"]:8.3f} {r["clearance_p01_m"]:8.3f} '
                  f'{r["clearance_p50_m"]:8.3f}')
    print('\n=== tracked -> nearest-GT (shadow delta, walking base) ===\n')
    hdr2 = (f'{"scenario":10s} {"mode":10s} {"n":>7s} {"p50 mm":>8s} '
            f'{"p90 mm":>8s} {"p99 mm":>8s}')
    print(hdr2)
    print('-' * len(hdr2))
    for s in scen:
        for m in ('oracle', 'estimated', 'rig'):
            r = runs.get(f'{s}_{m}')
            if not r or not r.get('tracked_to_gt_n'):
                continue
            print(f'{s:10s} {m:10s} {r["tracked_to_gt_n"]:7d} '
                  f'{r["tracked_to_gt_p50_mm"]:8.1f} '
                  f'{r["tracked_to_gt_p90_mm"]:8.1f} '
                  f'{r["tracked_to_gt_p99_mm"]:8.1f}')

    print('\nrig = the Phase-4 suspension control (band never lowered, no\n'
          'policy): the same field and the same code as the walking run, so\n'
          'the difference between its deltas and the oracle row is the RIG,\n'
          'not four phases of code drift.')
    print('\n=== verdict ===')
    for s in scen:
        o, e = runs.get(f'{s}_oracle'), runs.get(f'{s}_estimated')
        if not (o and e):
            continue
        if o.get('outcome') or e.get('outcome'):
            print(f'  NO COMPARISON — {s}: '
                  f'oracle {o.get("outcome", "ok")}, '
                  f'estimated {e.get("outcome", "ok")}')
            continue
        line = (f'{s}: oracle {o["margin_violation_events"]} margin violations '
                f'(min cl {o["clearance_min_m"]:.3f} m, fell '
                f'{"at %.1f s" % o["fell_at_s"] if o["fell_at_s"] else "no"}); '
                f'estimated {e["margin_violation_events"]} '
                f'(min cl {e["clearance_min_m"]:.3f} m, fell '
                f'{"at %.1f s" % e["fell_at_s"] if e["fell_at_s"] else "no"})')
        if o['margin_violation_events'] != 0:
            print('  ORACLE NOT CLEAN — ' + line)
            print('    The estimated arm cannot be scored as a perception '
                  'result against a baseline that itself violates: whatever '
                  'the oracle arm does here is a DPCBF/controller property.')
        elif e['margin_violation_events'] == 0:
            print('  PASS — ' + line)
        else:
            print('  FAIL — ' + line)


if __name__ == '__main__':
    main()
