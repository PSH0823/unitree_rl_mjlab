#!/usr/bin/python3
"""§17.3 metrics from ab_eval CSVs (offline paired-filter A/B) or from a
pair of live Filter-I/O capture files (--capture mode).

CSV mode:   phase4_ab_metrics.py name=out.csv [...]
Metrics per scenario: command-tracking MSE per arm (filtered vs desired,
3-component), tracking ratio (>=0.95 gate: (1-MSE_est/ref)/(1-MSE_orc/ref)
is ill-defined when desired≈output; we report MSE ratio AND the §17.3
"command tracking performance" as 1/(1+MSE)), degrade/stop time fractions,
active-constraint stats, solver failure counts.
"""
import csv
import math
import sys


def analyze(name, path):
    n = 0
    mse_o = mse_e = 0.0
    scale_lt1 = stop = 0
    act_o = act_e = 0
    act_o_max = act_e_max = 0
    unsolved_o = unsolved_e = 0
    diff = 0.0  # oracle-vs-estimated command discrepancy
    for row in csv.DictReader(open(path)):
        n += 1
        d = (float(row['des_sag']), float(row['des_lat']), float(row['des_yaw']))
        o = (float(row['o_sag']), float(row['o_lat']), float(row['o_yaw']))
        e = (float(row['e_sag']), float(row['e_lat']), float(row['e_yaw']))
        mse_o += sum((a - b) ** 2 for a, b in zip(d, o))
        mse_e += sum((a - b) ** 2 for a, b in zip(d, e))
        diff += sum((a - b) ** 2 for a, b in zip(o, e))
        sc = float(row['e_scale'])
        st = int(row['e_state'])
        if sc < 1.0:
            scale_lt1 += 1
        if st >= 2:
            stop += 1
        act_o += int(row['o_active'])
        act_e += int(row['e_active'])
        act_o_max = max(act_o_max, int(row['o_active']))
        act_e_max = max(act_e_max, int(row['e_active']))
        unsolved_o += 1 - int(row['o_solved'])
        unsolved_e += 1 - int(row['e_solved'])
    mse_o /= n
    mse_e /= n
    diff /= n
    perf_o = 1.0 / (1.0 + mse_o)
    perf_e = 1.0 / (1.0 + mse_e)
    print(f'{name}: ticks={n}')
    print(f'  cmd-tracking MSE  oracle={mse_o:.6f}  estimated={mse_e:.6f}  '
          f'perf ratio est/orc={perf_e/perf_o:.4f} (gate >=0.95)')
    print(f'  oracle-vs-estimated cmd RMS diff={math.sqrt(diff):.4f} m/s')
    print(f'  degrade(scale<1) {100*scale_lt1/n:.2f}%  stop {100*stop/n:.2f}%')
    print(f'  active constraints mean orc={act_o/n:.2f} est={act_e/n:.2f} '
          f'max orc={act_o_max} est={act_e_max}')
    print(f'  unsolved ticks orc={unsolved_o} est={unsolved_e}')
    return perf_e / perf_o


def main():
    worst = 1.0
    for a in sys.argv[1:]:
        name, path = a.split('=', 1)
        worst = min(worst, analyze(name, path))
    print(f'WORST perf ratio: {worst:.4f} (gate >= 0.95)')


if __name__ == '__main__':
    main()
