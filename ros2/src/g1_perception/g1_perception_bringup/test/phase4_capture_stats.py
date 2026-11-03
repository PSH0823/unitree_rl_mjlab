#!/usr/bin/python3
"""Analyze a Filter-I/O capture (filter_io_log.h format): live closed-loop
stats (§17.3) and the T6 staleness timeline at query resolution.

Usage:
  phase4_capture_stats.py stats <capture.bin>
  phase4_capture_stats.py t6 <capture.bin>
"""
import math
import struct
import sys

HDR = struct.Struct('<8sdII')
PRE = struct.Struct('<d fff 5d 3d d d I I')
OB = struct.Struct('<5d i')
SUF = struct.Struct('<3d 3d iii B 3f')


def read_records(path):
    f = open(path, 'rb')
    hdr = f.read(HDR.size)
    magic, timestep, mode, _ = HDR.unpack(hdr)
    assert magic.startswith(b'T1CAP01'), magic
    while True:
        b = f.read(PRE.size)
        if len(b) < PRE.size:
            return
        v = PRE.unpack(b)
        (t, lx, ly, rx, r0, r1, r2, r3, r4, d0, d1, d2, scale, age, state,
         n) = v
        ob = f.read(OB.size * n)
        if len(ob) < OB.size * n:
            return
        sb = f.read(SUF.size)
        if len(sb) < SUF.size:
            return
        s = SUF.unpack(sb)
        yield {
            't': t, 'axes': (lx, ly, rx), 'robot': (r0, r1, r2, r3, r4),
            'desired': (d0, d1, d2), 'scale': scale, 'age': age,
            'state': state, 'n_obs': n,
            'out': (s[0], s[1], s[2]), 'active': s[6], 'solved': s[9],
        }


def stats(path):
    n = 0
    mse = 0.0
    scale_lt1 = stop = 0
    act = 0
    act_max = 0
    unsolved = 0
    n_obs_sum = 0
    t0 = t1 = None
    for r in read_records(path):
        n += 1
        t0 = r['t'] if t0 is None else t0
        t1 = r['t']
        # effective desired = scale * desired (what the seam fed Filter)
        d = tuple(x * r['scale'] for x in r['desired'])
        mse += sum((a - b) ** 2 for a, b in zip(r['desired'], r['out']))
        if r['scale'] < 1.0:
            scale_lt1 += 1
        if r['state'] >= 2:
            stop += 1
        act += r['active']
        act_max = max(act_max, r['active'])
        unsolved += 0 if r['solved'] else 1
        n_obs_sum += r['n_obs']
    print(f'records={n} span={t1-t0:.1f}s sim')
    print(f'cmd-tracking MSE (desired-unscaled vs out) = {mse/n:.6f}')
    print(f'degrade(scale<1) {100*scale_lt1/n:.3f}%  stop-state {100*stop/n:.3f}%')
    print(f'active constraints mean {act/n:.2f} max {act_max}; unsolved {unsolved}')
    print(f'obstacles per query mean {n_obs_sum/n:.1f}')


def t6(path):
    prev = None
    events = []
    stop_t = None
    max_infl_age = 0.0
    for r in read_records(path):
        st = r['state']
        if prev is None or st != prev['state']:
            events.append((r['t'], st, r['age'], r['scale'], r['n_obs']))
        if st == 2:
            max_infl_age = max(max_infl_age, r['age'])
        prev = r
    names = {0: 'FRESH', 1: 'DEGRADE', 2: 'STOP', 3: 'NO_DATA'}
    print('state transitions (t_sim, state, age_s, scale, n_obs):')
    for t, st, age, scale, nob in events:
        print(f'  t={t:9.3f}  {names.get(st,st):8s} age={age:6.3f} '
              f'scale={scale:5.3f} n_obs={nob}')
    # gate checks
    for i in range(1, len(events)):
        t, st, age, scale, nob = events[i]
        if st == 1 and events[i-1][1] == 0:
            print(f'degrade onset: age={age:.4f}s (gate: within one query of '
                  f'0.300 → {"PASS" if age <= 0.302 else "FAIL"})')
        if st == 2 and events[i-1][1] == 1:
            print(f'stop onset: age={age:.4f}s (gate: 0.600 → '
                  f'{"PASS" if age <= 0.602 else "FAIL"})')
        if st == 0 and events[i-1][1] in (2, 3):
            print(f'recovery to FRESH at t={t:.3f} (prev state age '
                  f'{events[i-1][2]:.3f}s)')
    print(f'max age seen in STOP: {max_infl_age:.3f}s '
          f'(retained-set inflation horizon caps at 1.0)')


if __name__ == '__main__':
    {'stats': stats, 't6': t6}[sys.argv[1]](sys.argv[2])
