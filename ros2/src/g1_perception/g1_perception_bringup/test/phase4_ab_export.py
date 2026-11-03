#!/usr/bin/python3
"""Export the ab_eval input streams for one scenario (§17.3 offline paired-
filter A/B):

  oracle stream    — exact closed-form GT at every 1 kHz query tick, from the
                     scenario json (same trajectory math as
                     scenario_state_source.py, imported — no duplication);
  estimated stream — /obstacles_safe frames from a phase4_obstacles_dump
                     JSONL of the fixture replay.

Binary layout matches ab_eval.cc / FilterIoObstacle:
  record: double key; uint32 n; n × {double x,y,r,vx,vy; int32 id}

Usage: phase4_ab_export.py <scenario.json> <dump.jsonl> <out_prefix>
Writes <out_prefix>_oracle.bin and <out_prefix>_estimated.bin.
"""
import json
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario_state_source import obstacle_state  # noqa: E402

TICK = 0.001  # ab_eval queries at the 1 kHz seam rate


def write_stream(path, records):
    with open(path, 'wb') as f:
        for key, circles in records:
            f.write(struct.pack('<dI', key, len(circles)))
            for (x, y, r, vx, vy, oid) in circles:
                f.write(struct.pack('<dddddi', x, y, r, vx, vy, oid))


def main():
    scen_path, dump_path, prefix = sys.argv[1:4]
    scen = json.load(open(scen_path))

    oracle = []
    n_ticks = int(round(scen['duration_s'] / TICK))
    for k in range(n_ticks):
        t = k * TICK
        circles = []
        for i, body in enumerate(scen['mocap_bodies']):
            ob = next((o for o in scen['obstacles'] if o['body'] == body),
                      None)
            if ob is None:
                continue
            xy, vel = obstacle_state(ob, t, scen['park'])
            r = ob.get('radius', scen['radius_m'])
            circles.append((xy[0], xy[1], r, vel[0], vel[1], i))
        oracle.append((t, circles))
    write_stream(prefix + '_oracle.bin', oracle)

    estimated = []
    for line in open(dump_path):
        rec = json.loads(line)
        if rec['topic'] != '/obstacles_safe':
            continue
        circles = [(c['x'], c['y'], c['r'], c['vx'], c['vy'],
                    int(c['uid']) & 0x7fffffff) for c in rec['circles']]
        estimated.append((rec['t'], circles))
    estimated.sort(key=lambda r: r[0])
    write_stream(prefix + '_estimated.bin', estimated)
    print(f'{prefix}: oracle {len(oracle)} ticks, '
          f'estimated {len(estimated)} frames')


if __name__ == '__main__':
    main()
