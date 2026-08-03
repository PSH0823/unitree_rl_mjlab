#!/usr/bin/env python3
"""Obstacle fields for the walking A/B (§17.1 S1-S4, live-sim analogues).

The fixture scenarios S1-S4 are *bag replays* against a robot pinned in a
grounded standing pose: the simulator is not in the loop, so they cannot
produce a collision rate. The walking A/B needs the same four densities as
LIVE fields, which on this stack means the DPCBF DynamicObstacleManager's
seeded arena — the only obstacle source the simulator physically simulates
(`collision_enabled: true`, so a contact is a real contact).

These are therefore NOT the S1-S4 bags and must not be reported as them.
They are matched on the property that matters for the perception limit under
test — obstacle count, radius range and speed range — and W4 is the exact
Phase-4 live-arena field (90 obstacles, seed 42), so the walking numbers are
directly comparable with the suspended-rig numbers recorded there.

  walk_scenarios.py <name> <in_yaml> <out_yaml>
"""
import sys

# Only the dynamic_obstacles keys differ; everything else (r_rob, p_max, QP
# parameters) is the frozen dpcbf_config.yaml, copied verbatim.
SCENARIOS = {
    # S1 analogue: sparse and static — the class every Phase-4 gate passed.
    'W1': dict(count=6, speed_range=[0.0, 0.0], radius_range=[0.20, 0.30],
               arena=[8.0, 8.0], seed=20260802,
               note='S1-like: sparse static field'),
    # S2 analogue: same sparsity, obstacles crossing at S2 speeds.
    'W2': dict(count=6, speed_range=[0.5, 0.8], radius_range=[0.20, 0.30],
               arena=[8.0, 8.0], seed=20260802,
               note='S2-like: sparse crossing field'),
    # S3 analogue: the 20-obstacle swarm whose containment/A-B failed.
    'W3': dict(count=20, speed_range=[0.2, 0.8], radius_range=[0.20, 0.30],
               arena=[8.0, 8.0], seed=20260801,
               note='S3-like: 20-obstacle swarm'),
    # The Phase-4 live arena, verbatim: the rig-vs-density control.
    'W4': dict(count=90, speed_range=[0.0, 0.8], radius_range=[0.20, 0.30],
               arena=[20.0, 20.0], seed=42,
               note='Phase-4 live arena (90 obstacles, seed 42)'),
}


def main():
    name, src, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    s = SCENARIOS[name]
    lines = open(src).read().splitlines()
    out, in_arena = [], False
    for ln in lines:
        stripped = ln.strip()
        if stripped.startswith('arena:'):
            in_arena = True
        elif ln and not ln[0].isspace() and not stripped.startswith('#'):
            in_arena = False
        if stripped.startswith('count:'):
            ln = f'  count: {s["count"]}'
        elif stripped.startswith('radius_range:'):
            ln = f'  radius_range: {s["radius_range"]}'
        elif stripped.startswith('speed_range:'):
            ln = f'  speed_range: {s["speed_range"]}'
        elif stripped.startswith('random_seed:'):
            ln = f'  random_seed: {s["seed"]}'
        elif in_arena and stripped.startswith('size:'):
            ln = f'    size: {s["arena"]}'
        elif stripped.startswith('enabled:') and not out:
            pass
        # The visualization window has no place in a headless batch run.
        elif stripped.startswith('window_name:'):
            pass
        out.append(ln)
    # Disable the OpenCV visualiser for batch runs (it is the only thing in
    # this config that opens a window per run).
    text = '\n'.join(out)
    head, sep, tail = text.partition('visualization:')
    if sep:
        tail = tail.replace('enabled: true', 'enabled: false', 1)
        text = head + sep + tail
    open(dst, 'w').write(text + '\n')
    print(f'{name}: {s["note"]}')
    print(f'  count={s["count"]} radius={s["radius_range"]} '
          f'speed={s["speed_range"]} arena={s["arena"]} seed={s["seed"]}')


if __name__ == '__main__':
    main()
