#!/usr/bin/python3
"""Scenario scene generator for S1/S2 fixtures (§17.1, Phase 3).

Builds the G1 scene (scene_g1.xml, H-1 mid360_link site) plus four MOCAP
cylinders (r=0.15, h=1.5 — the standard sim obstacle) and writes:

  <out>/scenario_mirror.xml    mirror model for sim_mjlidar_bridge
  <out>/s1_static.json         scenario: 3 surveyed static cylinders
  <out>/s2_cross_05.json       scenario: single crosser at 0.5 m/s
  <out>/s2_cross_08.json       scenario: single crosser at 0.8 m/s

One mirror model serves every scenario: the scenario json tells
scenario_state_source.py where each mocap cylinder is (or how it moves);
unused cylinders are parked at PARK (far outside the 5 m lidar range).

S1 (T4): cylinders with faces surveyed at exactly 1.000 / 2.000 / 3.000 m
from the base-footprint origin at bearings 45° / 135° / −45° (the same
surveyed props as the Phase-2 wall_scene, without the walls).

S2 (T5): one cylinder crossing the +x half-plane on the line x = 2.0 from
y = −4.9 to y = +4.9 at constant speed, entering already moving: the start
point is OUTSIDE range_max (5.0 m), so track birth happens on a moving
object — the T5 reading of "crossing". (A first cut had the crosser parked
in view for 3 s before moving; a velocity step onto a converged static
track is a different, harder problem than T5 tests, and it produced
association losses that vanish with the faithful scenario.) Closest
approach 2.0 m; inside DPCBF's p_max (3.0) for |y| < 2.24; in sensor range
for |y| < 4.58.

The robot pose is GROUNDED (zero joints, lowest AABB corner on z=0) exactly
as in the Phase-2 wall_scene — no dynamics ever run (kinematic mirror).

Usage: make_scenario_scene.py [--out DIR] [--scene PATH]
"""
import argparse
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SCENE = os.path.abspath(os.path.join(
    HERE, '..', '..', '..', 'src', 'assets', 'robots', 'unitree_g1', 'xmls',
    'scene_g1.xml'))

CYL_R = 0.15
CYL_HALF_H = 0.75
PARK = [50.0, 50.0]           # outside range_max by a wide margin

# S1 surveyed props: (name, bearing_deg, face range [m])
S1_TARGETS = [
    ('s1_cyl_1m', 45.0, 1.0),
    ('s1_cyl_2m', 135.0, 2.0),
    ('s1_cyl_3m', -45.0, 3.0),
]
MOCAP_BODIES = [t[0] for t in S1_TARGETS] + ['s2_crosser']

S2_LINE = {'x': 2.0, 'y0': -4.9, 'y1': 4.9}   # endpoints outside range_max
S2_PREROLL_S = 1.0
S2_TAIL_S = 1.0


def grounded_qpos(model):
    """Zero-joint standing pose with the lowest robot AABB corner at z=0."""
    data = mujoco.MjData(model)
    qpos = np.zeros(model.nq)
    qpos[2] = 0.793
    qpos[3] = 1.0
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)

    min_z = np.inf
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0:
            continue
        center = model.geom_aabb[g, :3]
        half = model.geom_aabb[g, 3:]
        xmat = data.geom_xmat[g].reshape(3, 3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corner = center + half * np.array([sx, sy, sz])
                    min_z = min(min_z, (data.geom_xpos[g] + xmat @ corner)[2])
    qpos[2] -= min_z
    return qpos


def s1_positions():
    out = {}
    for name, bearing_deg, face in S1_TARGETS:
        b = math.radians(bearing_deg)
        d = face + CYL_R
        out[name] = [d * math.cos(b), d * math.sin(b)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/scenarios_phase3')
    ap.add_argument('--scene', default=DEFAULT_SCENE)
    args = ap.parse_args()

    spec = mujoco.MjSpec.from_file(args.scene)
    spec.meshdir = os.path.join(os.path.dirname(os.path.abspath(args.scene)),
                                spec.meshdir or 'assets')
    for name in MOCAP_BODIES:
        body = spec.worldbody.add_body(name=name, mocap=True,
                                       pos=[PARK[0], PARK[1], CYL_HALF_H])
        body.add_geom(name=name + '_geom',
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[CYL_R, CYL_HALF_H, 0],
                      rgba=[0.3, 0.3, 0.8, 1])
    model = spec.compile()
    assert model.nmocap == len(MOCAP_BODIES)

    qpos = grounded_qpos(model)

    os.makedirs(args.out, exist_ok=True)
    xml_path = os.path.join(args.out, 'scenario_mirror.xml')
    with open(xml_path, 'w') as f:
        f.write(spec.to_xml())
    check = mujoco.MjModel.from_xml_path(xml_path)
    assert check.nq == model.nq and check.nmocap == len(MOCAP_BODIES)

    common = {
        'nq': int(model.nq),
        'qpos': [float(v) for v in qpos],
        'mocap_bodies': MOCAP_BODIES,     # publish order == mocapid order
        'radius_m': CYL_R,
        'half_height_m': CYL_HALF_H,
        'park': PARK,
    }

    pos = s1_positions()
    s1 = dict(common)
    s1['name'] = 's1_static'
    s1['duration_s'] = 30.0
    s1['obstacles'] = [
        {'body': n, 'mode': 'static', 'pos': pos[n]} for n, _, _ in S1_TARGETS]
    s1['gt_targets'] = [
        {'name': n, 'bearing_deg': b, 'face_range_m': r,
         'center': pos[n], 'radius_m': CYL_R} for n, b, r in S1_TARGETS]

    for speed, tag in ((0.5, 's2_cross_05'), (0.8, 's2_cross_08')):
        s2 = dict(common)
        s2['name'] = tag
        cross_s = (S2_LINE['y1'] - S2_LINE['y0']) / speed
        s2['duration_s'] = S2_PREROLL_S + cross_s + S2_TAIL_S
        s2['obstacles'] = [{
            'body': 's2_crosser', 'mode': 'cross',
            'p0': [S2_LINE['x'], S2_LINE['y0']],
            'p1': [S2_LINE['x'], S2_LINE['y1']],
            'speed': speed, 't_start': S2_PREROLL_S,
        }]
        with open(os.path.join(args.out, tag + '.json'), 'w') as f:
            json.dump(s2, f, indent=2)

    with open(os.path.join(args.out, 's1_static.json'), 'w') as f:
        json.dump(s1, f, indent=2)

    print(f'wrote {xml_path} (+ s1_static / s2_cross_05 / s2_cross_08 json)')
    print(f'nq={model.nq} nmocap={model.nmocap} pelvis_z={qpos[2]:.4f}')


if __name__ == '__main__':
    main()
