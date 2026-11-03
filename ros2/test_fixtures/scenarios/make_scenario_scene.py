#!/usr/bin/python3
"""Scenario scene generator for S1/S2 fixtures (§17.1, Phase 3) and the
Phase-4 S3/S4 fixtures.

Builds the G1 scene (scene_g1.xml, H-1 mid360_link site) plus four MOCAP
cylinders (r=0.15, h=1.5 — the standard sim obstacle) and writes:

  <out>/scenario_mirror.xml    mirror model for sim_mjlidar_bridge
  <out>/s1_static.json         scenario: 3 surveyed static cylinders
  <out>/s2_cross_05.json       scenario: single crosser at 0.5 m/s
  <out>/s2_cross_08.json       scenario: single crosser at 0.8 m/s

Phase 4 adds (separate mirror — the S1/S2 bags recorded /sim/mj_state with
nmocap=4 and the mirror validates lengths strictly, so the old mirror must
stay byte-stable for old bags):

  <out>/scenario_mirror_p4.xml mirror with 4 legacy + 20 swarm + 1 blocker
                               mocap bodies (nmocap=25)
  <out>/s3_swarm.json          S3: 20-obstacle seeded swarm, box-reflect
                               motion (the DynamicObstacleManager model),
                               radii seeded in [0.15, 0.28] m so the
                               min_radius=0.20 clamp regime is exercised
                               on BOTH sides (§9.6 calibration)
  <out>/s4_occlusion.json      S4: static blocker r=0.30 at (2.0, 0) +
                               crosser on x=3.2 at 0.6 m/s emerging from
                               the blocker's shadow (occlusion ~2 s >
                               tracking_duration 1.0 s: track dies and
                               re-acquires — the extrapolation+staleness
                               worst case the policy exists for)

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
import random

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

# ---- Phase 4 (S3/S4) ----
S3_SEED = 20260801
S3_COUNT = 20
S3_RADIUS_RANGE = (0.15, 0.28)     # straddles the min_radius=0.20 clamp
S3_SPEED_RANGE = (0.2, 0.8)        # all moving; arena max is 0.8 (§2.4)
S3_BOX = {'x': [0.8, 7.0], 'y': [-4.0, 4.0]}   # robot at origin is outside
S3_DURATION_S = 30.0
S3_BODIES = ['s3_%02d' % i for i in range(S3_COUNT)]

S4_BLOCKER = {'body': 's4_blocker', 'radius': 0.30, 'pos': [2.0, 0.0]}
# Crosser INSIDE p_max (2.6 < 3.0 — containment during occlusion coasting is
# safety-relevant only if DPCBF would consume the track, §2.3-2); shadow
# half-width at x=2.6 is 0.30*2.6/2.0 = 0.39 m (+0.15 own radius) → occluded
# ~1.8 s at 0.6 m/s > tracking_duration 1.0 s: the track dies mid-shadow and
# re-acquires on emergence — the §10.3 worst case, on purpose.
S4_LINE = {'x': 2.6, 'y0': -4.9, 'y1': 4.9}
S4_SPEED = 0.6
P4_MOCAP_BODIES = MOCAP_BODIES + S3_BODIES + [S4_BLOCKER['body']]


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


def build_mirror(scene, bodies_radii, out_xml):
    """Compile scene + mocap cylinders (name -> radius) and write out_xml."""
    spec = mujoco.MjSpec.from_file(scene)
    spec.meshdir = os.path.join(os.path.dirname(os.path.abspath(scene)),
                                spec.meshdir or 'assets')
    for name, radius in bodies_radii:
        body = spec.worldbody.add_body(name=name, mocap=True,
                                       pos=[PARK[0], PARK[1], CYL_HALF_H])
        body.add_geom(name=name + '_geom',
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[radius, CYL_HALF_H, 0],
                      rgba=[0.3, 0.3, 0.8, 1])
    model = spec.compile()
    assert model.nmocap == len(bodies_radii)
    with open(out_xml, 'w') as f:
        f.write(spec.to_xml())
    check = mujoco.MjModel.from_xml_path(out_xml)
    assert check.nq == model.nq and check.nmocap == len(bodies_radii)
    return model


def s3_obstacles(rng):
    """Seeded swarm: radii, box-reflect trajectories, min 0.9 m apart at t=0
    and >= 1.0 m from the origin (the robot stands there)."""
    out = []
    placed = []
    for i, body in enumerate(S3_BODIES):
        radius = round(rng.uniform(*S3_RADIUS_RANGE), 3)
        while True:
            p0 = [rng.uniform(*S3_BOX['x']), rng.uniform(*S3_BOX['y'])]
            if p0[0] * p0[0] + p0[1] * p0[1] < 1.0:
                continue
            if all((p0[0] - q[0]) ** 2 + (p0[1] - q[1]) ** 2 >= 0.9 ** 2
                   for q in placed):
                break
        placed.append(p0)
        speed = rng.uniform(*S3_SPEED_RANGE)
        heading = rng.uniform(0.0, 2.0 * math.pi)
        out.append({
            'body': body, 'mode': 'reflect', 'p0': p0,
            'v0': [round(speed * math.cos(heading), 4),
                   round(speed * math.sin(heading), 4)],
            'box': S3_BOX, 't_start': 0.0, 'radius': radius,
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/scenarios_phase3')
    ap.add_argument('--scene', default=DEFAULT_SCENE)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    xml_path = os.path.join(args.out, 'scenario_mirror.xml')
    model = build_mirror(args.scene, [(n, CYL_R) for n in MOCAP_BODIES],
                         xml_path)

    qpos = grounded_qpos(model)

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

    # ---- Phase 4: S3/S4 on the extended mirror ----
    rng = random.Random(S3_SEED)
    s3_obs = s3_obstacles(rng)
    p4_radii = ([(n, CYL_R) for n in MOCAP_BODIES] +
                [(ob['body'], ob['radius']) for ob in s3_obs] +
                [(S4_BLOCKER['body'], S4_BLOCKER['radius'])])
    p4_xml = os.path.join(args.out, 'scenario_mirror_p4.xml')
    p4_model = build_mirror(args.scene, p4_radii, p4_xml)
    p4_qpos = grounded_qpos(p4_model)
    p4_common = dict(common)
    p4_common['nq'] = int(p4_model.nq)
    p4_common['qpos'] = [float(v) for v in p4_qpos]
    p4_common['mocap_bodies'] = P4_MOCAP_BODIES

    s3 = dict(p4_common)
    s3['name'] = 's3_swarm'
    s3['duration_s'] = S3_DURATION_S
    s3['seed'] = S3_SEED
    s3['obstacles'] = s3_obs
    with open(os.path.join(args.out, 's3_swarm.json'), 'w') as f:
        json.dump(s3, f, indent=2)

    s4 = dict(p4_common)
    s4['name'] = 's4_occlusion'
    cross_s = (S4_LINE['y1'] - S4_LINE['y0']) / S4_SPEED
    s4['duration_s'] = S2_PREROLL_S + cross_s + S2_TAIL_S
    s4['obstacles'] = [
        {'body': S4_BLOCKER['body'], 'mode': 'static',
         'pos': S4_BLOCKER['pos'], 'radius': S4_BLOCKER['radius']},
        {'body': 's2_crosser', 'mode': 'cross',
         'p0': [S4_LINE['x'], S4_LINE['y0']],
         'p1': [S4_LINE['x'], S4_LINE['y1']],
         'speed': S4_SPEED, 't_start': S2_PREROLL_S, 'radius': CYL_R},
    ]
    with open(os.path.join(args.out, 's4_occlusion.json'), 'w') as f:
        json.dump(s4, f, indent=2)

    print(f'wrote {xml_path} (+ s1_static / s2_cross_05 / s2_cross_08 json)')
    print(f'nq={model.nq} nmocap={model.nmocap} pelvis_z={qpos[2]:.4f}')
    print(f'wrote {p4_xml} (+ s3_swarm / s4_occlusion json)')
    print(f'p4: nq={p4_model.nq} nmocap={p4_model.nmocap}')


if __name__ == '__main__':
    main()
