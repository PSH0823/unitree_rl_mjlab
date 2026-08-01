#!/usr/bin/python3
"""Measured-wall validation scene generator (Phase 2 gate, §18; §16.1).

Builds the G1 scene (scene_g1.xml, with the H-1 mid360_link site) plus
surveyed validation geometry, compiles it with python mujoco, and writes:

  <out>/wall_scene_mirror.xml   mirror model for sim_mjlidar_bridge
  <out>/wall_scene_gt.json      grounded standing qpos + surveyed GT table

Surveyed geometry (distances are from the robot base origin (0,0) — which is
the base_footprint origin, i.e. the /scan range origin — to the NEAREST
surface):

  wall_1m   box face  ⊥ +x  at bearing   0.0°, range 1.000 m
  wall_2m   box face  ⊥ +y  at bearing  90.0°, range 2.000 m
  wall_4m   box face  ⊥ −x  at bearing 180.0°, range 4.000 m
  cyl_1m    cylinder r=0.15 at bearing  45.0°, face range 1.000 m
  cyl_2m    cylinder r=0.15 at bearing 135.0°, face range 2.000 m
  cyl_3m    cylinder r=0.15 at bearing −45.0°, face range 3.000 m

Bearing spans never overlap (walls are 1 m wide: ±26.6° at 1 m, ±14.3° at
2 m, ±7.2° at 4 m around their bearings). Walls are 2.5 m tall and cylinders
1.5 m (the sim obstacle height), so every target intersects the §9.4 height
band [0.15, 1.60] from the −7°…+52° Mid360 elevation fan at standing sensor
height. The cylinders are the Q-3 bin-occupancy baseline props (§17.4).

The robot pose is GROUNDED: zero joints (G1 zero pose = straight standing),
pelvis lowered until the lowest robot-geom AABB corner touches z=0. No
dynamics ever run on this scene (the sidecar mirror is kinematic), so the
pose is exact and permanent — this sidesteps the no-gamepad/no-FixStand
constraint that forced Phase 1's visual evidence into the suspension rig.

Phase 5 repeats this test against a real wall with a tape measure.

Usage: make_wall_scene.py [--out DIR] [--scene PATH]
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

WALL_HALF_W = 0.50      # m; keeps bearing spans of the six targets disjoint
WALL_HALF_T = 0.025     # m
WALL_HALF_H = 1.25      # m (top 2.5 m > band top 1.6 m)
CYL_R = 0.15            # m (standard sim obstacle radius)
CYL_HALF_H = 0.75       # m (1.5 m tall, the sim obstacle height)

# name, kind, bearing_deg, surveyed face range [m]
TARGETS = [
    ('wall_1m', 'wall', 0.0, 1.0),
    ('wall_2m', 'wall', 90.0, 2.0),
    ('wall_4m', 'wall', 180.0, 4.0),
    ('cyl_1m', 'cylinder', 45.0, 1.0),
    ('cyl_2m', 'cylinder', 135.0, 2.0),
    ('cyl_3m', 'cylinder', -45.0, 3.0),
]


def add_targets(spec):
    for name, kind, bearing_deg, face in TARGETS:
        b = math.radians(bearing_deg)
        if kind == 'wall':
            center_d = face + WALL_HALF_T
            pos = [center_d * math.cos(b), center_d * math.sin(b), WALL_HALF_H]
            # slab normal along the bearing: size x = half thickness, rotate
            # about z by the bearing so the inner face is exactly at `face`.
            spec.worldbody.add_geom(
                name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[WALL_HALF_T, WALL_HALF_W, WALL_HALF_H], pos=pos,
                quat=[math.cos(b / 2), 0, 0, math.sin(b / 2)],
                rgba=[0.8, 0.3, 0.3, 1])
        else:
            center_d = face + CYL_R
            pos = [center_d * math.cos(b), center_d * math.sin(b), CYL_HALF_H]
            spec.worldbody.add_geom(
                name=name, type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=[CYL_R, CYL_HALF_H, 0], pos=pos,
                rgba=[0.3, 0.3, 0.8, 1])


def grounded_qpos(model):
    """Zero-joint standing pose with the lowest robot AABB corner at z=0."""
    data = mujoco.MjData(model)
    qpos = np.zeros(model.nq)
    qpos[2] = 0.793          # scene default pelvis height, refined below
    qpos[3] = 1.0            # identity quat (w x y z)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)

    min_z = np.inf
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0:
            continue         # floor / walls / cylinders live on worldbody
        center = model.geom_aabb[g, :3]
        half = model.geom_aabb[g, 3:]
        xmat = data.geom_xmat[g].reshape(3, 3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corner = center + half * np.array([sx, sy, sz])
                    min_z = min(min_z, (data.geom_xpos[g] + xmat @ corner)[2])
    qpos[2] -= min_z
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    site = data.site('mid360_link')
    return qpos, float(site.xpos[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/wall_scene_phase2')
    ap.add_argument('--scene', default=DEFAULT_SCENE)
    args = ap.parse_args()

    spec = mujoco.MjSpec.from_file(args.scene)
    # mj_saveXML/to_xml keep meshdir as written; the mirror loads the XML from
    # --out, so the meshdir must be absolute (same fix as simulate's dump).
    spec.meshdir = os.path.join(os.path.dirname(os.path.abspath(args.scene)),
                                spec.meshdir or 'assets')
    add_targets(spec)
    model = spec.compile()

    qpos, sensor_z = grounded_qpos(model)

    os.makedirs(args.out, exist_ok=True)
    xml_path = os.path.join(args.out, 'wall_scene_mirror.xml')
    with open(xml_path, 'w') as f:
        f.write(spec.to_xml())
    # the mirror must load what we just wrote
    check = mujoco.MjModel.from_xml_path(xml_path)
    assert check.nq == model.nq and check.nmocap == 0

    gt = {
        'scene': args.scene,
        'nq': int(model.nq),
        'qpos': [float(v) for v in qpos],
        'sensor_height_m': sensor_z,
        'targets': [
            {'name': n, 'kind': k, 'bearing_deg': b, 'range_m': r,
             'radius_m': CYL_R if k == 'cylinder' else None}
            for n, k, b, r in TARGETS],
    }
    json_path = os.path.join(args.out, 'wall_scene_gt.json')
    with open(json_path, 'w') as f:
        json.dump(gt, f, indent=2)
    print(f'wrote {xml_path}')
    print(f'wrote {json_path}')
    print(f'nq={model.nq} sensor_height={sensor_z:.4f} m '
          f'pelvis_z={qpos[2]:.4f} m (grounded)')


if __name__ == '__main__':
    main()
