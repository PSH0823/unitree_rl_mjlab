#!/usr/bin/python3
"""T7 extrinsic guard (§16.2): the MJCF mid360_link site and the xacro
mid360_joint must describe the same physical pose.

Compares ROTATION MATRICES, not raw numbers: MJCF euler (intrinsic xyz) and
URDF rpy (extrinsic xyz) do not commute at roll = pi, so equal-looking numbers
can hide a 2*pitch (~0.1 deg) rotation error - exactly the class of bug this
guard exists to catch.

Also asserts the base_link->torso_link fixed approximation in the xacro equals
the MJCF body chain at zero waist angles.

Exit 0 = pass. Runs under system python (R-11); needs `mujoco` + `xacro` CLI.
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import mujoco
except ImportError:
    print("T7: FAIL - python 'mujoco' not importable under", sys.executable)
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
XACRO = os.path.join(HERE, "..", "urdf", "g1_mid360.xacro")
SCENE = os.path.join(REPO, "src", "assets", "robots", "unitree_g1", "xmls", "scene_g1.xml")

TRANS_TOL = 1e-9          # m
ANGLE_TOL = 1e-6          # rad (a 0.000892-rad sign error shows up as 1.8e-3)


def rpy_to_mat(r, p, y):
    """URDF rpy: R = Rz(y) @ Ry(p) @ Rx(r) (extrinsic xyz)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_mat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_angle(Ra, Rb):
    return float(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0)))


def urdf_joint(root, name):
    for j in root.iter("joint"):
        if j.get("name") == name:
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in o.get("rpy", "0 0 0").split()])
            return xyz, rpy
    raise KeyError(name)


def main():
    failures = []

    # --- xacro side -------------------------------------------------------
    urdf_xml = subprocess.run(
        ["xacro", XACRO], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(urdf_xml)
    t_x, rpy_x = urdf_joint(root, "mid360_joint")
    t_torso_x, rpy_torso_x = urdf_joint(root, "torso_joint")
    R_x = rpy_to_mat(*rpy_x)

    # H-1 literal guard on the xacro itself (Appendix A verbatim)
    h1_xyz = np.array([0.0002835, 0.00003, 0.428434])
    h1_rpy = np.array([3.14159265, 0.000892, 0.0])
    if not np.allclose(t_x, h1_xyz, atol=1e-12):
        failures.append(f"xacro mid360 xyz {t_x} != H-1 {h1_xyz}")
    if not np.allclose(rpy_x, h1_rpy, atol=1e-12):
        failures.append(f"xacro mid360 rpy {rpy_x} != H-1 {h1_rpy}")

    # --- MJCF side --------------------------------------------------------
    if not os.path.exists(SCENE):
        failures.append(f"scene not found: {SCENE}")
        report(failures)
    m = mujoco.MjModel.from_xml_path(SCENE)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "mid360_link")
    if sid < 0:
        failures.append("MJCF site 'mid360_link' missing from scene_g1.xml "
                        "(Phase 1 scene edit not applied yet - T7 is a hard "
                        "gate from the end of Phase 1)")
        report(failures)

    body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.site_bodyid[sid])
    if body_name != "torso_link":
        failures.append(f"site parent body {body_name} != torso_link")

    t_m = np.array(m.site_pos[sid])
    R_m = quat_to_mat(*m.site_quat[sid])

    # --- compare torso->mid360 -------------------------------------------
    dt = float(np.max(np.abs(t_m - t_x)))
    da = rot_angle(R_x, R_m)
    print(f"T7: torso->mid360 translation diff {dt:.3e} m, rotation diff {da:.3e} rad")
    if dt > TRANS_TOL:
        failures.append(f"translation diff {dt:.3e} m > {TRANS_TOL}")
    if da > ANGLE_TOL:
        failures.append(f"rotation diff {da:.3e} rad > {ANGLE_TOL} "
                        "(hint: MJCF euler is intrinsic xyz - the pitch sign "
                        "must be NEGATED relative to URDF rpy at roll=pi)")

    # --- compare base->torso nominal chain -------------------------------
    chain = ["waist_yaw_link", "waist_roll_link", "torso_link"]
    t_chain = np.zeros(3)
    for b in chain:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        if bid < 0:
            failures.append(f"MJCF body missing: {b}")
            report(failures)
        if not np.allclose(m.body_quat[bid], [1, 0, 0, 0], atol=1e-12):
            failures.append(f"MJCF body {b} has non-identity quat; xacro "
                            "torso_joint approximation no longer valid")
        t_chain += m.body_pos[bid]
    dt_torso = float(np.max(np.abs(t_chain - t_torso_x)))
    print(f"T7: base->torso nominal diff {dt_torso:.3e} m")
    if dt_torso > TRANS_TOL:
        failures.append(f"base->torso xacro {t_torso_x} != MJCF chain {t_chain}")
    if np.any(rpy_torso_x != 0):
        failures.append(f"xacro torso_joint rpy expected zero, got {rpy_torso_x}")

    report(failures)


def report(failures):
    if failures:
        print("T7: FAIL")
        for f in failures:
            print("  -", f)
        sys.exit(1)
    print("T7: PASS - MJCF site pose == xacro pose (matrix comparison)")
    sys.exit(0)


if __name__ == "__main__":
    main()
