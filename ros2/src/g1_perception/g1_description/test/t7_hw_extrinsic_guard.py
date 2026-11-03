#!/usr/bin/python3
"""T7-hw extrinsic guard (Phase 5A): the hardware odometry config must agree
with the xacro, which is the single source of truth for extrinsics (§8.3).

DLIO has no TF listener. Its extrinsics/baselink2{lidar,imu} parameters are
the ONLY thing that tells it where base_link sits relative to the sensor, so
a stale copy there does not produce a TF conflict anyone would notice - it
silently redefines base_link and every obstacle in odom moves with it. This
guard recomputes the two transforms from g1_mid360.xacro and compares them to
g1_perception_bringup/config/dlio.yaml.

  baselink2lidar = (base_link->torso_link) o (torso_link->mid360_link, H-1)
  baselink2imu   = baselink2lidar o (mid360 lidar->IMU offset)

The lidar->IMU offset is the Livox Mid-360 manual constant, NOT measured on
this unit (5B checklist item); it is asserted here only so that the yaml and
this script cannot disagree about which constant was used.

Exit 0 = pass. Runs under system python (R-11); needs `xacro` CLI + numpy.
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, "..", "urdf", "g1_mid360.xacro")
DLIO_YAML = os.path.abspath(os.path.join(
    HERE, "..", "..", "g1_perception_bringup", "config", "dlio.yaml"))

# Livox Mid-360 user manual: IMU origin expressed in the point-cloud frame,
# rotation identity. (FAST-LIO configs carry the negation as extrinsic_T.)
LIDAR_TO_IMU_T = np.array([0.011, 0.02329, -0.04412])

TRANS_TOL = 1e-6          # m  - yaml is written to 9 decimals
ANGLE_TOL = 1e-6          # rad


def rpy_to_mat(r, p, y):
    """URDF rpy: R = Rz(y) @ Ry(p) @ Rx(r) (extrinsic xyz)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_angle(Ra, Rb):
    """Angle between two rotations, via the skew part of Ra^T Rb.

    NOT arccos((tr-1)/2) like t7_extrinsic_guard.py: that form is fine when
    the matrices agree bitwise but amplifies rounding by ~1/sqrt(eps) near
    identity, so a yaml rounded to 9 decimals reads as a 2e-5 rad error.
    Exact to first order for the small angles this guard actually sees.
    """
    Rd = Ra.T @ Rb
    v = 0.5 * np.array([Rd[2, 1] - Rd[1, 2],
                        Rd[0, 2] - Rd[2, 0],
                        Rd[1, 0] - Rd[0, 1]])
    s = float(np.linalg.norm(v))
    c = (np.trace(Rd) - 1.0) / 2.0
    return float(np.arctan2(s, c))


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

    urdf_xml = subprocess.run(
        ["xacro", XACRO], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(urdf_xml)
    t_torso, rpy_torso = urdf_joint(root, "torso_joint")
    t_mid, rpy_mid = urdf_joint(root, "mid360_joint")

    R_torso = rpy_to_mat(*rpy_torso)
    R_mid = rpy_to_mat(*rpy_mid)

    R_bl = R_torso @ R_mid
    t_bl = t_torso + R_torso @ t_mid
    t_bi = t_bl + R_bl @ LIDAR_TO_IMU_T

    if not os.path.exists(DLIO_YAML):
        print("T7-hw: FAIL - dlio config not found:", DLIO_YAML)
        sys.exit(1)
    with open(DLIO_YAML) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]

    for name, want_t, want_R in (
            ("baselink2lidar", t_bl, R_bl),
            ("baselink2imu", t_bi, R_bl)):
        got_t = np.array(params[f"extrinsics/{name}/t"], dtype=float)
        got_R = np.array(params[f"extrinsics/{name}/R"], dtype=float).reshape(3, 3)
        dt = float(np.max(np.abs(got_t - want_t)))
        da = rot_angle(want_R, got_R)
        print(f"T7-hw: {name} translation diff {dt:.3e} m, rotation diff {da:.3e} rad")
        if dt > TRANS_TOL:
            failures.append(f"{name}/t {got_t} != xacro-derived {want_t} "
                            f"(diff {dt:.3e} m) - regenerate from the xacro, "
                            "do not hand-edit dlio.yaml")
        if da > ANGLE_TOL:
            failures.append(f"{name}/R differs from xacro-derived by "
                            f"{da:.3e} rad")

    # The two frame names DLIO broadcasts must not collide with the
    # robot_state_publisher tree, or mid360_link/torso_link gets two parents.
    reserved = {"mid360_link", "torso_link", "base_link", "base_footprint", "odom"}
    for key in ("frames/lidar", "frames/imu"):
        val = params.get(key)
        if val in reserved:
            failures.append(f"{key}={val} collides with the §8.2 TF tree; "
                            "DLIO would give that frame a second parent")
    if params.get("frames/baselink") != "base_link":
        failures.append("frames/baselink must be base_link (§8.1)")
    if params.get("frames/odom") != "odom":
        failures.append("frames/odom must be odom (§8.1)")
    if params.get("use_sim_time") is not False:
        failures.append("dlio.yaml must set use_sim_time: false — upstream "
                        "ships true and there is no /clock on hardware")

    if failures:
        print("T7-hw: FAIL")
        for f in failures:
            print("  -", f)
        sys.exit(1)
    print("T7-hw: PASS - dlio.yaml extrinsics == xacro chain; frames disjoint")
    sys.exit(0)


if __name__ == "__main__":
    main()
