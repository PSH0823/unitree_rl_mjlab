#!/usr/bin/python3
"""T2 wall-occlusion gate (§11.4, §16.2): the raycast backend must not let
rays pass through a large flat wall. H-3 (old branch, MuJoCo 3.3.6 C API)
measured mj_multiRay missing up to 23.7% of hits on large flat geoms at
grazing incidence via AABB-corner pruning — an error that is always a MISSED
hit, never a wrong range. PASS <=> zero through-wall rays.

Method: sensor at Mid360 standing height fires the real mid360 rosette at a
large thin wall 8 m ahead. For every ray, the exact ray-box intersection is
computed analytically (slab method); a ray that must hit the wall but returns
no hit / a hit beyond the wall is a through-wall ray. Several wall widths and
cutoffs are swept because the pruning failure depends on AABB extent.

Usage: ./t2_wall_occlusion.py [--backend cpu]
"""
import argparse
import os
import sys
import time

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
default_src = os.path.join(here, "..", "..", "..", "external", "MuJoCo-LiDAR", "src")
sys.path.insert(0, os.path.abspath(default_src))

import mujoco  # noqa: E402
from mujoco_lidar.lidar_wrapper import MjLidarWrapper  # noqa: E402
from mujoco_lidar.scan_gen import LivoxGenerator  # noqa: E402

SENSOR_Z = 1.2654          # H-1 optical origin height when standing
WALL_X = 8.0               # m ahead
WALL_THICK = 0.05          # half-thickness 0.025
TOL = 0.01                 # m


def make_model(half_w, half_h):
    xml = f"""
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="sensor_body" pos="0 0 {SENSOR_Z}">
      <site name="lidar" pos="0 0 0"/>
      <geom type="sphere" size="0.01" contype="0" conaffinity="0" group="2"/>
    </body>
    <geom name="wall" type="box"
          pos="{WALL_X} 0 {half_h}" size="{WALL_THICK / 2} {half_w} {half_h}"/>
  </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def ray_box_hit(origins, dirs, center, half):
    """Vectorized slab test. Returns (hit_mask, t_near)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
    t0 = (center - half - origins) * inv
    t1 = (center + half - origins) * inv
    tmin = np.minimum(t0, t1).max(axis=1)
    tmax = np.maximum(t0, t1).min(axis=1)
    hit = (tmax >= np.maximum(tmin, 0.0))
    return hit, tmin


def run_case(half_w, half_h, cutoff, theta, phi):
    m = make_model(half_w, half_h)
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)

    geomgroup = np.ones(6, dtype=np.uint8)
    geomgroup[2] = 0  # mask the sensor's own housing, as in production
    lidar = MjLidarWrapper(m, site_name="lidar", backend="cpu",
                           cutoff_dist=cutoff, args={"geomgroup": geomgroup})
    t_start = time.perf_counter()
    lidar.trace_rays(d, theta, phi)
    elapsed = time.perf_counter() - t_start
    dist = lidar.get_distances()

    dirs = np.stack([np.cos(phi) * np.cos(theta),
                     np.cos(phi) * np.sin(theta),
                     np.sin(phi)], axis=-1)
    origins = np.broadcast_to(np.array([0.0, 0.0, SENSOR_Z]), dirs.shape)
    center = np.array([WALL_X, 0.0, half_h])
    half = np.array([WALL_THICK / 2, half_w, half_h])
    expect_hit, t_near = ray_box_hit(origins, dirs, center, half)
    expect_hit &= (t_near < cutoff - TOL)

    missed = expect_hit & (dist < 0)
    passed_through = expect_hit & (dist > 0) & (dist > t_near + TOL)
    n_bad = int(np.count_nonzero(missed) + np.count_nonzero(passed_through))
    n_exp = int(np.count_nonzero(expect_hit))
    range_err = np.abs(dist[expect_hit & (dist > 0)] -
                       t_near[expect_hit & (dist > 0)])
    return n_exp, int(np.count_nonzero(missed)), int(np.count_nonzero(passed_through)), \
        n_bad, float(range_err.max() if range_err.size else 0.0), elapsed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="cpu")
    args = ap.parse_args()
    assert args.backend == "cpu", "gate currently pins the production backend"

    gen = LivoxGenerator("mid360")
    theta, phi = gen.sample_ray_angles()
    n = len(theta)

    print(f"T2: backend=cpu rays/frame={n} wall at {WALL_X} m, "
          f"sensor z={SENSOR_Z}")
    worst = 0
    total_expected = 0
    for half_w, half_h, cutoff in [
        (4.0, 1.5, 100.0),    # modest wall
        (20.0, 3.0, 100.0),   # large flat geom (H-3 trigger shape)
        (20.0, 3.0, 40.0),    # production cutoff
        (100.0, 5.0, 40.0),   # pathological AABB
    ]:
        n_exp, n_miss, n_thru, n_bad, max_err, dt = run_case(
            half_w, half_h, cutoff, theta, phi)
        total_expected += n_exp
        worst = max(worst, n_bad)
        print(f"T2: wall {2*half_w:6.1f}x{2*half_h:4.1f} m cutoff {cutoff:5.1f}: "
              f"expected hits {n_exp:5d}  missed {n_miss:4d}  "
              f"through {n_thru:4d}  ({100.0*n_bad/max(n_exp,1):6.3f}%)  "
              f"max range err {max_err*1000:.2f} mm  raycast {dt*1e3:.1f} ms")

    if worst == 0:
        print(f"T2: PASS - zero through-wall rays across {total_expected} "
              "expected hits in all cases")
        return 0
    print("T2: FAIL - backend misses occlusions (H-3); switch backend or patch "
          "to per-ray mj_ray")
    return 1


if __name__ == "__main__":
    sys.exit(main())
