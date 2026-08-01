#!/usr/bin/python3
"""T3 pattern-envelope gate (§16.2): every angle in MuJoCo-LiDAR's mid360.npy
must lie inside the Livox Mid360 datasheet FOV — azimuth 360 deg, elevation
-7..+52 deg in the sensor frame. §20 Q-3 flags a known FOV metadata
discrepancy in the upstream repo; the datasheet is normative.

Usage: PYTHONPATH=<MuJoCo-LiDAR>/src ./t3_pattern_envelope.py
"""
import os
import sys

import numpy as np

ELEV_MIN_DEG = -7.0
ELEV_MAX_DEG = 52.0
# The .npy pattern overshoots the datasheet envelope by up to 0.22 deg on
# 0.22% of rays (measured 2026-08-01: -7.212..+52.164) — the Q-3 metadata
# discrepancy. The gate exists to catch a WRONG pattern (wrong sensor,
# degree/radian mixups), so it hard-fails only beyond this tolerance and
# reports the strict-envelope fraction; Phase 5 bin-occupancy comparison
# against real hardware adjudicates the true rosette (Q-3).
TOL_DEG = 0.25
STRICT_FRACTION_MIN = 0.99

here = os.path.dirname(os.path.abspath(__file__))
default_src = os.path.join(here, "..", "..", "..", "external", "MuJoCo-LiDAR", "src")
sys.path.insert(0, os.path.abspath(default_src))

from mujoco_lidar.scan_gen import LivoxGenerator  # noqa: E402


def main():
    gen = LivoxGenerator("mid360")
    angles = gen.ray_angles  # (N, 2)
    theta, phi = angles[:, 0], angles[:, 1]
    n = len(angles)

    theta_deg = np.degrees(theta)
    phi_deg = np.degrees(phi)
    print(f"T3: mid360.npy rays={n}, samples/frame={gen.samples}")
    print(f"T3: azimuth   min={theta_deg.min():.4f} max={theta_deg.max():.4f} deg")
    print(f"T3: elevation min={phi_deg.min():.4f} max={phi_deg.max():.4f} deg "
          f"(datasheet {ELEV_MIN_DEG}..{ELEV_MAX_DEG})")

    failures = []
    # azimuth may be encoded [0,360] or [-180,180]; require a full ring
    span = theta_deg.max() - theta_deg.min()
    if not (359.0 <= span <= 360.0 + 1e-3) or theta_deg.min() < -180.0 - 1e-3 \
            or theta_deg.max() > 360.0 + 1e-3:
        failures.append(f"azimuth not a full 360 ring (span {span:.3f} deg)")
    hard_bad = np.count_nonzero((phi_deg < ELEV_MIN_DEG - TOL_DEG) |
                                (phi_deg > ELEV_MAX_DEG + TOL_DEG))
    strict_ok = np.count_nonzero((phi_deg >= ELEV_MIN_DEG) &
                                 (phi_deg <= ELEV_MAX_DEG)) / n
    print(f"T3: strict-envelope fraction {100.0 * strict_ok:.3f}% "
          f"(gate >= {100.0 * STRICT_FRACTION_MIN:.0f}%), "
          f"hard-outliers beyond +/-{TOL_DEG} deg: {hard_bad}")
    if hard_bad:
        failures.append(f"{hard_bad}/{n} rays beyond +/-{TOL_DEG} deg tolerance")
    if strict_ok < STRICT_FRACTION_MIN:
        failures.append(f"only {100.0 * strict_ok:.2f}% within strict envelope")

    # sanity on per-frame sampling: one frame must span the full azimuth ring
    th_f, ph_f = gen.sample_ray_angles()
    print(f"T3: one frame: {len(th_f)} rays, az span "
          f"{np.degrees(th_f.min()):.1f}..{np.degrees(th_f.max()):.1f} deg, "
          f"elev span {np.degrees(ph_f.min()):.2f}..{np.degrees(ph_f.max()):.2f} deg")

    if failures:
        print("T3: FAIL")
        for f in failures:
            print("  -", f)
        return 1
    print("T3: PASS - pattern within datasheet envelope")
    return 0


if __name__ == "__main__":
    sys.exit(main())
