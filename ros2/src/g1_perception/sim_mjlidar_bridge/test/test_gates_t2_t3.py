"""Pytest wrappers running gates T2 (wall occlusion) and T3 (pattern
envelope) as part of `colcon test`. Both are pure-python (mujoco +
MuJoCo-LiDAR) — no simulator needed."""
import os
import subprocess
import sys

import pytest

mujoco = pytest.importorskip("mujoco")

# python-mujoco pin (decided in the interim block after Phase 5A, carried since
# Phase 1). PIN = 3.6.0, deliberately NOT the 3.3.6 the simulator vendors on the
# C side. H-3 is a real `mj_multiRay` AABB-pruning miss measured on 3.3.6 — rays
# pass through walls — and T2 passes on 3.6.0 precisely because it is fixed
# there. Matching the C runtime for symmetry would trade a passing safety gate
# for a failing one and force the T2 backend switch. The skew is bounded: the C
# side does physics, the python side raycasts the mirror model, and the only
# thing they exchange is pose (`/sim/mj_state`), which test_mirror.py and T7
# already assert agree for the same MJCF.
MUJOCO_PIN = (3, 6, 0)


def _ver(v):
    return tuple(int(x) for x in v.split('.')[:3])


def test_python_mujoco_pin():
    """Fail loudly on drift below the pin rather than let H-3 back in silently."""
    assert _ver(mujoco.__version__) >= MUJOCO_PIN, (
        f'python mujoco {mujoco.__version__} is older than the {MUJOCO_PIN} pin; '
        'H-3 (mj_multiRay misses occlusions) is unfixed below it and T2 is '
        'expected to FAIL. Re-run T2 before changing this pin.')


HERE = os.path.dirname(os.path.abspath(__file__))
GATES = os.path.join(HERE, "..", "test_gates")
MJLIDAR = os.path.join(HERE, "..", "..", "..", "external", "MuJoCo-LiDAR", "src")

if not os.path.isdir(MJLIDAR):
    pytest.skip("MuJoCo-LiDAR checkout missing (run ros2/setup_external.sh)",
                allow_module_level=True)


def _run(script):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.abspath(MJLIDAR) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run([sys.executable, os.path.join(GATES, script)],
                          capture_output=True, text=True, env=env, timeout=300)
    sys.stdout.write(proc.stdout)
    sys.stderr.write(proc.stderr)
    return proc.returncode


def test_t2_wall_occlusion():
    assert _run("t2_wall_occlusion.py") == 0


def test_t3_pattern_envelope():
    assert _run("t3_pattern_envelope.py") == 0
