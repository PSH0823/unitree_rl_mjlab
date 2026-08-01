"""Pytest wrappers running gates T2 (wall occlusion) and T3 (pattern
envelope) as part of `colcon test`. Both are pure-python (mujoco +
MuJoCo-LiDAR) — no simulator needed."""
import os
import subprocess
import sys

import pytest

pytest.importorskip("mujoco")

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
