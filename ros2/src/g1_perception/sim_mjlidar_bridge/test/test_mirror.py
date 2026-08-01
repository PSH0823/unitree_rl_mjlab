"""Mirror-correctness unit test (§5 of the phase prompt): a known qpos fed to
MirrorModel must reproduce the site pose an independent mj_kinematics
computation gives, and state-length mismatches must be rejected loudly."""
import os
import tempfile

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from sim_mjlidar_bridge.mirror import MirrorModel  # noqa: E402

MODEL_XML = """
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="base" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="arm" pos="0 0 0.2">
        <joint name="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.02"/>
        <site name="tip" pos="0.4 0 0.05" euler="3.14159265 0 0"/>
      </body>
    </body>
    <body name="puck" mocap="true" pos="1 0 0">
      <geom type="cylinder" size="0.25 0.75" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture()
def xml_path(tmp_path):
    p = tmp_path / "model.xml"
    p.write_text(MODEL_XML)
    return str(p)


def test_site_pose_matches_reference(xml_path):
    mirror = MirrorModel(xml_path)
    qpos = np.array([0.3, -0.2, 1.4, 0.9689124, 0.0, 0.24740396, 0.0, 0.7])
    mocap_pos = [2.0, 1.0, 0.5]
    mocap_quat = [1.0, 0.0, 0.0, 0.0]
    mirror.set_state(qpos, mocap_pos, mocap_quat)
    pos, mat = mirror.site_pose("tip")

    # independent reference computation
    ref_m = mujoco.MjModel.from_xml_path(xml_path)
    ref_d = mujoco.MjData(ref_m)
    ref_d.qpos[:] = qpos
    ref_d.mocap_pos[0] = mocap_pos
    ref_d.mocap_quat[0] = mocap_quat
    mujoco.mj_kinematics(ref_m, ref_d)

    assert np.allclose(pos, ref_d.site("tip").xpos, atol=1e-12)
    assert np.allclose(mat, np.array(ref_d.site("tip").xmat).reshape(3, 3),
                       atol=1e-12)
    # mocap must have reached the mirror (raycast sees obstacles through it)
    assert np.allclose(mirror.data.mocap_pos[0], mocap_pos)


def test_moving_joint_moves_site(xml_path):
    mirror = MirrorModel(xml_path)
    base = [0, 0, 1, 1, 0, 0, 0]
    mirror.set_state(np.array(base + [0.0]), [1, 0, 0], [1, 0, 0, 0])
    p0, _ = mirror.site_pose("tip")
    mirror.set_state(np.array(base + [1.0]), [1, 0, 0], [1, 0, 0, 0])
    p1, _ = mirror.site_pose("tip")
    assert np.linalg.norm(p1 - p0) > 0.1


def test_length_mismatch_rejected(xml_path):
    mirror = MirrorModel(xml_path)
    with pytest.raises(ValueError, match="qpos length"):
        mirror.set_state(np.zeros(3), [0, 0, 0], [1, 0, 0, 0])
    with pytest.raises(ValueError, match="mocap"):
        mirror.set_state(np.zeros(8), [0, 0, 0, 0], [1, 0, 0, 0])


def test_free_body_qpos_addr(xml_path):
    mirror = MirrorModel(xml_path)
    assert mirror.free_body_qpos_addr("base") == 0
    with pytest.raises(ValueError):
        mirror.free_body_qpos_addr("arm")
    with pytest.raises(ValueError):
        mirror.free_body_qpos_addr("nope")
