"""Pose-only mirror of the running simulation model (§11.2).

The mirror loads the COMPILED model dumped by the simulate ROS2 module
(mj_saveXML of the post-AddToSpec spec) — never the raw scene_g1.xml, which
lacks the runtime-added obstacle mocap bodies. State written here comes from
/sim/mj_state; only mj_kinematics runs (poses, no dynamics).
"""
import numpy as np

import mujoco


class MirrorModel:
    def __init__(self, xml_path: str):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

    def set_state(self, qpos, mocap_pos, mocap_quat) -> None:
        m, d = self.model, self.data
        if len(qpos) != m.nq:
            raise ValueError(f"qpos length {len(qpos)} != mirror nq {m.nq} "
                             "(mirror model out of date? re-dump it)")
        if len(mocap_pos) != 3 * m.nmocap or len(mocap_quat) != 4 * m.nmocap:
            raise ValueError(
                f"mocap lengths ({len(mocap_pos)},{len(mocap_quat)}) != mirror "
                f"nmocap {m.nmocap} * (3,4)")
        d.qpos[:] = qpos
        if m.nmocap:
            d.mocap_pos[:] = np.asarray(mocap_pos).reshape(m.nmocap, 3)
            d.mocap_quat[:] = np.asarray(mocap_quat).reshape(m.nmocap, 4)
        mujoco.mj_kinematics(m, d)

    def site_pose(self, name: str):
        s = self.data.site(name)
        return np.array(s.xpos), np.array(s.xmat).reshape(3, 3)

    def free_body_qpos_addr(self, body_name: str) -> int:
        m = self.model
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            raise ValueError(f"body '{body_name}' not in mirror model")
        jid = m.body_jntadr[bid]
        if jid < 0 or m.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"body '{body_name}' has no free joint")
        return int(m.jnt_qposadr[jid])
