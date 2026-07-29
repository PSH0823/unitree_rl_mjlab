"""Success-rate-driven obstacle curriculum."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


def obstacle_curriculum(env, env_ids, cfg: NavigationTaskCfg):
  state = get_navigation_state(env, cfg)
  if not cfg.curriculum.enabled or env_ids is None:
    return {"stage": state.curriculum_stage, "success_ema": state.success_ema}
  ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.arange(
    env.num_envs, device=env.device
  )[env_ids]
  completed = env.episode_length_buf[ids] > 0
  if completed.any():
    ids = ids[completed]
    robot_pos_all, _, _, _, _ = state.robot_planar_state()
    robot_pos = robot_pos_all[ids]
    successes = (
      torch.linalg.vector_norm(state.goal[ids] - robot_pos, dim=-1)
      <= cfg.robot.r_rob + cfg.goal.radius
    ).float()
    rate = cfg.curriculum.success_ema_rate
    state.success_ema.mul_(1.0 - rate).add_(rate * successes.mean())
    max_stage = len(cfg.curriculum.stages) - 1
    if (
      state.success_ema >= cfg.curriculum.promote_threshold
      and state.curriculum_stage < max_stage
    ):
      state.curriculum_stage += 1
      state.success_ema.fill_(0.5)
    elif (
      state.success_ema <= cfg.curriculum.demote_threshold
      and state.curriculum_stage > 0
    ):
      state.curriculum_stage -= 1
      state.success_ema.fill_(0.65)
  return {
    "stage": state.curriculum_stage,
    "success_ema": state.success_ema,
  }
