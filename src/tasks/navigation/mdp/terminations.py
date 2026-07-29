"""Navigation termination conditions."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


def goal_reached(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  return (
    torch.linalg.vector_norm(
      state.goal - robot_pos, dim=-1
    )
    <= cfg.robot.r_rob + cfg.goal.radius
  )


def obstacle_collision(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(
    state.position - robot_pos[:, None, :], dim=-1
  )
  return (
    state.active & (distance <= state.radius + cfg.robot.r_rob)
  ).any(dim=-1)


def outside_arena(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  pos, _, _, _, _ = state.robot_planar_state()
  center = torch.tensor(cfg.arena.center, device=env.device)
  half = torch.tensor(cfg.arena.size, device=env.device) * 0.5
  return (torch.abs(pos - center) >= half - cfg.robot.r_rob).any(dim=-1)


def fallen(env, minimum_height: float = 0.45) -> torch.Tensor:
  return env.scene["robot"].data.root_link_pos_w[:, 2] < minimum_height


def time_out(env) -> torch.Tensor:
  return env.episode_length_buf >= env.max_episode_length
