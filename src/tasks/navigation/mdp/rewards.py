"""Navigation and pure-DPCBF reward terms."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


def _active(env) -> torch.Tensor:
  return (~env.reset_buf).float()


def goal_progress(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(state.goal - robot_pos, dim=-1)
  progress = state.previous_goal_distance - distance
  state.previous_goal_distance[:] = distance
  return progress * _active(env)


def goal_reached_reward(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(
    state.goal - robot_pos, dim=-1
  )
  reached = distance <= cfg.robot.r_rob + cfg.goal.radius
  collision = env.termination_manager.get_term("collision")
  wall_collision = env.termination_manager.get_term("outside_arena")
  return (reached & ~collision & ~wall_collision).float()


def obstacle_collision_penalty(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(
    state.position - robot_pos[:, None, :], dim=-1
  )
  collision = state.active & (
    distance <= state.radius + cfg.robot.r_rob
  )
  return collision.any(dim=-1).float()


def wall_collision_penalty(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  center = torch.tensor(cfg.arena.center, device=env.device)
  half = torch.tensor(cfg.arena.size, device=env.device) * 0.5
  wall_collision = (
    torch.abs(robot_pos - center) >= half - cfg.robot.r_rob
  ).any(dim=-1)
  obstacle_collision = env.termination_manager.get_term("collision")
  return (wall_collision & ~obstacle_collision).float()


def cbf_reward(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  """Table-II CBF reward for the active DPCBF constraint."""
  action_term = env.action_manager.get_term("navigation")
  values = action_term.dpcbf_values
  if values is None:
    return torch.zeros(env.num_envs, device=env.device)
  condition = torch.where(
    values.valid,
    values.condition,
    torch.full_like(values.condition, torch.inf),
  ).min(dim=-1).values
  condition = torch.where(
    values.valid.any(dim=-1), condition, torch.zeros_like(condition)
  )
  intervention = (action_term.policy_action - action_term.safe_action).square().sum(
    dim=-1
  )
  sigma_squared = cfg.dpcbf.reward_sigma**2
  reward = (
    torch.minimum(condition, torch.zeros_like(condition))
    + torch.exp(-intervention / sigma_squared)
    - 1.0
  )
  return reward * _active(env)


def action_rate(env) -> torch.Tensor:
  return torch.sum(
    torch.square(env.action_manager.action - env.action_manager.prev_action), dim=-1
  )


def constant(env) -> torch.Tensor:
  return _active(env)


def timeout_penalty(env) -> torch.Tensor:
  return env.reset_time_outs.float()
