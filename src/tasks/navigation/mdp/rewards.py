"""Navigation and pure-DPCBF reward terms."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state
from .terminations import arena_contact, goal_reached, obstacle_contact


def _active(env) -> torch.Tensor:
  return (~env.reset_buf).float()


def goal_progress(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(state.goal - robot_pos, dim=-1)
  progress = state.previous_goal_distance - distance
  state.previous_goal_distance[:] = distance
  return progress * _active(env)


def heading_progress(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  """Reward reduced heading error only near the goal."""
  state = get_navigation_state(env, cfg)
  robot_pos, yaw, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(state.goal - robot_pos, dim=-1)
  heading_error = torch.abs(
    torch.atan2(
      torch.sin(state.goal_heading - yaw),
      torch.cos(state.goal_heading - yaw),
    )
  )
  progress = state.previous_heading_error - heading_error
  state.previous_heading_error[:] = heading_error
  scale = cfg.goal.heading_progress_distance_scale
  gate = torch.exp(-torch.square(distance / scale))
  return progress * gate * _active(env)


def goal_reached_reward(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  reached = goal_reached(env, cfg)
  return (
    reached & ~obstacle_contact(env, cfg) & ~arena_contact(env, cfg)
  ).float()


def obstacle_collision_penalty(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  return obstacle_contact(env, cfg).float()


def wall_collision_penalty(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  return arena_contact(env, cfg).float()


def route_box_penalty(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  """Penalize time outside the start-goal diagonal rectangle plus 0.5 m."""
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  margin = cfg.arena.route_box_margin
  lower = torch.minimum(state.start_position, state.goal) - margin
  upper = torch.maximum(state.start_position, state.goal) + margin
  return ((robot_pos < lower) | (robot_pos > upper)).any(dim=-1).float()


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


def termination_penalty(env, term_name: str) -> torch.Tensor:
  return env.termination_manager.get_term(term_name).float()
