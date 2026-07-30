"""Navigation termination conditions."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


def _wrapped_angle(angle: torch.Tensor) -> torch.Tensor:
  return torch.atan2(torch.sin(angle), torch.cos(angle))


def obstacle_contact(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  """Instantaneous planar robot-obstacle overlap."""
  state = get_navigation_state(env, cfg)
  robot_pos, _, _, _, _ = state.robot_planar_state()
  distance = torch.linalg.vector_norm(
    state.position - robot_pos[:, None, :], dim=-1
  )
  return (
    state.active & (distance <= state.radius + cfg.robot.r_rob)
  ).any(dim=-1)


def arena_contact(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  """Instantaneous footprint crossing of the fixed arena boundary."""
  state = get_navigation_state(env, cfg)
  pos, _, _, _, _ = state.robot_planar_state()
  center = torch.tensor(cfg.arena.center, device=env.device)
  half = torch.tensor(cfg.arena.size, device=env.device) * 0.5
  return (torch.abs(pos - center) >= half - cfg.robot.r_rob).any(dim=-1)


def goal_reached(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_pos, yaw, vel_b, _, yaw_rate = state.robot_planar_state()
  position_ok = (
    torch.linalg.vector_norm(state.goal - robot_pos, dim=-1)
    <= cfg.robot.r_rob + cfg.goal.radius
  )
  tolerance = torch.deg2rad(
    torch.tensor(
      state.difficulty()["goal_heading_tolerance_deg"], device=env.device
    )
  )
  heading_ok = _wrapped_angle(state.goal_heading - yaw).abs() <= tolerance
  speed_ok = (
    torch.linalg.vector_norm(vel_b, dim=-1) <= cfg.goal.maximum_speed
  )
  yaw_rate_ok = yaw_rate.abs() <= cfg.goal.maximum_yaw_rate
  return position_ok & heading_ok & speed_ok & yaw_rate_ok


def obstacle_collision(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  collision = obstacle_contact(env, cfg)
  state.collision_steps[:] = torch.where(
    collision, state.collision_steps + 1, torch.zeros_like(state.collision_steps)
  )
  grace_steps = max(1, round(cfg.robot.collision_grace_s * cfg.high_level_hz))
  return state.collision_steps >= grace_steps


def outside_arena(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  pos, _, _, _, _ = state.robot_planar_state()
  center = torch.tensor(cfg.arena.center, device=env.device)
  half = torch.tensor(cfg.arena.size, device=env.device) * 0.5
  outside = arena_contact(env, cfg)
  state.outside_arena_steps[:] = torch.where(
    outside,
    state.outside_arena_steps + 1,
    torch.zeros_like(state.outside_arena_steps),
  )
  grace_steps = max(1, round(cfg.arena.outside_grace_s * cfg.high_level_hz))
  timed_outside = state.outside_arena_steps >= grace_steps
  footprint_excess = (
    torch.abs(pos - center) - (half - cfg.robot.r_rob)
  ).amax(dim=-1)
  far_outside = footprint_excess >= cfg.arena.hard_outside_distance
  return timed_outside | far_outside


def fallen(env, minimum_height: float = 0.45) -> torch.Tensor:
  return env.scene["robot"].data.root_link_pos_w[:, 2] < minimum_height


def time_out(env) -> torch.Tensor:
  return env.episode_length_buf >= env.max_episode_length
