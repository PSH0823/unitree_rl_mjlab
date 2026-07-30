"""Actor/critic observations for graph navigation."""

from __future__ import annotations

import torch

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


def _world_to_body(vector: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
  c, s = torch.cos(yaw), torch.sin(yaw)
  x = c[:, None] * vector[..., 0] + s[:, None] * vector[..., 1]
  y = -s[:, None] * vector[..., 0] + c[:, None] * vector[..., 1]
  return torch.stack((x, y), dim=-1)


def graph_observation(
  env,
  cfg: NavigationTaskCfg,
  noisy: bool = True,
  robot_state: tuple[torch.Tensor, ...] | None = None,
) -> torch.Tensor:
  """Return flattened ``[robot, obstacles, goal]`` nodes.

  Each node is ``[one_hot_label(3), rel_x, rel_y, radius, rel_vx, rel_vy,
  valid]``. Coordinates and velocities are robot-yaw-frame quantities.
  """
  state = get_navigation_state(env, cfg)
  if robot_state is None:
    robot_state = (
      state.perceived_robot_planar_state()
      if noisy
      else state.robot_planar_state()
    )
  rel_pos_w, rel_vel_w, radius, valid, _ = state.select_obstacles(
    noisy=noisy, robot_state=robot_state
  )
  robot_pos, yaw, _, robot_vel_w, _ = robot_state
  rel_pos = _world_to_body(rel_pos_w, yaw)
  rel_vel = _world_to_body(rel_vel_w, yaw)
  batch, constraints = valid.shape
  nodes = torch.zeros(batch, constraints + 2, 9, device=env.device)

  nodes[:, 0, 0] = 1.0
  nodes[:, 0, 5] = cfg.robot.r_rob
  nodes[:, 0, 8] = 1.0
  nodes[:, 1 : constraints + 1, 1] = 1.0
  nodes[:, 1 : constraints + 1, 3:5] = rel_pos
  nodes[:, 1 : constraints + 1, 5] = radius
  nodes[:, 1 : constraints + 1, 6:8] = rel_vel
  nodes[:, 1 : constraints + 1, 8] = valid.float()

  goal_index = constraints + 1
  goal_rel_w = state.goal - robot_pos
  goal_rel = _world_to_body(goal_rel_w[:, None, :], yaw)[:, 0]
  goal_vel = _world_to_body(-robot_vel_w[:, None, :], yaw)[:, 0]
  nodes[:, goal_index, 2] = 1.0
  nodes[:, goal_index, 3:5] = goal_rel
  nodes[:, goal_index, 5] = cfg.goal.radius
  nodes[:, goal_index, 6:8] = goal_vel
  nodes[:, goal_index, 8] = 1.0
  return nodes.flatten(start_dim=1)


def local_robot_state(
  env,
  cfg: NavigationTaskCfg,
  robot_state: tuple[torch.Tensor, ...] | None = None,
) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  if robot_state is None:
    robot_state = state.robot_planar_state()
  pos, yaw, vel_b, _, yaw_rate = robot_state
  action_term = env.action_manager.get_term("navigation")
  center = torch.tensor(cfg.arena.center, device=env.device)
  half = torch.tensor(cfg.arena.size, device=env.device) * 0.5
  normalized_pos = ((pos - center) / half).clamp(-1.5, 1.5)
  heading_error = state.goal_heading - yaw
  return torch.cat(
    (
      normalized_pos,
      torch.sin(yaw)[:, None],
      torch.cos(yaw)[:, None],
      vel_b,
      yaw_rate[:, None],
      action_term.velocity_command,
      action_term.raw_action,
      torch.sin(heading_error)[:, None],
      torch.cos(heading_error)[:, None],
    ),
    dim=-1,
  )


def actor_observation(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  state = get_navigation_state(env, cfg)
  robot_state = state.perceived_robot_planar_state()
  graph = graph_observation(
    env, cfg, noisy=True, robot_state=robot_state
  )
  if cfg.actor_state_mode == "gat_only":
    return graph
  return torch.cat(
    (graph, local_robot_state(env, cfg, robot_state=robot_state)), dim=-1
  )


def critic_observation(env, cfg: NavigationTaskCfg) -> torch.Tensor:
  # The asymmetric critic sees ground-truth graph values.
  state = get_navigation_state(env, cfg)
  robot_state = state.robot_planar_state()
  return torch.cat(
    (
      graph_observation(env, cfg, noisy=False, robot_state=robot_state),
      local_robot_state(env, cfg, robot_state=robot_state),
    ),
    dim=-1,
  )
