"""Vectorized navigation state, obstacle perception and DPCBF math."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from mjlab.envs.mdp.dr.geom import _recompute_geom_bounds
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.lab_api.math import euler_xyz_from_quat

from ..navigation_config import NavigationTaskCfg


_STATE_ATTR = "_dpcbf_navigation_state"


@dataclass
class DpcbfValues:
  h: torch.Tensor
  condition: torch.Tensor
  lg_h: torch.Tensor
  valid: torch.Tensor


class NavigationState:
  def __init__(self, env, cfg: NavigationTaskCfg):
    cfg.validate()
    self.env = env
    self.cfg = cfg
    self.device = env.device
    b, n = env.num_envs, cfg.obstacle.max_count
    self.position = torch.zeros(b, n, 2, device=self.device)
    self.velocity = torch.zeros_like(self.position)
    self.radius = torch.full(
      (b, n), cfg.obstacle.radius_range[0], device=self.device
    )
    self.active = torch.zeros(b, n, dtype=torch.bool, device=self.device)
    self.goal = torch.zeros(b, 2, device=self.device)
    self.previous_goal_distance = torch.zeros(b, device=self.device)
    self.position_bias = torch.zeros_like(self.position)
    self.success_ema = torch.tensor(0.0, device=self.device)
    self.curriculum_stage = 0
    self._latency = max(0, cfg.perception.latency_steps)
    self._perception_history: list[tuple[torch.Tensor, ...]] = []
    self._last_selected = None

  @property
  def robot(self):
    return self.env.scene["robot"]

  def robot_planar_state(self) -> tuple[torch.Tensor, ...]:
    data = self.robot.data
    # Entity poses are world-frame values, whereas navigation state is stored
    # in each replicated environment's local arena frame.
    pos = data.root_link_pos_w[:, :2] - self.env.scene.env_origins[:, :2]
    _, _, yaw = euler_xyz_from_quat(data.root_link_quat_w)
    vel_b = data.root_link_lin_vel_b[:, :2]
    vel_w = data.root_link_lin_vel_w[:, :2]
    yaw_rate = data.root_link_ang_vel_b[:, 2]
    return pos, yaw, vel_b, vel_w, yaw_rate

  def difficulty(self) -> dict[str, float]:
    if not self.cfg.curriculum.enabled:
      return {
        "obstacle_fraction": 1.0,
        "speed_fraction": 1.0,
        "noise_fraction": 1.0,
      }
    return self.cfg.curriculum.stages[self.curriculum_stage]

  def reset(self, env_ids: torch.Tensor) -> None:
    count = env_ids.numel()
    if count == 0:
      return
    cfg, device = self.cfg, self.device
    difficulty = self.difficulty()
    max_active = min(cfg.obstacle.count, cfg.obstacle.max_count)
    active_count = max(1, round(max_active * difficulty["obstacle_fraction"]))
    slots = torch.arange(cfg.obstacle.max_count, device=device)
    self.active[env_ids] = slots.unsqueeze(0) < active_count

    r0, r1 = cfg.obstacle.radius_range
    self.radius[env_ids] = r0 + (r1 - r0) * torch.rand(
      count, cfg.obstacle.max_count, device=device
    )
    v0, v1 = cfg.obstacle.speed_range
    speed_max = v0 + (v1 - v0) * difficulty["speed_fraction"]
    speed = v0 + (speed_max - v0) * torch.rand(
      count, cfg.obstacle.max_count, device=device
    )
    angle = 2.0 * torch.pi * torch.rand_like(speed)
    self.velocity[env_ids] = torch.stack(
      (speed * torch.cos(angle), speed * torch.sin(angle)), dim=-1
    )
    self.velocity[env_ids] *= self.active[env_ids].unsqueeze(-1)

    # ``reset_root_state_uniform`` writes the configured reset pose immediately
    # before this event, but derived link poses are intentionally not forwarded
    # until all reset events finish.  The configured planar reset offset is
    # exactly zero, so use that fresh local pose rather than stale link data.
    robot_pos = torch.zeros(count, 2, device=device)
    half = torch.tensor(cfg.arena.size, device=device) * 0.5
    center = torch.tensor(cfg.arena.center, device=device)
    lower, upper = center - half, center + half
    sampled = lower + (upper - lower) * torch.rand(
      count, cfg.obstacle.max_count, 2, device=device
    )
    # Vectorized rejection keeps obstacles clear of the initial robot footprint.
    clearance = cfg.robot.r_rob + self.radius[env_ids] + 0.5
    for _ in range(32):
      bad = torch.linalg.vector_norm(sampled - robot_pos[:, None, :], dim=-1) < clearance
      if not bad.any():
        break
      replacement = lower + (upper - lower) * torch.rand_like(sampled)
      sampled = torch.where(bad.unsqueeze(-1), replacement, sampled)
    self.position[env_ids] = sampled

    self.sample_goal(env_ids, robot_pos=robot_pos)
    self.previous_goal_distance[env_ids] = torch.linalg.vector_norm(
      self.goal[env_ids] - robot_pos, dim=-1
    )
    bias_std = cfg.perception.per_episode_position_bias_std
    self.position_bias[env_ids] = torch.randn_like(sampled) * bias_std
    self._write_obstacles(env_ids, update_size=True)
    self._perception_history.clear()

  def sample_goal(
    self, env_ids: torch.Tensor, robot_pos: torch.Tensor | None = None
  ) -> None:
    if robot_pos is None:
      current_pos, _, _, _, _ = self.robot_planar_state()
      robot_pos = current_pos[env_ids]
    cfg = self.cfg
    half = torch.tensor(cfg.arena.size, device=self.device) * 0.5
    center = torch.tensor(cfg.arena.center, device=self.device)
    margin = cfg.goal.radius + cfg.arena.boundary_margin
    proposed = robot_pos.clone()
    pending = torch.ones(env_ids.numel(), dtype=torch.bool, device=self.device)
    for _ in range(24):
      angle = 2.0 * torch.pi * torch.rand(env_ids.numel(), device=self.device)
      distance = cfg.goal.min_distance + (
        cfg.goal.max_distance - cfg.goal.min_distance
      ) * torch.rand(env_ids.numel(), device=self.device)
      candidate = robot_pos + distance[:, None] * torch.stack(
        (torch.cos(angle), torch.sin(angle)), dim=-1
      )
      candidate = torch.maximum(
        torch.minimum(candidate, center + half - margin), center - half + margin
      )
      clearance = (
        self.radius[env_ids] + cfg.goal.radius + 0.2
      )
      overlap = (
        self.active[env_ids]
        & (
          torch.linalg.vector_norm(
            self.position[env_ids] - candidate[:, None, :], dim=-1
          )
          < clearance
        )
      ).any(dim=-1)
      accepted = pending & ~overlap
      proposed = torch.where(accepted[:, None], candidate, proposed)
      pending &= overlap
      if not pending.any():
        break
    if pending.any():
      proposed[pending] = candidate[pending]
    self.goal[env_ids] = proposed
    pose = torch.zeros(env_ids.numel(), 7, device=self.device)
    pose[:, :2] = (
      self.goal[env_ids] + self.env.scene.env_origins[env_ids, :2]
    )
    pose[:, 2] = 0.03
    pose[:, 3] = 1.0
    self.env.scene["goal_marker"].write_mocap_pose_to_sim(pose, env_ids)

  def step_obstacles(self, dt: float) -> None:
    self.position += self.velocity * dt
    half = torch.tensor(self.cfg.arena.size, device=self.device) * 0.5
    center = torch.tensor(self.cfg.arena.center, device=self.device)
    lower = center - half + self.radius.unsqueeze(-1)
    upper = center + half - self.radius.unsqueeze(-1)
    hit_low = self.position < lower
    hit_high = self.position > upper
    hit = hit_low | hit_high
    self.position = torch.maximum(torch.minimum(self.position, upper), lower)
    self.velocity = torch.where(hit, -self.velocity, self.velocity)
    self._write_obstacles(None, update_size=False)

  def _write_obstacles(
    self, env_ids: torch.Tensor | None, update_size: bool
  ) -> None:
    ids = (
      torch.arange(self.env.num_envs, device=self.device)
      if env_ids is None
      else env_ids
    )
    for index in range(self.cfg.obstacle.max_count):
      entity = self.env.scene[f"obstacle_{index:02d}"]
      pose = torch.zeros(ids.numel(), 7, device=self.device)
      pose[:, :2] = (
        self.position[ids, index] + self.env.scene.env_origins[ids, :2]
      )
      pose[:, 2] = self.cfg.obstacle.height * 0.5
      pose[:, 3] = 1.0
      # Inactive cylinders are parked far below the floor.
      pose[:, 2] = torch.where(
        self.active[ids, index], pose[:, 2], torch.full_like(pose[:, 2], -10.0)
      )
      entity.write_mocap_pose_to_sim(pose, ids)
      if update_size:
        geom_id = entity.indexing.geom_ids[0]
        self.env.sim.model.geom_size[ids, geom_id, 0] = self.radius[ids, index]
        _recompute_geom_bounds(
          self.env, ids, SceneEntityCfg(f"obstacle_{index:02d}")
        )

  def perceived(self) -> tuple[torch.Tensor, ...]:
    cfg = self.cfg.perception
    difficulty = self.difficulty()["noise_fraction"]
    pos, vel, radius, active = (
      self.position,
      self.velocity,
      self.radius,
      self.active,
    )
    if cfg.enabled:
      pos = pos + self.position_bias * difficulty
      pos = pos + torch.randn_like(pos) * cfg.position_std * difficulty
      vel = vel + torch.randn_like(vel) * cfg.velocity_std * difficulty
      radius = (
        radius + torch.randn_like(radius) * cfg.radius_std * difficulty
      ).clamp_min(0.02)
      drop = torch.rand_like(radius) < cfg.dropout_probability * difficulty
      active = active & ~drop
    snapshot = (pos, vel, radius, active)
    self._perception_history.append(tuple(x.clone() for x in snapshot))
    keep = self._latency + 1
    if len(self._perception_history) > keep:
      self._perception_history.pop(0)
    effective_latency = round(self._latency * difficulty)
    effective_latency = min(effective_latency, len(self._perception_history) - 1)
    return self._perception_history[-1 - effective_latency]

  def select_obstacles(
    self, noisy: bool = True
  ) -> tuple[torch.Tensor, ...]:
    obs_pos, obs_vel, radius, active = (
      self.perceived()
      if noisy
      else (self.position, self.velocity, self.radius, self.active)
    )
    robot_pos, _, _, robot_vel_w, _ = self.robot_planar_state()
    rel_pos = obs_pos - robot_pos[:, None, :]
    rel_vel = obs_vel - robot_vel_w[:, None, :]
    distance = torch.linalg.vector_norm(rel_pos, dim=-1)
    valid = active & (distance <= self.cfg.robot.p_max)
    if self.cfg.obstacle.priority == 0:
      score = distance
    else:
      closing = -(rel_vel * rel_pos).sum(-1) / (
        torch.linalg.vector_norm(rel_vel, dim=-1) * distance
      ).clamp_min(1.0e-6)
      # Descending alignment, then nearest distance.
      score = -closing + distance * 1.0e-4
    score = torch.where(valid, score, torch.full_like(score, 1.0e9))
    k = self.cfg.obstacle.num_constraints
    indices = torch.topk(score, k=k, dim=-1, largest=False).indices
    gather2 = indices.unsqueeze(-1).expand(-1, -1, 2)
    result = (
      torch.gather(rel_pos, 1, gather2),
      torch.gather(rel_vel, 1, gather2),
      torch.gather(radius, 1, indices),
      torch.gather(valid, 1, indices),
      indices,
    )
    self._last_selected = result
    return result

  def dpcbf(self, action: torch.Tensor) -> DpcbfValues:
    rel_pos, rel_vel, obs_radius, valid, _ = self.select_obstacles(noisy=False)
    _, phi, vel_b, robot_vel_w, _ = self.robot_planar_state()
    px, py = rel_pos.unbind(-1)
    p2 = (rel_pos * rel_pos).sum(-1).clamp_min(1.0e-8)
    alpha_los = torch.atan2(py, px)
    ca, sa = torch.cos(alpha_los), torch.sin(alpha_los)
    x_tilde = ca * rel_vel[..., 0] + sa * rel_vel[..., 1]
    y_tilde = -sa * rel_vel[..., 0] + ca * rel_vel[..., 1]
    safe_velocity = torch.sqrt((rel_vel * rel_vel).sum(-1) + self.cfg.robot.eps_v**2)
    safe_radius = (
      self.cfg.robot.r_rob + obs_radius
    ) * self.cfg.robot.safety_factor
    smooth_clearance = torch.sqrt(
      (p2 - safe_radius.square() + self.cfg.robot.eps_d**2).clamp_min(1.0e-8)
    )
    safe_distance = smooth_clearance - self.cfg.robot.eps_d
    adaptive = (
      (self.cfg.robot.safety_factor**2 - 1.0) ** 0.5 / safe_radius
    )
    lam = self.cfg.dpcbf.k_lambda * safe_distance / safe_velocity
    h = x_tilde + adaptive * (
      lam * y_tilde.square() + self.cfg.dpcbf.k_mu * safe_distance
    )

    position_term = (
      self.cfg.dpcbf.k_lambda * y_tilde.square() / safe_velocity
      + self.cfg.dpcbf.k_mu
    )
    dh_dx = py * y_tilde / p2 - adaptive * (
      2.0 * lam * x_tilde * y_tilde * py / p2
      + px * position_term / smooth_clearance
    )
    dh_dy = -px * y_tilde / p2 + adaptive * (
      2.0 * lam * x_tilde * y_tilde * px / p2
      - py * position_term / smooth_clearance
    )
    phi_tilde = phi[:, None] - alpha_los
    cosine, sine = torch.cos(phi_tilde), torch.sin(phi_tilde)
    vs, vl = vel_b[:, 0:1], vel_b[:, 1:2]
    v1 = vs * sine + vl * cosine
    v2 = vs * cosine - vl * sine
    velocity_cubed = safe_velocity.pow(3)
    common_x = (
      self.cfg.dpcbf.k_lambda
      * safe_distance
      * x_tilde
      * y_tilde.square()
      / velocity_cubed
    )
    common_y = self.cfg.dpcbf.k_lambda * safe_distance * (
      2.0 * y_tilde / safe_velocity - y_tilde.pow(3) / velocity_cubed
    )
    dh_dphi = v1 - adaptive * (v1 * common_x + v2 * common_y)
    dh_dvs = -cosine + adaptive * (cosine * common_x - sine * common_y)
    dh_dvl = sine - adaptive * (sine * common_x + cosine * common_y)
    # p_rel_dot = v_obs - v_robot, matching the deployed C++ implementation.
    lf_h = -dh_dx * rel_vel[..., 0] - dh_dy * rel_vel[..., 1]
    lg_u = (
      dh_dvs * action[:, None, 0]
      + dh_dvl * action[:, None, 1]
      + dh_dphi * action[:, None, 2]
    )
    condition = lf_h + lg_u + self.cfg.dpcbf.alpha * h
    lg_h = torch.stack((dh_dvs, dh_dvl, dh_dphi), dim=-1)
    return DpcbfValues(h=h, condition=condition, lg_h=lg_h, valid=valid)

  def filter_action(
    self, policy_action: torch.Tensor
  ) -> tuple[torch.Tensor, DpcbfValues]:
    """Project onto the most critical valid DPCBF half-space.

    This is the closed-form CBF-QP projection used by the reference
    environment.  For the multi-obstacle DPCBF, the active row is the valid
    row with the smallest ``Lf h + Lg h u + alpha h`` value.
    """
    values = self.dpcbf(policy_action)
    masked_condition = torch.where(
      values.valid,
      values.condition,
      torch.full_like(values.condition, torch.inf),
    )
    active_index = masked_condition.argmin(dim=-1)
    gather_index = active_index[:, None, None].expand(-1, 1, 3)
    active_lg = torch.gather(values.lg_h, 1, gather_index).squeeze(1)
    active_condition = torch.gather(
      masked_condition, 1, active_index[:, None]
    ).squeeze(1)
    violation = values.valid.any(dim=-1) & (active_condition < 0.0)
    denominator = active_lg.square().sum(dim=-1).clamp_min(1.0e-12)
    correction = (
      -active_condition / denominator
    )[:, None] * active_lg
    safe_action = torch.where(
      violation[:, None], policy_action + correction, policy_action
    )

    # The humanoid can only execute controls inside these bounds.  Unlike the
    # point-integrator example, an unbounded projection is not a valid command.
    limits = (
      self.cfg.robot.a_s_range,
      self.cfg.robot.a_l_range,
      self.cfg.robot.w_range,
    )
    for index, (lower, upper) in enumerate(limits):
      safe_action[:, index].clamp_(lower, upper)
    return safe_action, values


def get_navigation_state(env, cfg: NavigationTaskCfg) -> NavigationState:
  state = getattr(env, _STATE_ATTR, None)
  if state is None:
    state = NavigationState(env, cfg)
    setattr(env, _STATE_ATTR, state)
  return state


@requires_model_fields("geom_size", "geom_rbound", "geom_aabb")
def reset_navigation(env, env_ids, cfg: NavigationTaskCfg) -> None:
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device)
  get_navigation_state(env, cfg).reset(env_ids.to(dtype=torch.int))


def resample_play_goals(
  env, env_ids, cfg: NavigationTaskCfg, dt: float | None = None
) -> None:
  del dt
  del env_ids
  state = get_navigation_state(env, cfg)
  if not cfg.goal.resample_on_success_in_play:
    return
  robot_pos, _, _, _, _ = state.robot_planar_state()
  reached = (
    torch.linalg.vector_norm(state.goal - robot_pos, dim=-1)
    <= cfg.robot.r_rob + cfg.goal.radius
  )
  ids = torch.nonzero(reached, as_tuple=False).squeeze(-1)
  if ids.numel():
    state.sample_goal(ids)
    state.previous_goal_distance[ids] = torch.linalg.vector_norm(
      state.goal[ids] - robot_pos[ids], dim=-1
    )
