"""Hierarchical navigation action: acceleration -> velocity -> frozen G1 policy."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import onnx
import torch
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.action_manager import ActionTerm, ActionTermCfg

from src.assets.robots import G1_ACTION_SCALE

from ..navigation_config import NavigationTaskCfg
from .state import get_navigation_state


class FrozenOnnxMlp(torch.nn.Module):
  """Loads the exported 98->29 velocity policy without requiring onnxruntime."""

  def __init__(self, path: str, device: str) -> None:
    super().__init__()
    policy_path = Path(path)
    if not policy_path.is_file():
      raise FileNotFoundError(f"Low-level policy not found: {policy_path}")
    model = onnx.load(str(policy_path))
    values = {
      item.name: torch.as_tensor(
        np.asarray(onnx.numpy_helper.to_array(item)).copy(),
        dtype=torch.float32,
        device=device,
      )
      for item in model.graph.initializer
    }
    self.register_buffer("mean", values["obs_normalizer._mean"])
    self.register_buffer("std", values["onnx::Div_24"])
    layers = []
    for index in (0, 2, 4, 6):
      weight = values[f"mlp.{index}.weight"]
      bias = values[f"mlp.{index}.bias"]
      layer = torch.nn.Linear(weight.shape[1], weight.shape[0], device=device)
      layer.weight.data.copy_(weight)
      layer.bias.data.copy_(bias)
      layer.requires_grad_(False)
      layers.append(layer)
      if index != 6:
        layers.append(torch.nn.ELU())
    self.mlp = torch.nn.Sequential(*layers)
    self.eval()

  @torch.inference_mode()
  def forward(self, observation: torch.Tensor) -> torch.Tensor:
    return self.mlp((observation - self.mean) / self.std)


@dataclass(kw_only=True)
class NavigationActionCfg(ActionTermCfg):
  navigation_cfg: NavigationTaskCfg

  def build(self, env) -> "NavigationAction":
    return NavigationAction(self, env)


class NavigationAction(ActionTerm):
  cfg: NavigationActionCfg

  def __init__(self, cfg: NavigationActionCfg, env) -> None:
    super().__init__(cfg, env)
    self.nav_cfg = cfg.navigation_cfg
    self.state = get_navigation_state(env, self.nav_cfg)
    self._raw = torch.zeros(env.num_envs, 3, device=env.device)
    self._policy_physical = torch.zeros_like(self._raw)
    self._safe_physical = torch.zeros_like(self._raw)
    self._physical = torch.zeros_like(self._raw)
    self._dpcbf_values = None
    self.velocity_command = torch.zeros_like(self._raw)
    self._last_low_action = torch.zeros(env.num_envs, 29, device=env.device)
    self._phase_time = torch.zeros(env.num_envs, device=env.device)
    self._physics_counter = 0
    action_rand = self.nav_cfg.action_randomization
    self._action_latency = max(0, action_rand.latency_steps)
    self._action_history = torch.zeros(
      env.num_envs, self._action_latency + 1, 3, device=env.device
    )
    self._acceleration_scale = torch.ones(env.num_envs, 2, device=env.device)
    self._low_decimation = round(
      1.0 / (self.nav_cfg.physics_dt * self.nav_cfg.low_level_hz)
    )
    self._low_dt = 1.0 / self.nav_cfg.low_level_hz
    self._high_dt = 1.0 / self.nav_cfg.high_level_hz
    self._low_policy = FrozenOnnxMlp(
      self.nav_cfg.low_level_policy, str(env.device)
    )
    self._joint_action = JointPositionActionCfg(
      entity_name=cfg.entity_name,
      actuator_names=(".*",),
      scale=G1_ACTION_SCALE,
      use_default_offset=True,
    ).build(env)

  @property
  def action_dim(self) -> int:
    return 3

  @property
  def raw_action(self) -> torch.Tensor:
    return self._raw

  @property
  def physical_action(self) -> torch.Tensor:
    return self._physical

  @property
  def policy_action(self) -> torch.Tensor:
    return self._policy_physical

  @property
  def safe_action(self) -> torch.Tensor:
    return self._safe_physical

  @property
  def dpcbf_values(self):
    return self._dpcbf_values

  def process_actions(self, actions: torch.Tensor) -> None:
    self._raw[:] = actions.clamp(-1.0, 1.0)
    self._action_history = torch.roll(self._action_history, shifts=-1, dims=1)
    self._action_history[:, -1] = self._raw
    difficulty = self.state.difficulty()["noise_fraction"]
    effective_latency = round(self._action_latency * difficulty)
    delayed_action = self._action_history[:, self._action_latency - effective_latency]
    ranges = (
      self.nav_cfg.robot.a_s_range,
      self.nav_cfg.robot.a_l_range,
      self.nav_cfg.robot.w_range,
    )
    for i, (low, high) in enumerate(ranges):
      self._policy_physical[:, i] = (
        0.5 * (high + low) + 0.5 * (high - low) * delayed_action[:, i]
      )
    randomization = self.nav_cfg.action_randomization
    if randomization.enabled:
      scale = 1.0 + (self._acceleration_scale - 1.0) * difficulty
      self._policy_physical[:, :2] *= scale
      self._policy_physical[:, :2] += (
        torch.randn_like(self._policy_physical[:, :2])
        * randomization.acceleration_noise_std
        * difficulty
      )
      self._policy_physical[:, 2] += (
        torch.randn_like(self._policy_physical[:, 2])
        * randomization.yaw_rate_noise_std
        * difficulty
      )
      self._policy_physical[:, 0].clamp_(*self.nav_cfg.robot.a_s_range)
      self._policy_physical[:, 1].clamp_(*self.nav_cfg.robot.a_l_range)
      self._policy_physical[:, 2].clamp_(*self.nav_cfg.robot.w_range)
    self._safe_physical[:], self._dpcbf_values = self.state.filter_action(
      self._policy_physical
    )
    if self.nav_cfg.dpcbf.filter_actions:
      self._physical[:] = self._safe_physical
    else:
      self._physical[:] = self._policy_physical
    # Acceleration is in SI m/s^2. Integrate once at the 10 Hz high-level step.
    self.velocity_command[:, :2] += self._physical[:, :2] * self._high_dt
    self.velocity_command[:, 0].clamp_(*self.nav_cfg.robot.v_s_range)
    self.velocity_command[:, 1].clamp_(*self.nav_cfg.robot.v_l_range)
    self.velocity_command[:, 2] = self._physical[:, 2]

  def _low_level_observation(self) -> torch.Tensor:
    data = self._entity.data
    command = self.velocity_command
    phase = torch.stack(
      (
        torch.sin(2.0 * torch.pi * self._phase_time / 0.6),
        torch.cos(2.0 * torch.pi * self._phase_time / 0.6),
      ),
      dim=-1,
    )
    phase = torch.where(
      torch.linalg.vector_norm(command, dim=-1, keepdim=True) < 0.1,
      torch.zeros_like(phase),
      phase,
    )
    return torch.cat(
      (
        data.root_link_ang_vel_b,
        data.projected_gravity_b,
        command,
        phase,
        data.joint_pos - data.default_joint_pos,
        data.joint_vel - data.default_joint_vel,
        self._last_low_action,
      ),
      dim=-1,
    )

  def apply_actions(self) -> None:
    self.state.step_obstacles(self.nav_cfg.physics_dt)
    if self._physics_counter % self._low_decimation == 0:
      low_obs = self._low_level_observation()
      if low_obs.shape[-1] != 98:
        raise RuntimeError(
          f"Expected 98 low-level observations, got {low_obs.shape[-1]}"
        )
      self._last_low_action[:] = self._low_policy(low_obs)
      self._joint_action.process_actions(self._last_low_action)
      self._phase_time += self._low_dt
    self._joint_action.apply_actions()
    self._physics_counter += 1

  def reset(self, env_ids=None) -> None:
    if env_ids is None:
      env_ids = slice(None)
    self._raw[env_ids] = 0.0
    self._policy_physical[env_ids] = 0.0
    self._safe_physical[env_ids] = 0.0
    self._physical[env_ids] = 0.0
    self.velocity_command[env_ids] = 0.0
    self._last_low_action[env_ids] = 0.0
    self._phase_time[env_ids] = 0.0
    self._action_history[env_ids] = 0.0
    low, high = self.nav_cfg.action_randomization.acceleration_scale_range
    shape = self._acceleration_scale[env_ids].shape
    self._acceleration_scale[env_ids] = low + (high - low) * torch.rand(
      shape, device=self.device
    )
    self._joint_action.reset(env_ids)
