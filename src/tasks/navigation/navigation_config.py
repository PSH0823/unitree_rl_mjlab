"""Single source of truth for the G1 DPCBF-navigation task."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass
class ObstacleCfg:
  count: int = 20
  max_count: int = 20
  radius_range: tuple[float, float] = (0.2, 0.3)
  speed_range: tuple[float, float] = (0.0, 0.8)
  height: float = 1.5
  rgba: tuple[float, float, float, float] = (0.1, 0.4, 1.0, 0.65)
  collision_enabled: bool = True
  # 0: distance, 1: relative-velocity closing alignment (distance tie-break).
  priority: int = 1
  num_constraints: int = 10


@dataclass
class ArenaCfg:
  size: tuple[float, float] = (10.0, 10.0)
  center: tuple[float, float] = (0.0, 0.0)
  boundary_margin: float = 0.15


@dataclass
class RobotCfg:
  base_body: str = "pelvis"
  r_rob: float = 0.30
  p_max: float = 3.0
  v_s_range: tuple[float, float] = (-1.0, 2.0)
  v_l_range: tuple[float, float] = (-1.0, 1.0)
  w_range: tuple[float, float] = (-1.0, 1.0)
  a_s_range: tuple[float, float] = (-2.0, 4.0)
  a_l_range: tuple[float, float] = (-2.0, 2.0)
  safety_factor: float = 1.05
  eps_v: float = 0.05
  eps_d: float = 0.05


@dataclass
class DpcbfCfg:
  # Pure DPCBF only: eCBF, optimal-decay and slack are intentionally absent.
  k_mu: float = 1.0
  k_lambda: float = 0.5
  alpha: float = 2.0
  reward_sigma: float = 0.5
  filter_actions: bool = True


@dataclass
class GoalCfg:
  radius: float = 0.30
  min_distance: float = 2.0
  max_distance: float = 6.0
  resample_on_success_in_play: bool = True


@dataclass
class PerceptionRandomizationCfg:
  enabled: bool = True
  position_std: float = 0.03
  velocity_std: float = 0.05
  radius_std: float = 0.015
  per_episode_position_bias_std: float = 0.02
  dropout_probability: float = 0.03
  latency_steps: int = 1


@dataclass
class ActionRandomizationCfg:
  enabled: bool = True
  acceleration_scale_range: tuple[float, float] = (0.90, 1.10)
  acceleration_noise_std: float = 0.10
  yaw_rate_noise_std: float = 0.03
  latency_steps: int = 1


@dataclass
class CurriculumCfg:
  enabled: bool = True
  success_ema_rate: float = 0.02
  promote_threshold: float = 0.80
  demote_threshold: float = 0.55
  stages: tuple[dict[str, float], ...] = (
    {"obstacle_fraction": 0.3, "speed_fraction": 0.0, "noise_fraction": 0.0},
    {"obstacle_fraction": 0.5, "speed_fraction": 0.35, "noise_fraction": 0.25},
    {"obstacle_fraction": 0.75, "speed_fraction": 0.65, "noise_fraction": 0.60},
    {"obstacle_fraction": 1.0, "speed_fraction": 1.0, "noise_fraction": 1.0},
  )


@dataclass
class RewardCfg:
  progress_weight: float = 60.0
  goal_weight: float = 1.0
  collision_weight: float = -1.0
  wall_collision_weight: float = -1.0
  cbf_weight: float = 100.0
  action_rate_weight: float = -0.02
  time_weight: float = -0.01
  timeout_weight: float = -10.0


@dataclass
class NavigationTaskCfg:
  obstacle: ObstacleCfg = field(default_factory=ObstacleCfg)
  arena: ArenaCfg = field(default_factory=ArenaCfg)
  robot: RobotCfg = field(default_factory=RobotCfg)
  dpcbf: DpcbfCfg = field(default_factory=DpcbfCfg)
  goal: GoalCfg = field(default_factory=GoalCfg)
  perception: PerceptionRandomizationCfg = field(
    default_factory=PerceptionRandomizationCfg
  )
  action_randomization: ActionRandomizationCfg = field(
    default_factory=ActionRandomizationCfg
  )
  curriculum: CurriculumCfg = field(default_factory=CurriculumCfg)
  reward: RewardCfg = field(default_factory=RewardCfg)

  physics_dt: float = 0.005
  high_level_hz: float = 10.0
  low_level_hz: float = 50.0
  episode_length_s: float = 30.0
  actor_state_mode: str = "gat_only"  # "local" or "gat_only"
  num_envs: int = 1024
  low_level_policy: str = str(
    _REPO_ROOT / "deploy/robots/g1/config/policy/velocity/v1/exported/policy.onnx"
  )

  def validate(self) -> None:
    if self.obstacle.priority not in (0, 1):
      raise ValueError("obstacle.priority must be 0 (distance) or 1 (closing alignment)")
    if self.obstacle.count > self.obstacle.max_count:
      raise ValueError("obstacle.count cannot exceed obstacle.max_count")
    if self.obstacle.num_constraints > self.obstacle.max_count:
      raise ValueError("num_constraints cannot exceed max_count")
    if self.actor_state_mode not in ("local", "gat_only"):
      raise ValueError("actor_state_mode must be 'local' or 'gat_only'")
    if self.dpcbf.reward_sigma <= 0.0:
      raise ValueError("dpcbf.reward_sigma must be positive")
    for name, rate in (
      ("high_level_hz", self.high_level_hz),
      ("low_level_hz", self.low_level_hz),
    ):
      steps = 1.0 / (self.physics_dt * rate)
      if abs(steps - round(steps)) > 1.0e-6:
        raise ValueError(f"{name} must be an integer divisor of physics rate")


NAVIGATION_CFG = NavigationTaskCfg()
