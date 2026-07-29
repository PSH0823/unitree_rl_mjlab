"""Manager-based G1 DPCBF-navigation environment."""

from __future__ import annotations

import functools
import copy

import mujoco
from mjlab.entity import EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.viewer import ViewerConfig

from src.assets.robots import get_g1_robot_cfg

from . import mdp
from .mdp.actions import NavigationActionCfg
from .navigation_config import NAVIGATION_CFG, NavigationTaskCfg


def _obstacle_spec(
  radius: float,
  height: float,
  rgba: tuple[float, float, float, float],
  collision: bool,
) -> mujoco.MjSpec:
  spec = mujoco.MjSpec()
  body = spec.worldbody.add_body(name="obstacle")
  body.add_geom(
    name="cylinder",
    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
    size=(radius, height * 0.5, 0.0),
    rgba=rgba,
    # contype=2 avoids obstacle-obstacle contacts; conaffinity=1 still collides
    # with the robot's default contype=1 geoms.
    contype=2 if collision else 0,
    conaffinity=1 if collision else 0,
    friction=(0.8, 0.01, 0.001),
  )
  return spec


def _goal_spec(radius: float) -> mujoco.MjSpec:
  spec = mujoco.MjSpec()
  body = spec.worldbody.add_body(name="goal")
  body.add_geom(
    name="goal_disk",
    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
    size=(radius, 0.025, 0.0),
    rgba=(0.15, 0.95, 0.25, 0.65),
    contype=0,
    conaffinity=0,
  )
  return spec


def _arena_boundary_spec(cfg: NavigationTaskCfg) -> mujoco.MjSpec:
  spec = mujoco.MjSpec()
  body = spec.worldbody.add_body(name="arena_boundary")
  sx, sy = cfg.arena.size
  cx, cy = cfg.arena.center
  thickness, height = 0.035, 0.025
  rgba = (0.2, 0.7, 1.0, 0.28)
  for name, pos, size in (
    ("north", (cx, cy + sy / 2, height), (sx / 2, thickness, height)),
    ("south", (cx, cy - sy / 2, height), (sx / 2, thickness, height)),
    ("east", (cx + sx / 2, cy, height), (thickness, sy / 2, height)),
    ("west", (cx - sx / 2, cy, height), (thickness, sy / 2, height)),
  ):
    body.add_geom(
      name=name,
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=pos,
      size=size,
      rgba=rgba,
      contype=0,
      conaffinity=0,
    )
  return spec


def make_g1_navigation_env_cfg(
  play: bool = False, task_cfg: NavigationTaskCfg = NAVIGATION_CFG
) -> ManagerBasedRlEnvCfg:
  task_cfg = copy.deepcopy(task_cfg)
  if play:
    task_cfg.curriculum.enabled = False
    task_cfg.perception.enabled = False
    task_cfg.action_randomization.enabled = False
    # CBF-RL trains with filtered rollouts but deploys the learned policy
    # without a runtime filter.
    task_cfg.dpcbf.filter_actions = False
  task_cfg.validate()
  entities = {
    "robot": get_g1_robot_cfg(),
    "goal_marker": EntityCfg(
      spec_fn=functools.partial(_goal_spec, task_cfg.goal.radius)
    ),
    "arena_boundary": EntityCfg(
      spec_fn=functools.partial(_arena_boundary_spec, task_cfg)
    ),
  }
  for index in range(task_cfg.obstacle.max_count):
    entities[f"obstacle_{index:02d}"] = EntityCfg(
      init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, -10.0 - 2.0 * index)),
      spec_fn=functools.partial(
        _obstacle_spec,
        task_cfg.obstacle.radius_range[0],
        task_cfg.obstacle.height,
        task_cfg.obstacle.rgba,
        task_cfg.obstacle.collision_enabled,
      )
    )

  observations = {
    "actor": ObservationGroupCfg(
      terms={
        "navigation": ObservationTermCfg(
          func=mdp.actor_observation, params={"cfg": task_cfg}
        )
      },
      concatenate_terms=True,
      enable_corruption=False,
      history_length=1,
    ),
    "critic": ObservationGroupCfg(
      terms={
        "navigation": ObservationTermCfg(
          func=mdp.critic_observation, params={"cfg": task_cfg}
        )
      },
      concatenate_terms=True,
      enable_corruption=False,
      history_length=1,
    ),
  }

  actions = {
    "navigation": NavigationActionCfg(
      entity_name="robot", navigation_cfg=task_cfg
    )
  }
  events = {
    "reset_base": EventTermCfg(
      func=envs_mdp.reset_root_state_uniform,
      mode="reset",
      params={
        "pose_range": {
          "x": (0.0, 0.0),
          "y": (0.0, 0.0),
          "yaw": (-3.14159, 3.14159),
        },
        "velocity_range": {},
      },
    ),
    "reset_joints": EventTermCfg(
      func=envs_mdp.reset_joints_by_offset,
      mode="reset",
      params={
        "position_range": (0.0, 0.0),
        "velocity_range": (0.0, 0.0),
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      },
    ),
    "reset_navigation": EventTermCfg(
      func=mdp.reset_navigation,
      mode="reset",
      params={"cfg": task_cfg},
    ),
  }
  if play:
    events["resample_goal"] = EventTermCfg(
      func=mdp.resample_play_goals,
      mode="step",
      params={"cfg": task_cfg},
    )

  reward_cfg = task_cfg.reward
  rewards = {
    "progress": RewardTermCfg(
      func=mdp.goal_progress,
      weight=reward_cfg.progress_weight,
      params={"cfg": task_cfg},
    ),
    "goal": RewardTermCfg(
      func=mdp.goal_reached_reward,
      weight=reward_cfg.goal_weight,
      params={"cfg": task_cfg},
    ),
    "collision": RewardTermCfg(
      func=mdp.obstacle_collision_penalty,
      weight=reward_cfg.collision_weight,
      params={"cfg": task_cfg},
    ),
    "wall_collision": RewardTermCfg(
      func=mdp.wall_collision_penalty,
      weight=reward_cfg.wall_collision_weight,
      params={"cfg": task_cfg},
    ),
    "cbf": RewardTermCfg(
      func=mdp.cbf_reward,
      weight=reward_cfg.cbf_weight,
      params={"cfg": task_cfg},
    ),
    "action_rate": RewardTermCfg(
      func=mdp.action_rate, weight=reward_cfg.action_rate_weight
    ),
    "time": RewardTermCfg(func=mdp.constant, weight=reward_cfg.time_weight),
    "timeout": RewardTermCfg(
      func=mdp.timeout_penalty, weight=reward_cfg.timeout_weight
    ),
  }

  terminations = {
    "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
    "collision": TerminationTermCfg(
      func=mdp.obstacle_collision, params={"cfg": task_cfg}
    ),
    "outside_arena": TerminationTermCfg(
      func=mdp.outside_arena, params={"cfg": task_cfg}
    ),
    "fallen": TerminationTermCfg(func=mdp.fallen),
  }
  if not play:
    terminations["goal_reached"] = TerminationTermCfg(
      func=mdp.goal_reached, params={"cfg": task_cfg}
    )

  curriculum = (
    {
      "obstacle_difficulty": CurriculumTermCfg(
        func=mdp.obstacle_curriculum, params={"cfg": task_cfg}
      )
    }
    if not play
    else {}
  )
  decimation = round(1.0 / (task_cfg.physics_dt * task_cfg.high_level_hz))
  return ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(terrain_type="plane"),
      entities=entities,
      sensors=(),
      num_envs=1 if play else task_cfg.num_envs,
      extent=max(task_cfg.arena.size),
    ),
    observations=observations,
    actions=actions,
    commands={},
    events=events,
    rewards=rewards,
    terminations=terminations,
    curriculum=curriculum,
    metrics={},
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY,
      entity_name="robot",
      body_name="torso_link",
      distance=8.0,
      elevation=-65.0,
      azimuth=90.0,
    ),
    sim=SimulationCfg(
      nconmax=128,
      njmax=2000,
      mujoco=MujocoCfg(
        timestep=task_cfg.physics_dt,
        iterations=10,
        ls_iterations=20,
      ),
    ),
    decimation=decimation,
    episode_length_s=task_cfg.episode_length_s if not play else 1.0e9,
    scale_rewards_by_dt=False,
  )
