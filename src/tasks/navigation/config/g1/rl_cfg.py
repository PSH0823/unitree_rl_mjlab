"""PPO and trainable-GAT configuration."""

from dataclasses import dataclass

from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg

from ...navigation_config import NAVIGATION_CFG


@dataclass
class GATModelCfg(RslRlModelCfg):
  num_nodes: int = 12
  node_dim: int = 9
  gat_embedding_dim: int = 16
  local_state_dim: int = 6


def unitree_g1_navigation_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  cfg = NAVIGATION_CFG
  local_dim = 0 if cfg.actor_state_mode == "gat_only" else 6
  return RslRlOnPolicyRunnerCfg(
    actor=GATModelCfg(
      class_name="src.tasks.navigation.models:GATActorModel",
      hidden_dims=(256, 128, 64),
      activation="elu",
      obs_normalization=False,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 0.6,
        "std_type": "scalar",
      },
      num_nodes=cfg.obstacle.num_constraints + 2,
      local_state_dim=local_dim,
    ),
    critic=RslRlModelCfg(
      hidden_dims=(256, 256, 128),
      activation="elu",
      obs_normalization=True,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      learning_rate=3.0e-4,
      num_learning_epochs=5,
      num_mini_batches=8,
      gamma=0.99,
      lam=0.95,
      entropy_coef=0.005,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="g1_dpcbf_navigation",
    run_name="gat",
    num_steps_per_env=32,
    max_iterations=10000,
    save_interval=100,
    clip_actions=1.0,
  )

