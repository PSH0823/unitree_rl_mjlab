from mjlab.tasks.registry import register_mjlab_task

from ...navigation_env_cfg import make_g1_navigation_env_cfg
from .rl_cfg import unitree_g1_navigation_ppo_runner_cfg


register_mjlab_task(
  task_id="Unitree-G1-DPCBF-Navigation",
  env_cfg=make_g1_navigation_env_cfg(),
  play_env_cfg=make_g1_navigation_env_cfg(play=True),
  rl_cfg=unitree_g1_navigation_ppo_runner_cfg(),
)

