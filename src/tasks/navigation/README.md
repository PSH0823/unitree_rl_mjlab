# G1 DPCBF-RL navigation

This task trains a 10 Hz position-navigation policy on top of the frozen 50 Hz
G1 velocity policy. Each vectorized MuJoCo world contains one robot, one arena,
and its own independently randomized moving cylinders.

## Configuration

Edit `navigation_config.py`. `NAVIGATION_CFG` is the single configuration source
for the arena, robot limits, obstacle selection, DPCBF, rewards, perception and
action randomization, curriculum, rates, and low-level policy path.

Important options:

- `obstacle.priority = 0`: nearest obstacles first.
- `obstacle.priority = 1`: strongest closing relative-velocity alignment first,
  with distance as the tie-break.
- `actor_state_mode = "gat_only"`: only the learned GAT embedding.(recommended)
- `actor_state_mode = "local"`: GAT embedding plus
  `[v_s, v_l, yaw_rate, previous_action(3)]`.
- `dpcbf.{k_mu,k_lambda,alpha}`: pure DPCBF reward parameters. There are no
  eCBF, optimal-decay, or slack-variable branches in this task.
- `dpcbf.filter_actions`: apply the closed-form active-row DPCBF projection
  during training. Play mode disables it to evaluate the learned policy without
  a runtime filter, as in CBF-RL.

`max_count` fixes the compiled scene and graph size. `count` can be changed up
to `max_count` without changing code. If `max_count` or `num_constraints` is
changed, restart training so the registered environment and GAT dimensions are
rebuilt.

## Graph input

The nodes are ordered as robot, selected obstacles, and goal. A node contains:

```text
[label(3), relative_position_body(2), radius,
 relative_velocity_body(2), valid]
```

Labels are robot `[1,0,0]`, obstacle `[0,1,0]`, and goal `[0,0,1]`. `psi1`
creates edge messages from the two labels and
`[dx,dy,surface_distance,dvx,dvy]`; `psi2` computes attention; and `psi3`
updates messages before the weighted sum. The GAT is inside the RSL-RL actor, so
PPO trains it jointly with the policy head. With no detected obstacle, one valid
dummy node uses relative position `(100,100)` and radius `0.2`.

## DPCBF reward and action flow

The actor emits normalized actions that map to `[a_s,a_l,w]`. Accelerations are
SI values in m/s² and are integrated once per high-level step:

```text
v_cmd[k+1] = clip(v_cmd[k] + a[k] / high_level_hz)
```

The frozen ONNX policy receives `[v_s_cmd,v_l_cmd,w]` and produces 29 joint
targets at 50 Hz. During training, the most critical valid DPCBF row projects
the policy action to `u_safe`; play mode uses `u_policy` directly. The single
CBF reward is

```text
cbf_weight * (
  min(Lf_h + Lg_h * u_policy + alpha * h, 0)
  + exp(-||u_policy - u_safe||^2 / 0.5^2)
  - 1
) * active
```

All task rewards use their configured per-step weights directly
(`scale_rewards_by_dt=False`). Progress remains the raw decrease in goal
distance; it is intentionally not divided by `u_max * delta_t`, so its weight
must account for that normalization if desired.

## Randomization policy

Actor perception receives position/radius/velocity noise, episode-correlated
position bias, dropout and a FIFO latency. Actions receive acceleration scale
error, acceleration/yaw-rate noise and latency. The critic, reward, collision,
and termination paths use ground truth. This prevents noisy labels from
changing the task itself while training the actor for LiDAR-like errors.

The success-rate curriculum increases obstacle count, obstacle speed, and then
sensor/action randomization over four stages. Play mode uses full obstacle
difficulty, disables corruption, and samples another random goal whenever the
current goal is reached.

## Run

```bash
python scripts/train.py Unitree-G1-DPCBF-Navigation
```

For a local checkpoint:

```bash
python scripts/play.py Unitree-G1-DPCBF-Navigation \
  --checkpoint-file logs/rsl_rl/g1_dpcbf_navigation/<run>/model_<iteration>.pt
```