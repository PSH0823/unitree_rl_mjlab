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
- `actor_state_mode = "gat_only"`: only the learned GAT embedding.
- `actor_state_mode = "local"`: GAT embedding plus
  `[arena_position(2), sin(yaw), cos(yaw), v_s, v_l, yaw_rate,
  velocity_command(3), previous_action(3), sin(goal_heading_error),
  cos(goal_heading_error)]`.
- `dpcbf.{k_mu,k_lambda,alpha}`: pure DPCBF reward parameters. There are no
  eCBF, optimal-decay, or slack-variable branches in this task.
- `dpcbf.filter_actions`: apply the closed-form active-row DPCBF projection
  during training. Play mode disables it to evaluate the learned policy without
  a runtime filter, as in CBF-RL.
- `arena.route_box_margin = 0.5`: the soft route corridor is the rectangle
  whose diagonal corners are the episode start and goal, expanded by 0.5 m.

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
PPO trains it jointly with the policy head. Undetected/padding obstacles remain
masked; there is no large-coordinate dummy node.

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

Actor obstacle perception receives position/radius/velocity noise,
episode-correlated position bias, dropout and FIFO latency. Actor robot state
(`x,y`, yaw, body linear velocity, and yaw rate) receives white noise,
episode-correlated pose bias, pose random walk, and FIFO latency. Actions
receive acceleration scale error, acceleration/yaw-rate noise and latency. The
critic, reward, collision, and termination paths use ground truth.

The success-rate curriculum increases obstacle count, obstacle speed, start
range (`±1`, `±2.5`, then `±4 m`), and sensor/action randomization over four
stages. Goal heading tolerance tightens from `30°` to `20°` and then `10°`;
play evaluates that final `10°` stage.
Success additionally requires speed below `0.2 m/s` and yaw rate below
`0.2 rad/s`.

Obstacle contact costs `-5` per 10 Hz step and terminates only after 0.5 s of
consecutive contact (`-50` terminal cost). Crossing the arena footprint costs
`-10` per step and terminates after 2 s outside or 1 m excessive departure
(`-50`). Route-corridor departure costs `-1` per step without termination.
Falling costs `-100`, timeout costs `-15`, and valid pose-goal success gives
`+20`. Training terminates on goal success with a 30 s timeout. Play does not
terminate or reset on success: the reached pose becomes the next route-box
start and a new goal is sampled continuously. Play has no practical time limit,
while collision, arena-departure, and fall terminations remain active.

The actor uses a tanh-squashed Gaussian, so sampled actions are truly bounded
to `[-1,1]`. PPO starts with standard deviation `0.3`, uses entropy coefficient
`0.001`, and defaults to 4000 iterations to reduce the late-training action
variance growth observed in the older 9900-step checkpoint.

## Run

```bash
python scripts/train.py Unitree-G1-DPCBF-Navigation
```

For a local checkpoint:

```bash
python scripts/play.py Unitree-G1-DPCBF-Navigation \
  --checkpoint-file logs/rsl_rl/g1_dpcbf_navigation/<run>/model_<iteration>.pt
```
