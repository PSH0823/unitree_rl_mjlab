# DPCBF safety filter

This directory contains simulation-independent DPCBF and OSQP code plus the
MuJoCo dynamic-obstacle manager. The simulator supplies ground-truth robot and
obstacle states, while the core filter only depends on plain C++ state structs.
That separation allows a real-robot state estimator and obstacle tracker to use
the same filter later.

## Command path

```text
gamepad velocity request
  -> OSQP-DPCBF filter (a_s, a_l, w)
  -> safe velocity request (v_s, v_l, w)
  -> DDS wireless controller
  -> ONNX locomotion policy
  -> joint position targets
```

The simulator uses MuJoCo's `m->opt.timestep` as the integration `dt`. Robot
position and yaw come from the configured floating-base body. Body-frame linear
velocity comes from `mj_objectVelocity(..., flg_local=1)`. Dynamic-obstacle
positions and velocities are copied directly from the mocap obstacle manager.

## QP safety options

`qp_parameters.enabled` is the master switch. When it is `true`, the following
feature switches change the QP structure at startup:

- The existing DPCBF constraints are always included. Setting
  `dpcbf_enabled: false` is also supported as an optional advanced setting.
- `ecbf_enabled` adds one relative-degree-two distance constraint per selected
  obstacle. With `p = p_obstacle - p_robot`,
  `V = v_obstacle - v_robot`, and
  `D = p^T p - (r_robot + r_obstacle)^2`, the implemented row is
  `A_ecbf u + b_ecbf >= 0`, where
  `b_ecbf = 2 V^T V + (alpha_1 + alpha_2) D_dot
  + alpha_1 alpha_2 D`. A single `alpha_ecbf` sets both alpha values; separate
  `alpha_1` and `alpha_2` keys remain supported.
- `odcbf_enabled` changes each selected DPCBF row to
  `L_f h + L_g h u + w_i alpha h >= 0`, adds one independent `w_i >= 1` per
  obstacle, and adds `decay_weight (w_i - 1)^2` to the objective.
- `slack_enabled` adds an independent `delta_i >= 0` to every active DPCBF and
  eCBF row and adds `slack_weight delta_i^2` to the objective. Acceleration,
  yaw-rate, and velocity bounds remain hard constraints.

With `default_num_constraints: 10` and all three optional features enabled, the
QP has 33 variables and 55 rows. Feature branches themselves are negligible;
the extra OSQP variables and rows are the meaningful cost. The matrix uses a
fixed sparse pattern (125 stored entries in this configuration) so warm starts
and matrix updates remain efficient. Very large decay/slack weights can still
worsen numerical conditioning, so solver timing and inaccurate-solution counts
should be monitored when increasing them.

All obstacle, robot, and QP values are loaded at process startup from
`config/dpcbf_config.yaml`. Restart `unitree_mujoco` after changing the file.

## Visualization

Running `./simulate/build/unitree_mujoco` also opens the OpenCV window configured
under `visualization` in `config/dpcbf_config.yaml`.

- The left pane is a world-frame top-down view of the arena, robot, all dynamic
  obstacles, and the robot-centered `p_max` circle.
- Colored obstacles are the nearest obstacles actually used by the QP. The
  filter first rejects obstacles outside `p_max`, sorts the rest by distance,
  and keeps at most `default_num_constraints`. Unselected obstacles remain
  visible in a muted color.
- A colored arrow starting at the robot is
  `v_relative = v_obstacle - v_robot` in the world frame. Each selected
  obstacle also has a matching `h=0` parabola centered on the robot and aligned
  with that obstacle's Line-of-Sight axis.
- The right pane shows one card per selected obstacle. Each card rotates that
  relative velocity into the obstacle Line-of-Sight frame and plots its current
  DPCBF `h=0` parabola. The lightly shaded side is `h<0`.

The parabola freezes the distance-dependent and regularized-speed-dependent
coefficients at the current simulation sample, exactly as they are evaluated by
the QP at that instant. Set `visualization.enabled: false` to disable the extra
window. `velocity_axis_limit` changes the `+/- m/s` range in each velocity card,
and `velocity_arrow_seconds` sets the common visual projection from velocity to
world distance for both the arrows and left-pane parabolas.
`world_parabola_lateral_limit` and `world_parabola_backward_limit` crop the
left-pane curves in LoS velocity units so their negative-X tails do not cover
the world view.

The first CMake configure downloads pinned revisions of `google/osqp-cpp`, OSQP,
and Abseil. Incremental builds do not contact the network.

## Test

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
./simulate/build/dpcbf/dpcbf_safety_filter_test dpcbf/config/dpcbf_config.yaml
```

The test compares the analytical DPCBF derivatives and moving-obstacle drift
against central finite differences, checks eCBF coefficients, velocity
integration, all feature-switch combinations, ODCBF/slack lower bounds, OSQP,
and the configured maximum-obstacle constraint count.
