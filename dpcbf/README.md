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

Robot position and yaw come from the configured floating-base body. Body-frame
linear velocity comes from `mj_objectVelocity(..., flg_local=1)`.
Dynamic-obstacle positions and velocities are copied directly from the mocap
obstacle manager. The first two QP inputs are normalized velocity increments
at `qp_parameters.reference_control_frequency_hz` (500 Hz by default). The
applied update is
`v_next = v + delta_v_normalized * reference_frequency * control_dt`.
Consequently the scale is 1 at 0.002 s and 5 at 0.01 s. Position, yaw, and
obstacle motion still use the real simulation timestep.

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
  `b_ecbf = 2 V^T V + (alpha_1 + alpha_2) D_do + alpha_1 alpha_2 D`. A single `alpha_ecbf` sets both alpha values; separate
  `alpha_1` and `alpha_2` keys remain supported.
- `odcbf_enabled` changes each selected DPCBF row to
  `L_f h + L_g h u + w_i alpha h >= 0`, adds one independent `w_i >= 1` per
  obstacle, and adds `decay_weight (w_i - 1)^2` to the objective.
- `slack_enabled` adds an independent `delta_i >= 0` to every active DPCBF and
  eCBF row and adds `slack_weight delta_i^2` to the objective. Acceleration,
  yaw-rate, and velocity bounds remain hard constraints.
- `obstacle_priority: 0` keeps the nearest obstacles inside `p_max`.
  `obstacle_priority: 1` instead sorts them by descending closing alignment
  `-dot(v_obstacle - v_robot, p_obstacle - p_robot) /
  (norm(v_relative) norm(p_relative))`, with distance as the tie-break. Thus an
  approaching obstacle scores near `+1` and a receding obstacle near `-1`.

With `default_num_constraints: 10` and all three optional features enabled, the
QP has 33 variables and 55 rows. Feature branches themselves are negligible;
the extra OSQP variables and rows are the meaningful cost. The matrix uses a
fixed sparse pattern (125 stored entries in this configuration) so warm starts
and matrix updates remain efficient. Very large decay/slack weights can still
worsen numerical conditioning, so solver timing and inaccurate-solution counts
should be monitored when increasing them.

All obstacle, robot, and QP values are loaded at process startup from
`config/dpcbf_config.yaml`. Restart `unitree_mujoco` after changing the file.

## Obstacle shape

`dynamic_obstacles.shape` selects what each mocap obstacle is made of:

- `cylinder` — one `mjGEOM_CYLINDER` sized from `radius_range` and `height`.
  This is the abstraction the DPCBF maths assumes, and the historical default.
- `human` — a standing pedestrian: 14 capsules, ellipsoids and boxes
  (feet, shins, thighs, pelvis, abdomen, chest, shoulders, hanging arms, neck,
  head) whose every dimension is a fraction of the person's standing height, so
  one body plan covers `human.height_range`. The table lives in
  `src/dynamic_obstacles.cpp` (`kHumanParts`), body-local frame: origin between
  the feet on the ground, +x forward, +y left, +z up.

Only the shape changes; the mocap-body interface does not. Each obstacle is
still exactly one mocap body, so `nmocap`, `/sim/mj_state`, the perception
sidecar's mirror model and the ground-truth oracle are all unaffected. Because a
person is not axially symmetric, humans carry a yaw: it follows the walking
direction (and turns on a wall bounce) unless `human.face_travel_direction` is
false, in which case the random spawn heading is held.

The radius handed to DPCBF is the **circumscribed** radius of the standing body,
derived from the parts table rather than hardcoded: `0.151 * height`, so 0.26 m
at 1.75 m. Primitives, not a mesh — they survive `mj_saveXML` into the mirror
model with no `meshdir` fixup, they raycast on every MuJoCo-LiDAR backend, and
14 convex geoms per person cost far less than a triangle soup.

### Choosing the projection height band

`human_band_probe` compiles the same spec the simulator does, isolates one
human, and reports what each `pointcloud_to_laserscan` `[min_height,
max_height]` band recovers — after clipping the band to what the Mid360 can
actually reach at that range (the sensor is mounted upside down, so its
reachable heights at horizontal distance `d` are
`[z - d*tan(52.16deg), z + d*tan(7.21deg)]` around `z = 1.265 m`):

```bash
./simulate/build_ros2/human_band_probe \
    src/assets/robots/unitree_g1/xmls/scene_g1.xml \
    dpcbf/config/dpcbf_config.yaml --range 2.0
```

What it shows, and the reason the band is **not** the tuning lever it looks
like: the hanging arms are as wide as the shoulders and span 0.46-0.79 of
standing height, so every band from the hips to the chin returns the same
silhouette. Measured at 2 m on a 1.80 m body (ground truth 0.271 m), the shipped
`0.15-1.60` band and a hand-picked `1.00-1.40` torso band agree to within 3 mm
of fitted radius. What does move the fit by 2x is **aspect angle**: facing the
sensor gives `r_fit` 0.295 m, side-on gives 0.141 m, because a person is 0.51 m
across and 0.24 m deep. The extractor's `radius_enlargement` (0.17 m) is what
keeps the side-on case covering the truth (0.141 + 0.17 = 0.311 > 0.271).

Two band edges do matter, and both are failure modes rather than tunings:

- **`max_height` below ~0.75 m** leaves only the legs, and the gap between them
  (0.13-0.22 m) exceeds `max_group_distance` (0.10 m) — one person is extracted
  as **two** obstacles.
- **`min_height` above ~1.55 m** leaves only the head, which fits a 0.10 m
  circle, and at ranges under 2.7 m the head is above the sensor's upward FOV
  entirely, so the obstacle vanishes.

Anywhere between those, the band is inert. Keep `0.15 / 1.60`.

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

## DPCBF parameter optimization

`parameter_optimization/tune_dpcbf.py` tunes the base `k_mu`, base `k_lambda`,
and DPCBF `alpha`. One Optuna trial evaluates one parameter set over a fixed,
reproducible set of 2D rollouts. The headless evaluator links the same
`DpcbfSafetyFilter` and OSQP C++ implementation used by MuJoCo.

The editable `CONFIG` dictionary at the top of the Python file contains:

- Optuna trial/rollout counts and all three search ranges.
- Episode timing and the fixed-command/waypoint/Perlin command mixture.
- Obstacle count, radius/speed ranges, and arena size.
- Robot bounds, `p_max`, safety regularizers, and `spawn_clearance`.
- DPCBF, eCBF, ODCBF, and slack switches and weights.
- The outer failure, survival, tracking, intervention, slack, and decay score
  weights.

`spawn_clearance` is a surface-to-surface initial clearance. An obstacle is
accepted only when
`center_distance >= r_rob + r_obs + spawn_clearance`.
Every Optuna trial uses the same rollout indices and scenario seed, so candidate
parameters see identical command and obstacle distributions.

Build and run:

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build --target dpcbf_rollout_evaluator -j2
python3 -m pip install -r dpcbf/parameter_optimization/requirements.txt
python3 dpcbf/parameter_optimization/tune_dpcbf.py
```

A dependency-free smoke evaluation of the midpoint parameters is also
available (PyYAML is still required):

```bash
python3 dpcbf/parameter_optimization/tune_dpcbf.py \
  --evaluate-once --rollouts 2
```

By default, only the resumable SQLite study and `best_dpcbf_config.yaml` remain
under `parameter_optimization/results`. Trial YAML/JSON and smoke-evaluation
files use temporary directories. Set `keep_trial_artifacts: true` or
`write_study_summary: true` in the Python `CONFIG` only when those diagnostics
are needed. A collision, arena exit, or hard QP failure terminates only that
rollout and receives both a failure penalty and a lost survival penalty in the
outer objective. Top candidates should still be validated with unseen scenario
seeds and then in the full MuJoCo G1 simulation.

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
