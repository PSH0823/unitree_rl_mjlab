# DPCBF Perception Subsystem — ROS2 Master Architecture

**Status:** Approved-for-implementation design document (no implementation has started).
**Baseline revision:** `f111cfa6c5ab3dd08b43ca19270017b3ddd98d75` — *"Add training navigation task using dpcbf"* (tip of `origin/dpcbf`). This is the restart point: it contains the original DPCBF subsystem and **no** perception code.
**Historical reference:** branch `dpcbf_perception` (commits `527d89b`, `a01d53c`, `7f2e6d3`) — the abandoned from-scratch perception implementation. It is mined for lessons and constants in §4; none of its code is carried forward as a dependency.
**Document date:** 2026-07-31.
**Rule:** §21 (Implementation Progress Log) is permanent and append-only. Every future implementation phase appends an entry there. This document is the single source of truth for the perception subsystem redesign.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Repository Audit (baseline `f111cfa`)](#2-repository-audit)
3. [Development & Deployment Environment Audit](#3-environment-audit)
4. [Historical Reference: the `dpcbf_perception` Branch Post-Mortem](#4-historical-post-mortem)
5. [External Package Audit](#5-external-package-audit)
6. [Architecture Overview](#6-architecture-overview)
7. [Interfaces: Topics, Messages, QoS](#7-interfaces)
8. [Frames and TF Tree](#8-frames-and-tf-tree)
9. [Perception Pipeline — Stage-by-Stage Design](#9-pipeline-detail)
10. [DPCBF Integration Design](#10-dpcbf-integration)
11. [Simulation Architecture](#11-simulation-architecture)
12. [Deployment Architecture (Real G1)](#12-deployment-architecture)
13. [Directory Structure and Build System](#13-directory-structure)
14. [Launch Architecture and ROS2 Composition Strategy](#14-launch-architecture)
15. [Dependency Graph](#15-dependency-graph)
16. [Testing Roadmap](#16-testing-roadmap)
17. [Evaluation and Benchmarking Roadmap](#17-evaluation-roadmap)
18. [Implementation Phases](#18-implementation-phases)
19. [Risk Analysis](#19-risk-analysis)
20. [Open Questions](#20-open-questions)
21. [Final Recommendations](#21-final-recommendations)
22. [Appendix A — Validated Constants Carried Forward](#appendix-a)
23. [Appendix B — References](#appendix-b)
24. [Implementation Progress Log (permanent, append-only)](#implementation-progress-log)

---

<a name="1-executive-summary"></a>
## 1. Executive Summary

### 1.1 What changes

The previous roadmap (branch `dpcbf_perception`, ~40k lines) implemented raycasting, point-cloud generation, projection, detection, tracking, and safety-obstacle generation from scratch in a ROS-free C++ core. That direction is abandoned. The new architecture reuses mature ROS2 packages for every pipeline stage where one exists, and keeps only four small custom components (≈1–2k lines total, vs 40k):

```
                SIMULATION                         HARDWARE
        ┌───────────────────────┐         ┌───────────────────────┐
        │ simulate (C++ MuJoCo) │         │ Livox Mid360 on G1    │
        │  └ /clock, /sim/state │         │  └ livox_ros_driver2  │
        └──────────┬────────────┘         └──────────┬────────────┘
                   │  (custom sidecar:               │  (unchanged pkg)
                   │   MuJoCo-LiDAR wrapper)         │
                   ▼                                 ▼
             /livox/lidar  sensor_msgs/PointCloud2  (identical topic+frame)
                   │                                 │
                   │            + TF odom→base_link  │
                   │  (sim: ground truth)            │  (hw: LIO — DLIO/FAST-LIO)
                   └────────────────┬────────────────┘
                                    ▼
              [pcl_ros CropBox self-filter]        (unchanged pkg)
                                    ▼
              [patchwork++ ground seg — rough-terrain phase only]
                                    ▼
              [pointcloud_to_laserscan]            (unchanged pkg, binary)
                                    ▼   /scan  (base_footprint frame)
              [obstacle_detector_2: extractor+tracker]  (minimal fork)
                                    ▼   /tracked_obstacles  (odom frame, with velocities)
              [safety_obstacle_filter]             (custom, ~300 LOC)
                                    ▼   /obstacles_safe
              [dpcbf_ros_adapter]                  (custom lib, ~400 LOC,
                                    │               linked into simulate & deploy)
                                    ▼   std::vector<dpcbf::ObstacleState>
              [dpcbf::DpcbfSafetyFilter]           (UNCHANGED — perception-agnostic)
```

### 1.2 Reuse classification (the headline table)

| Component | Package | Classification | Rationale (see §5) |
|---|---|---|---|
| HW LiDAR driver | `livox_ros_driver2` | **Used unchanged** (config only) | Official, Humble/Jazzy, PointCloud2 + IMU output |
| Sim LiDAR | `MuJoCo-LiDAR` (TATP-233) | **Wrapper** (Python sidecar node) | Only maintained MuJoCo LiDAR sim; Python-only → needs a state-mirror bridge to our C++ sim |
| Self-filter / downsample | `perception_pcl` (`pcl_ros`) | **Used unchanged** | Released Humble binaries, composable CropBox/VoxelGrid |
| Ground segmentation | `patchwork-plusplus` | **Used unchanged** (deferred phase) | ROS2-native; only needed for rough terrain |
| 2D projection | `pointcloud_to_laserscan` | **Used unchanged** (binary apt pkg) | Transforms to `target_frame` **before** height slicing — exactly what a swaying humanoid needs |
| Detection + tracking | `obstacle_detector_2` (harmony-eu) | **Fork** (params + small patches) | Only ROS2 package outputting tracked circles `{center, velocity, radius, true_radius}` in a fixed frame; passive maintenance → fork and pin |
| Detection fallback | `nav2_dynamic` (kf_hungarian_tracker) | **Reserve option** | Actively maintained; heavier integration; adapter would map `ObstacleArray` |
| LIO (hardware odom) | DLIO (`feature/ros2`) primary; FAST-LIO humanoid port alternate | **Used unchanged** | DLIO consumes standard PointCloud2; FAST-LIO requires Livox CustomMsg |
| Safety inflation | `safety_obstacle_filter` | **Custom** (~300 LOC) | Project-specific math (validated constants from old branch); no existing package |
| DPCBF bridge | `dpcbf_ros_adapter` | **Custom** (~400 LOC) | The single seam between ROS2 and the unchanged DPCBF library |
| TF utilities | `base_footprint_publisher`, marker relay | **Custom** (tiny) | ~50 LOC each |
| Robot TF model | `g1_description` | **Custom** (data only) | xacro encoding of measured Mid360 extrinsics |
| DPCBF core | `dpcbf/` | **Unchanged, byte-for-byte** | Perception-agnostic by decree |
| Grid conversion | `pointcloud_to_grid` (jkk-research) | **Rejected** | Produces grids, not obstacles; adds a stage the circle detector doesn't need |
| Costmap → primitives | `costmap_converter` | **Rejected** | No Humble/Jazzy release, unmaintained for ROS2 |
| Nav2 costmap / grid_map | `nav2_costmap_2d`, `grid_map` | **Rejected for now** | Costmap machinery without a planner adds latency and still requires custom extraction; grid_map reserved for future terrain work |

### 1.3 Key decisions

- **D1 — ROS2 Humble** on both the dev/sim machine (already installed, Ubuntu 22.04.5) and the G1 onboard PC (ships Ubuntu 22.04). Revisit Jazzy at Humble EOL (May 2027). (§3, §5.10)
- **D2 — CycloneDDS everywhere** (`rmw_cyclonedds_cpp`), sharing one DDS domain with Unitree SDK2, whose `rt/…` topic naming is deliberately ROS2-compatible. (§12.2)
- **D3 — `dpcbf/` is frozen.** No perception concept enters the DPCBF package. All integration flows through `dpcbf_ros_adapter`. (§10)
- **D4 — The simulator publishes only simulated sensor data + state**; the entire perception stack consumes ROS2 topics identically in sim and on hardware. Switching sim↔hardware changes only the PointCloud2/odometry *source*, via a launch argument. (§11, §14)
- **D5 — Oracle-first, shadow-mode second, closed-loop last.** The ground-truth obstacle path stays permanently available in sim for regression and A/B evaluation — the single most valuable process decision inherited from the old branch. (§10.4, §17)
- **D6 — The perception→DPCBF contract is `obstacle_detector` `Obstacles` msgs in the `odom` frame** plus constant-velocity extrapolation at query time in the adapter. No new message package unless covariance export forces one. (§7.3, §10.2)

---

<a name="2-repository-audit"></a>
## 2. Repository Audit (baseline `f111cfa`)

Everything below was verified from source at the baseline revision (detached HEAD `f111cfa`).

### 2.1 Branch topology

```
cbb1ade init … (main lineage: unitree_rl_mjlab upstream)
└─ f111cfa  Add training navigation task using dpcbf   ← BASELINE (origin/dpcbf tip)
   └─ 527d89b done by P3 ─ a01d53c backup ─ 7f2e6d3 backup without ros version
                                            ↑ dpcbf_perception (historical only)
```

- `git merge-base HEAD dpcbf_perception` = `f111cfa` — the perception branch is exactly baseline + 3 commits.
- New work starts from `f111cfa` on a fresh branch (suggested: `dpcbf_perception_ros2`).

### 2.2 Top-level layout at baseline

| Path | Contents | Language | ROS2? |
|---|---|---|---|
| `dpcbf/` | DPCBF safety filter, dynamic obstacle manager, OpenCV visualizer, tuning, tests | C++17 | No |
| `simulate/` | unitree_mujoco fork: physics + GLFW render + Unitree SDK2 DDS bridge; vendored **MuJoCo 3.3.6** (prebuilt) | C++17 | No |
| `deploy/` | Real-robot FSM controller (Passive/FixStand/RLBase/Mimic), ONNX Runtime 1.22 (x64 + aarch64 vendored), Unitree SDK2 | C++17 | No |
| `src/`, `scripts/`, `setup.py` | mjlab (Isaac-Lab-style) RL training, incl. the DPCBF navigation task; mujoco-warp 3.5.0 | Python | No |
| `doc/` | GIFs, licenses | — | — |

**There is no ROS2, rclcpp, rclpy, package.xml, or colcon usage anywhere at baseline.** The only middleware is Unitree SDK2's DDS (CycloneDDS-based).

### 2.3 The DPCBF package — the contract everything must satisfy

Build targets (`dpcbf/CMakeLists.txt`): static libs `dpcbf_dynamic_obstacles`, `dpcbf_safety_filter`, `dpcbf_visualizer` (optional, OpenCV); executables `dpcbf_rollout_evaluator`, `dpcbf_safety_filter_test`. Dependencies: **osqp-cpp** (pinned FetchContent), **abseil-cpp** (pinned), **yaml-cpp**, OpenCV (optional). Eigen arrives transitively via osqp-cpp.

**Obstacle input contract** (`dpcbf/include/dpcbf/dpcbf_safety_filter.h:19-26`):

```cpp
struct ObstacleState {
  double x = 0.0;            // world frame, meters
  double y = 0.0;            // world frame, meters
  double radius = 0.0;       // collision radius, meters (filter applies s=1.05 on top)
  double velocity_x = 0.0;   // world frame, m/s
  double velocity_y = 0.0;   // world frame, m/s
  int id = -1;               // informational only — never used in filter logic
};
```

**Robot state contract** (`dpcbf_safety_filter.h:11-17`): `{x, y, phi}` in world frame; `{sagittal_velocity, lateral_velocity}` in **body frame**. World-frame robot velocity is reconstructed internally via the yaw rotation (`dpcbf_safety_filter.cpp:376-381`).

**Command contract:** in `VelocityCommand{sagittal, lateral, yaw_rate}` → out `SafetyFilterResult{command, acceleration (normalized at `reference_control_frequency_hz`=500), active-constraint counts, solved flag, decay/slack variables, selected_obstacles}`.

**Behavioral facts a perception system must respect:**

1. Obstacle position **and velocity are world-frame** — a body-frame feed would silently corrupt the constraint (`dpcbf_safety_filter.cpp:340`).
2. The filter pre-selects obstacles: within `p_max` (3.0 m default), sorted by distance or closing-alignment (`obstacle_priority` 0/1), truncated to `default_num_constraints` (10). Perception may publish more; the filter culls (`dpcbf_safety_filter.cpp:502-542`).
3. **No motion prediction inside the filter** — it consumes instantaneous `{p, v}`. Staleness compensation is the adapter's job (§10.3).
4. `Filter()` is **not thread-safe** (OSQP instance is not reentrant); it must stay on one thread.
5. `id` is informational; index-as-id is acceptable.
6. Config source: `dpcbf/config/dpcbf_config.yaml` (`r_rob` 0.30 m, `s` 1.05, `alpha` 2.0, `alpha_ecbf` 100, velocity/accel bounds, feature switches for DPCBF/eCBF/ODCBF/slack).

**Invocation rate (verified from source, resolving an audit discrepancy):** the filter runs at **1000 Hz** inside the Unitree SDK2 bridge thread. Call chain: `RecurrentThread("unitree_bridge", …, 1000 /*µs*/)` → `run()` → `lowstate->joystick->update()` (`simulate/src/unitree_sdk2_bridge.h:172-179`) → `axis_filter_(lx, ly, rx)` (`simulate/src/physics_joystick.h:56`) → the lambda at `simulate/src/main.cc:677-709` which snapshots ground-truth obstacles, reads robot ground truth, calls `safety_filter.Filter(...)`, and maps the safe command back to joystick axes. Note the lambda reads `mjData` **without holding `sim.mtx`** — a pre-existing data race the new design must not make worse (§10.5, §19 R-9).

### 2.4 Ground-truth obstacles in simulation today

`dpcbf::DynamicObstacleManager` (`dpcbf/{include/dpcbf/dynamic_obstacles.h, src/dynamic_obstacles.cpp}`): 90 mocap **cylinders**, radius ∈ [0.2, 0.3] m, speed ∈ [0.0, 0.8] m/s, height 1.5 m, in a 20×20 m arena; straight-line motion with boundary reflection. Lifecycle: `AddToSpec()` pre-compile → `BindModel()` → `Step(m, d, dt)` **before** `mj_step` inside `sim.mtx` (`main.cc:529`) → thread-safe `Snapshot()` (internal mutex). This is the **oracle** path and it stays (§10.4). Obstacles have `contype=2/conaffinity=1`: they collide with the robot but not each other.

### 2.5 The simulator (`simulate/`)

- Fork of unitreerobotics/unitree_mujoco. MuJoCo **3.3.6** vendored *prebuilt* (`simulate/mujoco/lib`), not built in-tree.
- Links: `pthread, mujoco, glfw, yaml-cpp, unitree_sdk2, boost_program_options, fmt` (`simulate/CMakeLists.txt`). Plain CMake — no ament, but `find_package(rclcpp)` can be added in the established pattern.
- Threads: **render** (main, GLFW ~60 Hz, locks `sim.mtx`), **physics** (`PhysicsLoop`: locks `sim.mtx`, `dynamic_obstacles.Step` then `mj_step`), **bridge** (RecurrentThread @1 kHz: applies `rt/lowcmd` PD targets to `d->ctrl`, publishes `rt/lowstate` incl. IMU, `rt/wireless_controller`; G1 extras `rt/secondary_imu`, `rt/lf/bmsstate`).
- DDS config: `simulate/config.yaml` → `domain_id: 0`, `interface: "lo"`; CLI overrides in `param.h`.
- Scene `src/assets/robots/unitree_g1/xmls/scene_g1.xml`: floating base `pelvis` → `torso_link` → head geoms; sensors = 29× joint pos/vel/torque + IMU (`imu_quat/gyro/acc` on pelvis site `imu`) + `frame_pos/frame_vel`. **No LiDAR site exists at baseline** — adding one is a Phase-1 change (§11.3).

### 2.6 Deployment (`deploy/`)

- Per-robot mains (`deploy/robots/g1/main.cpp` etc.), FSM at 1 kHz: subscribe `rt/lowstate`, publish `rt/lowcmd`; ONNX policy inference in `State_RLBase`. Platform: aarch64 (G1 onboard PC) and x64 (bench).
- Depends on system-wide `unitree_sdk2` (`/opt/unitree_robotics`), `ddscxx` (CycloneDDS C++), Eigen, yaml-cpp, Boost, fmt, cnpy. **No colcon, no perception code, no odometry consumer** at baseline.
- The velocity-command path on hardware (joystick → policy command) currently has **no DPCBF filter**; inserting it mirrors the simulator's `axis_filter` seam (§12.4).

### 2.7 Training stack (context only — out of scope for this subsystem)

The navigation task (`src/tasks/navigation/`) trains against **ground-truth obstacles with synthetic perception noise** (pos σ 0.03 m, vel σ 0.05 m/s, radius σ 0.015 m, 3% dropout, 1-step latency @10 Hz; asymmetric actor/critic). The observation contract (≤10 obstacles, body-yaw frame relative coordinates) tells us what the *policy* expects, but the perception subsystem targets the *safety filter*, not policy observations. The noise model is a useful reference target for what the real pipeline should achieve (§17.2). Training-side DPCBF is a closed-form single-constraint projection in PyTorch (`src/tasks/navigation/mdp/state.py:350-389`) — unaffected by this redesign.

### 2.8 What this audit pins down

- Integration points: **(a)** the `axis_filter` lambda in `simulate/src/main.cc` (sim closed loop), **(b)** the velocity-command path in `deploy` (hardware closed loop), **(c)** `scene_g1.xml` (sensor site), **(d)** `simulate` CMake (optional ROS2 module). Nothing else needs to change at baseline.
- Existing DPCBF assumptions: world-frame obstacles, ≤10 used, 3 m horizon, 1 kHz single-threaded consumer, no internal prediction.
- MuJoCo interface for a LiDAR: `mjModel/mjData` + site pose; physics thread owns `mjData` under `sim.mtx`.
- Deployment interface: CycloneDDS domain shared with Unitree SDK2; `rt/…` topics visible to ROS2 as `/lowstate` etc. when RMW is CycloneDDS.

---

<a name="3-environment-audit"></a>
## 3. Development & Deployment Environment Audit

Verified on the dev machine (2026-07-31):

| Item | Fact | Consequence |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS | Humble is the native distro |
| ROS2 | **Humble installed and sourced** (`ROS_DISTRO=humble`); RViz2, `pcl_ros`, `pcl_conversions`, `laser_geometry` present | Stage-3 pipeline packages partially pre-installed |
| RMW | `rmw_fastrtps_cpp` installed; **`rmw_cyclonedds_cpp` NOT installed** | Must `apt install ros-humble-rmw-cyclonedds-cpp` (D2) |
| Missing pkgs | `pointcloud_to_laserscan`, nav2, grid_map not installed | Phase 0 installs |
| DDS env | `ROS_DOMAIN_ID` unset (=0), `ROS_LOCALHOST_ONLY=0` | Matches simulate `domain_id: 0` — intentional overlap (§12.2) |
| GPU | RTX 2080 Ti, **driver/library mismatch (nvidia-smi fails)** | GPU backends of MuJoCo-LiDAR unusable until fixed; CPU backend suffices (§5.1) |
| Python | conda env active (Python 3.12) vs Humble's system Python 3.10 | The MuJoCo-LiDAR sidecar must run on **system Python 3.10** with rclpy; document env hygiene in bringup README (§19 R-11) |
| Build | `colcon` present | Workspace tooling ready |

G1 onboard (from vendor documentation, to be verified in Phase 5): Ubuntu 22.04, Jetson Orin NX (EDU) — Humble matches; Mid360 factory-mounted upside-down on the head.

---

<a name="4-historical-post-mortem"></a>
## 4. Historical Reference: the `dpcbf_perception` Branch Post-Mortem

The abandoned branch is a high-quality engineering artifact (~40k lines incl. tests, plus an extensive phase log). Its *architecture* is replaced; its *measurements* are gold. Read access: `git show dpcbf_perception:<path>`.

### 4.1 Why it is being abandoned

1. **Scale:** it re-implemented raycasting, projection, detection, tracking, safety, diagnostics, replay, and a codec layer — ~40k lines to maintain for one robot.
2. **The ROS gap it hit is now closed:** its Option-C analysis correctly found that ROS1 `obstacle_detector` (catkin) and ROS2 `pointcloud_to_laserscan` (ament) cannot run together — but `obstacle_detector_2` (a ROS2 Humble port) exists, dissolving the core reason for going ROS-free (§5.3).
3. **Hardware path was never reached:** deskew (P5) and the real-driver adapter were planned, not built. The ROS2 ecosystem gives both for free (driver timestamps; LIO handles deskew).

### 4.2 Measurements and decisions carried forward (normative for the new design)

| # | Finding | Carried into |
|---|---|---|
| H-1 | **Mid360 extrinsics** (from Unitree URDF `mid360_joint`): parent `torso_link`, translation `[0.0002835, 0.00003, 0.428434]` m, RPY `[π, 0.000892 rad (0.0511°), 0]` — **roll = π, the sensor is mounted upside-down**; optical origin ≈1.2654 m above floor when standing | `g1_description` (§8.3), sim site (§11.3), `MID360_config.json` |
| H-2 | Vertical FOV −7°…+52° in sensor frame → **+4.07°…−54.93° after the flip** (sweeps the ground ahead) | Slice-band and mounting sanity checks (§9.4) |
| H-3 | **`mj_multiRay` misses occlusions**: AABB-corner pruning lets up to 23.7% of rays pass through large flat geoms; error is always a *missed hit*, never a wrong range. Per-ray `mj_ray` is exact at 1.8× cost (8.33 ms vs 4.55 ms for 11 520 rays) | Phase-1 acceptance gate for MuJoCo-LiDAR's CPU backend, which wraps `mj_multiRay` (§11.4, §19 R-1) |
| H-4 | **Aperture masking**: `head_link` geoms enclose the sensor origin; without masking ~100% self-hits. Fix: `group="2"` on head geoms + `geomgroup` mask in raycast → 2.8% residual self-hit, ~8 800 valid pts/frame | Sim scene edit (§11.3); on hardware replaced by CropBox self-filter (§9.3) |
| H-5 | **Uniform-grid projection neutralizes the Livox rosette**: the split-and-merge detector assumes uniform angular increments; putting a uniform-bin projector in front makes the detector's input contract hold regardless of the scan pattern | Retained exactly, but via `pointcloud_to_laserscan` instead of custom code (§9.4) |
| H-6 | **Timer-driven KF tracking is wrong**: upstream `obstacle_tracker` runs at `loop_rate` (100 Hz), re-applying the last measurement between scans instead of coasting — non-deterministic and biased during occlusion. Measurement-driven dt is correct | Fork candidate for `obstacle_detector_2` (§9.5, patch P-2) |
| H-7 | Detector re-tuning: `radius_enlargement` 0.25→**0.17 m** (measured short-arc bias ±0.09–0.19 m center, ±0.006–0.145 m radius); association gate 0.30 m with radius-residual down-weight 0.3; confirm after 3 hits | Fork parameter file (§9.5, Appendix A) |
| H-8 | Safety inflation formula and constants: `r_safe = max(r_true, 0.20) + k_σ·σ + fixed 0.03 + latency_horizon·|v|`, `k_σ = 2.748` (~99.9% containment), `max_age` 0.30 s | `safety_obstacle_filter` (§9.6) |
| H-9 | **Per-frame fallback, never per-obstacle mixing**: oracle and estimated obstacles must not mix inside one QP solve | `dpcbf_ros_adapter` mode logic (§10.4) |
| H-10 | Oracle permanence + bit-for-bit regression against pre-change behavior proved refactors safe | Testing roadmap (§16) |
| H-11 | Tracking velocity RMSE < 0.1 m/s at obstacle speeds ≤ 0.8 m/s is achievable from this sensor geometry | Evaluation targets (§17.2) |

### 4.3 Explicitly not carried forward

Custom raycaster (→ MuJoCo-LiDAR), custom projector (→ `pointcloud_to_laserscan`), custom detector/tracker code (→ `obstacle_detector_2` fork), custom diagnostics/dump/JSON codec (→ `rosbag2`/MCAP + Foxglove), scan accumulation & deskew infrastructure (→ driver timestamps + LIO), the 9-step mode ladder (collapsed to 3 modes: oracle / shadow / estimated).

---

<a name="5-external-package-audit"></a>
## 5. External Package Audit

Each verdict below is grounded in source/README/issue inspection performed for this document (July 2026).

### 5.1 MuJoCo-LiDAR — `github.com/TATP-233/MuJoCo-LiDAR` — **ADOPT via wrapper**

- **What it is:** pure-Python LiDAR simulation library for MuJoCo. `MjLidarWrapper(mj_model, site_name, backend, cutoff_dist)`; backends: `cpu` (wraps `mj_multiRay`), `taichi`, `jax`, `warp` (own BVH ray tracers). `LivoxGenerator("mid360")` yields the **non-repetitive Mid360 rosette** from a pre-computed `mid360.npy` (~24 000 rays/frame); also Velodyne-style and generic patterns. Ships ROS2 examples (`lidar_vis_ros2.py`, `unitree_g1_ros2.py`) publishing standard `PointCloud2` (x,y,z float32) + TF. Headless capable. MIT license, ~147 stars, active 2025–2026, 1 open issue.
- **Performance:** CPU ≈ 9 M rays/s. Our need: 24 000 rays × 10 Hz = 0.24 M rays/s → **CPU backend is ~37× headroom; no GPU required** (important given the dev box's broken NVIDIA driver).
- **Limitation 1 — Python-only:** cannot be linked into the C++ `simulate` binary. Resolution: run it as a **sidecar process** that mirrors sim state (§11.2). It is a library that accepts `mjModel/mjData`, so a mirror model driven by `qpos/mocap_pos` messages works; the library does not provide a sync mechanism — that bridge is ours (~200 LOC Python).
- **Limitation 2 — CPU backend inherits `mj_multiRay`:** per H-3 this can miss occlusions on large flat geoms (arena walls). Phase-1 gate: reproduce the wall test; if it fails, either (a) switch backend to `warp`/`taichi` (independent BVH code path), or (b) patch the CPU backend to per-ray `mj_ray` (small, upstreamable).
- **Limitation 3 — FOV metadata discrepancy:** repo notes vs datasheet disagree on vertical FOV; the Livox datasheet says −7°…+52°. Phase-1 validates the `.npy` pattern against the datasheet envelope (§20 Q-3).
- **Alternatives rejected:** MuJoCo built-in `rangefinder` sensors (one ray each — unusable at 24k rays); `mujoco_ros2_control` RangefinderLidar plugin (single-axis rays, owns the sim loop); `livox_laser_simulation_ros2` (Gazebo-classic only — though its CSV pattern files are a backup pattern source); full Python sim rewrite (abandons the validated C++ SDK2 bridge and 1 kHz loop).

### 5.2 `pointcloud_to_grid` — `github.com/jkk-research/pointcloud_to_grid` — **REJECT**

ROS2 package converting PointCloud2 → two 2D `nav_msgs/OccupancyGrid`s (height & intensity). Maintained, but it is *only* a rasterization stage: no ground handling beyond naive thresholds, no detection, no tracking, no obstacle primitives. A grid intermediate is the wrong shape for a circle-detector pipeline — `pointcloud_to_laserscan` feeds `obstacle_detector_2` directly, without inventing and then re-segmenting a raster. Would only become relevant if we later switch to a costmap-style pipeline.

### 5.3 `obstacle_detector_2` — `github.com/harmony-eu/obstacle_detector_2` — **ADOPT as fork**

ROS2 (branch `humble-devel`, 43 commits) port of tysik/obstacle_detector (Przybyła 2017: split-and-merge segment extraction, circle grouping, per-obstacle Kalman tracking).

- **Nodes:** `obstacle_extractor` (LaserScan → segments+circles; **`transform_coordinates`+`frame_id` express output in a fixed frame** — verified), `obstacle_tracker` (KF; **supplements circles with velocity estimates**, expressed in the tracker frame — verified), `obstacle_publisher` (virtual obstacles for tests). `scans_merger` split into a separate repo (we don't need it — single LiDAR).
- **Messages:** `Obstacles{header, segments[], circles[]}`; `CircleObstacle{center: Point, velocity: Vector3, radius (with margin), true_radius}` — maps 1:1 onto `dpcbf::ObstacleState` (no ID field → index-as-id, acceptable per §2.3-5).
- **Fit:** input LaserScan (exactly what `pointcloud_to_laserscan` emits); output tracked circles with world/odom-frame velocities (exactly what DPCBF eats). The old branch ported *this same algorithm* by hand — H-7's parameters transfer directly.
- **Concerns → fork scope:** (i) passive maintenance (last substantive activity ~2022; Humble only) → pin to our fork; (ii) upstream tracker is timer-driven at `loop_rate` 100 Hz (H-6) → first mitigate by config (`loop_rate` ≈ scan rate, `tracking_duration` 1.0 s), patch to measurement-driven only if velocity noise fails §17 targets; (iii) parameters re-tuned per Appendix A; (iv) QoS review (sensor-data profile).
- **Original ROS1 repo:** reference for the paper, defaults (`loop_rate` 100, `tracking_duration` 2.0, variances 0.01/0.1/1.0) and RViz plugin ideas; not a build dependency.

### 5.4 `nav2_dynamic` — `github.com/ros-navigation/navigation2_dynamic` — **RESERVE**

Euclidean-clustering detection + `kf_hungarian_tracker` publishing `ObstacleArray` (position, velocity, size + UUID). Actively maintained under the Nav2 org (Humble & Jazzy). Heavier to adapt (3D clusters → circles; different message; needs its own projection choices). Kept as the fallback if the `obstacle_detector_2` fork underperforms on real Mid360 data; the `safety_obstacle_filter` boundary (§9.6) is where the swap would happen, invisible to DPCBF.

### 5.5 `pointcloud_to_laserscan` — ros-perception — **ADOPT unchanged (apt binary)**

Humble 2.0.1 / Jazzy 2.0.2. Composable node. **Verified: transforms the cloud into `target_frame` first, then applies `min_height/max_height` slicing and angular binning.** With `target_frame = base_footprint` (gravity-aligned, ground-height robot frame, §8.2) the slice band stays horizontal while the torso pitches/rolls during walking — precisely the stabilization the old branch built custom gravity-alignment code for. Output `LaserScan` with uniform `angle_increment` (H-5 satisfied). QoS: sensor-data; `use_inf=true`.

### 5.6 `livox_ros_driver2` — Livox-SDK — **ADOPT unchanged (config only)**

Official driver, ROS2 Humble/Jazzy, requires Livox-SDK2. `xfer_format`: 0 = PointCloud2 (`PointXYZRTLT`: x,y,z,intensity,tag,line,per-point timestamp), 1 = `CustomMsg` (FAST-LIO's required input), 2 = PointCloud2 (plain XYZI). Publishes `/livox/lidar` and `/livox/imu` (built-in 6-axis IMU, ~200 Hz). Config `MID360_config.json`: lidar/host IPs, ports, extrinsics, timestamp mode. **Contract consequence:** the perception stack must depend only on `x,y,z` (+ optionally intensity) fields so that both sim (plain XYZ[I]) and any hardware `xfer_format` satisfy it (§7.2). The driver can dual-publish CustomMsg for FAST-LIO without affecting the perception contract.

### 5.7 LiDAR-inertial odometry — **ADOPT unchanged; DLIO primary, FAST-LIO-humanoid alternate**

Perception needs `odom→base_link` TF + `nav_msgs/Odometry` on hardware (§8). Options verified:

| Package | ROS2 status | Mid360 | Input | Note |
|---|---|---|---|---|
| **DLIO** (`vectr-ucla/direct_lidar_inertial_odometry`) | `feature/ros2` branch, Humble | proven | **standard PointCloud2 + IMU** | Primary: no CustomMsg dependency; internal deskew |
| FAST-LIO2 community ports; **`deepglint/FAST_LIO_LOCALIZATION_HUMANOID`** | Humble ports | **validated on G1+Mid360 walking** | `CustomMsg` only | Alternate: strongest humanoid evidence, but couples driver format |
| KISS-ICP (+ EKF for IMU) | native ROS2 | yes | PointCloud2 | IMU-less fallback |
| LIO-SAM | port pending | yes | PointCloud2+IMU | Heavier; mapping-oriented |
| Unitree kinematic odometry (`rt/odommodestate`) | via SDK2 | n/a | — | Sanity cross-check only; drift/frame semantics unverified (§20 Q-5) |

LIO runs **only on hardware**; in sim, ground-truth odometry is published on the identical interface (§11.5), so the perception stack cannot tell the difference — and LIO-in-sim remains possible as a test by feeding it the simulated cloud.

### 5.8 Ground segmentation — `patchwork-plusplus` — **ADOPT unchanged, deferred**

ROS2-native (Humble/Jazzy), adaptive, humanoid-sway tolerant, ~10–30 ms/scan on Orin-class CPUs. For the flat 20×20 arena (Phases 1–4) the `base_footprint` height band `min_height=0.15` already excludes the floor; patchwork++ is inserted between self-filter and projection **only in the rough-terrain phase** (Phase 7), keeping topic interfaces unchanged. `linefit_ground_segmentation_ros2` is the lighter fallback.

### 5.9 Rejected / deferred ecosystem packages

- **`costmap_converter`** — no Humble/Jazzy release, ROS2 port unmaintained. Rejected.
- **`nav2_costmap_2d` obstacle/voxel layers** — sensible only when running Nav2; standalone use still requires custom extraction of primitives and adds raster latency. Rejected for the CBF path.
- **`grid_map` / `elevation_mapping`** — alive and good, but 2.5D terrain is future work (locomotion-facing, not CBF-facing). Deferred; the architecture leaves a tap point at `/points_no_ground` for it.
- **Autoware perception** — vastly over-scoped for 2D circle obstacles on an embedded PC.
- **`pcl_ros`/`perception_pcl`** — **adopted** for CropBox (self-filter) and optional VoxelGrid, as composable nodes (Humble binaries already on the dev machine).
- **Visualization/replay:** RViz2 + `foxglove_bridge` + `rosbag2` (MCAP) — adopted unchanged (§14.4).

### 5.10 Distro decision

**Humble.** Reasons: dev machine and G1 onboard PC both run Ubuntu 22.04; `obstacle_detector_2` is Humble-only; every other chosen package has Humble support; the FAST-LIO humanoid stack is Humble. Cost: EOL May 2027 → recorded as risk R-12 with a planned Jazzy migration checkpoint. Nothing in the architecture is Humble-specific except the fork pin.

---

<a name="6-architecture-overview"></a>
## 6. Architecture Overview

### 6.1 Layering and ownership

Eight layers, each with a single responsibility and an owner module. **No hidden coupling:** every arrow in §6.2 is a ROS2 topic/TF, a documented C++ call, or a config file — nothing else.

| Layer | Responsibility | Modules (owner) | May depend on |
|---|---|---|---|
| **Simulation** | Physics, robot policy I/O, GT obstacles, publishing sim state/clock | `simulate/` (+ optional ROS2 module), `dpcbf::DynamicObstacleManager` | MuJoCo, SDK2, (rclcpp optional) |
| **Sensor source** | Producing `/livox/lidar` (+ IMU) | sim: `sim_mjlidar_bridge` (wraps MuJoCo-LiDAR); hw: `livox_ros_driver2` | Simulation state topic / hardware |
| **State estimation** | `odom→base_link` TF + `/odom` | sim: GT from `sim_mjlidar_bridge`; hw: DLIO (or FAST-LIO-humanoid) | Sensor source |
| **Perception** | PointCloud2 → tracked, inflated circles in `odom` | `pcl_ros` CropBox, (patchwork++), `pointcloud_to_laserscan`, `obstacle_detector_2` fork, `safety_obstacle_filter` | Sensor source, TF |
| **DPCBF** | Safety filtering of velocity commands | `dpcbf/` (frozen) + `dpcbf_ros_adapter` (the only bridge) | Perception output topic |
| **Visualization** | RViz/Foxglove displays, markers | `g1_perception_utils` marker relay, bringup configs, existing `dpcbf_visualizer` (sim-only, untouched) | Any topic (read-only) |
| **Deployment** | Bringup on the G1, DDS coexistence, FSM integration | `g1_perception_bringup`, `deploy/` | Everything above |
| **Testing/Evaluation** | Regression, A/B oracle-vs-perception, benchmarks | test packages, rosbag2 datasets, eval scripts | Read-only taps |

### 6.2 Runtime dataflow (both worlds)

```
════════ SIMULATION ONLY ═══════════╗   ╔════════ HARDWARE ONLY ═══════════
 simulate (C++)                     ║   ║  Livox Mid360 ──UDP──▶ livox_ros_driver2
  ├─ physics 500 Hz (mj_step,       ║   ║        │                   ├─▶ /livox/lidar
  │   GT obstacles step)            ║   ║        │                   └─▶ /livox/imu
  ├─ SDK2 bridge 1 kHz              ║   ║        ▼
  │   (rt/lowstate, rt/lowcmd)      ║   ║   DLIO (feature/ros2)
  ├─ /clock            (rclcpp opt) ║   ║     ├─▶ /odom
  ├─ /sim/mj_state 100 Hz           ║   ║     └─▶ TF odom→base_link
  └─ /sim/gt_obstacles 50 Hz        ║   ║
        │                           ║   ║
        ▼                           ║   ║
 sim_mjlidar_bridge (Python sidecar)║   ║
  mirrors MJCF, mj_kinematics,      ║   ║
  MuJoCo-LiDAR raycast @10 Hz       ║   ║
  ├─▶ /livox/lidar                  ║   ║
  ├─▶ /livox/imu (optional)         ║   ║
  ├─▶ /odom  (ground truth)         ║   ║
  └─▶ TF odom→base_link             ║   ║
════════════════════════════════════╝   ╚══════════════════════════════════
                └───────────────┬───────────────────┘
                                ▼        SHARED PERCEPTION STACK (identical nodes,
   robot_state_publisher (g1_description):        identical topics, identical params;
       TF base_link→torso_link→mid360_link…       only the source launch file differs)
   base_footprint_publisher: TF odom→base_footprint
                                │
   /livox/lidar ─▶ CropBox self-filter ─▶ /points_self_filtered
                 [Phase 7 only: patchwork++ ─▶ /points_no_ground]
                 ─▶ pointcloud_to_laserscan (target_frame=base_footprint,
                        min_height 0.15, max_height 1.60) ─▶ /scan
                 ─▶ obstacle_extractor (frame_id=odom) ─▶ /raw_obstacles
                 ─▶ obstacle_tracker ─▶ /tracked_obstacles      [odom frame, velocities]
                 ─▶ safety_obstacle_filter ─▶ /obstacles_safe   [gated + inflated]
                                │
                                ▼
   dpcbf_ros_adapter (C++ lib, inside the consumer process)
     dble-buffer + const-velocity extrapolation + staleness policy
                                │ std::vector<dpcbf::ObstacleState> @ query time
                                ▼
   dpcbf::DpcbfSafetyFilter::Filter()   (1 kHz, unchanged)
        sim: axis_filter lambda in simulate   hw: velocity-command path in deploy
```

**Where ROS2 begins and ends:** ROS2 begins at the sensor-source publishers and ends inside `dpcbf_ros_adapter`. The DPCBF core, the physics loop, the SDK2 low-level control path, and the training stack never see ROS2. The simulator's ROS2 surface is exactly three publishers (`/clock`, `/sim/mj_state`, `/sim/gt_obstacles`) plus the adapter subscription when perception mode is active.

### 6.3 Module ownership

| Module | Repo location | Kind | Lines (est.) | Owner responsibility |
|---|---|---|---|---|
| `g1_description` | `ros2/src/g1_perception/g1_description` | data (xacro) | ~100 | The **only** place extrinsics live for ROS (§8.3) |
| `sim_mjlidar_bridge` | `ros2/src/g1_perception/sim_mjlidar_bridge` | Python node | ~250 | Mirror-state raycasting + sim GT odom/TF; owns nothing else |
| `base_footprint_publisher` | `g1_perception_utils` | C++ node | ~60 | odom→base_footprint TF only |
| `obstacles_marker_relay` | `g1_perception_utils` | C++ node | ~80 | `Obstacles` → `MarkerArray` for RViz/Foxglove |
| `safety_obstacle_filter` | own package | C++ composable node | ~300 | Gating + inflation (H-8) only; republishes `Obstacles` |
| `dpcbf_ros_adapter` | own package | C++ static lib + tests | ~400 | Topic→`ObstacleState` bridge, extrapolation, staleness/fallback policy, mode switch |
| `obstacle_detector_2` fork | external, pinned via `deps.repos` | fork | patches only | Params (Appendix A) + P-1/P-2 patches (§9.5) |
| `g1_perception_bringup` | own package | launch/config | ~300 | All launch files, parameter YAMLs, RViz/Foxglove layouts |
| `simulate` ROS2 module | `simulate/src/ros2_bridge.{h,cc}` | optional C++ | ~150 | `/clock`, `/sim/mj_state`, `/sim/gt_obstacles` publishers; guarded by CMake option |

---

<a name="7-interfaces"></a>
## 7. Interfaces: Topics, Messages, QoS

### 7.1 Topic table (canonical names — identical in sim and hardware)

| Topic | Type | Rate | Frame | QoS | Producer (sim / hw) |
|---|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz | `mid360_link` | SensorData (best-effort, depth 5) | `sim_mjlidar_bridge` / `livox_ros_driver2` |
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz | `mid360_link` | SensorData | sidecar (optional) / driver |
| `/odom` | `nav_msgs/Odometry` | ≥50 Hz | `odom`→`base_link` | Reliable, depth 10 | sidecar (GT) / DLIO |
| TF `odom→base_link` | tf2 | with `/odom` | — | tf default | sidecar / DLIO |
| TF static (robot) | tf2_static | latched | — | — | `robot_state_publisher` (both) |
| TF `odom→base_footprint` | tf2 | 100 Hz | — | — | `base_footprint_publisher` (both) |
| `/points_self_filtered` | PointCloud2 | 10 Hz | `mid360_link` | SensorData | CropBox component |
| `/points_no_ground` | PointCloud2 | 10 Hz | `mid360_link` | SensorData | patchwork++ (Phase 7 only) |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `base_footprint` | SensorData | `pointcloud_to_laserscan` |
| `/raw_obstacles` | `obstacle_detector/Obstacles` | 10 Hz | `odom` | Reliable, depth 5 | `obstacle_extractor` |
| `/tracked_obstacles` | `obstacle_detector/Obstacles` | 10–100 Hz (tracker `loop_rate`) | `odom` | Reliable, depth 5 | `obstacle_tracker` |
| `/obstacles_safe` | `obstacle_detector/Obstacles` | = tracker rate | `odom` | Reliable, depth 1 (latest wins) | `safety_obstacle_filter` |
| `/sim/mj_state` | custom `sim_msgs/MjState` (qpos[], mocap_pos[], mocap_quat[], sim_time) | 100 Hz | — | BestEffort depth 1 | simulate (sim only) |
| `/sim/gt_obstacles` | `obstacle_detector/Obstacles` | 50 Hz | `odom` | Reliable depth 1 | simulate (sim only; oracle/eval) |
| `/clock` | `rosgraph_msgs/Clock` | ≥100 Hz | — | Clock QoS | simulate (sim only) |
| `/dpcbf/status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | — | Reliable | adapter (both; mode, staleness, fallback state) |

### 7.2 The PointCloud2 contract (the sim/hardware seam)

Perception consumes **only** `x, y, z` (float32) and tolerates any extra fields. This makes all three hardware `xfer_format` variants and the sim sidecar output interchangeable. Per-point timestamps/ring/tag are *not* part of the perception contract (they remain available to LIO, which is a separate consumer). `frame_id` must be `mid360_link` in both worlds.

### 7.3 The obstacle contract

`obstacle_detector/msg/CircleObstacle` → `dpcbf::ObstacleState` mapping (performed in `dpcbf_ros_adapter`):

| CircleObstacle field | ObstacleState field | Note |
|---|---|---|
| `center.x/.y` (odom) | `x/y` | odom ≡ DPCBF "world" (§8.1) |
| `velocity.x/.y` (odom) | `velocity_x/velocity_y` | tracker KF estimate |
| `radius` (post-inflation by `safety_obstacle_filter`) | `radius` | filter's `s=1.05` still applies on top |
| `true_radius` | — (diagnostic only) | |
| (array index) | `id` | informational (§2.3-6) |
| `header.stamp` | — | drives extrapolation & staleness (§10.3) |

**Decision D6:** no new obstacle message package. If Phase 4 shows the inflation needs per-track covariance from the tracker (upstream doesn't publish it), the fork adds a `covariance[3]` field to `CircleObstacle` — recorded as the one sanctioned message change (§20 Q-2).

---

<a name="8-frames-and-tf-tree"></a>
## 8. Frames and TF Tree

### 8.1 Frame semantics

| Frame | Definition | Who publishes |
|---|---|---|
| `odom` | Gravity-aligned, locally-consistent world frame; **this is DPCBF's "world frame"** | sim: sidecar (≡ MuJoCo world); hw: LIO |
| `base_link` | ≡ MuJoCo `pelvis` (floating base) | sim: sidecar; hw: LIO (+ its extrinsic config) |
| `base_footprint` | `base_link` projected: z=0 (ground), roll=pitch=0, yaw=base yaw | `base_footprint_publisher` |
| `torso_link` | per URDF/MJCF kinematics | `robot_state_publisher` (static approx., §8.3) |
| `mid360_link` | LiDAR optical frame (H-1 extrinsics; **roll=π**) | `robot_state_publisher` |
| `map` | absent by design — DPCBF needs local consistency, not global localization | — |

In sim, `odom` ≡ MuJoCo world exactly (GT). On hardware, `odom` drifts slowly; DPCBF only consumes *relative* geometry (robot and obstacles in the same frame), so slow drift is harmless — obstacle and robot state always come from the same `odom`.

### 8.2 TF trees

```
SIM                                  HARDWARE
odom (=MuJoCo world)                 odom (LIO origin at boot)
 ├─ base_link      [sidecar, GT]      ├─ base_link       [DLIO]
 │   └─ torso_link [static approx]    │   └─ torso_link  [static approx]
 │       └─ mid360_link [static H-1]  │       └─ mid360_link [static H-1]
 └─ base_footprint [footprint pub]    └─ base_footprint  [footprint pub]
```

### 8.3 Extrinsics policy

- Single source of truth: `g1_description/urdf/g1_mid360.xacro`, encoding H-1: `torso_link → mid360_link`, xyz `0.0002835 0.00003 0.428434`, rpy `3.14159265 0.000892 0` .
- `base_link→torso_link` is published as a **fixed** transform at the nominal standing waist pose. This is an approximation (the waist joint moves); its error budget is bounded by the waist range of motion and is acceptable for 2D obstacle geometry at ≤3 m (the slice band absorbs vertical error; azimuthal error from waist yaw is the real term — see risk R-8). If Phase 5 shows it matters, upgrade path: publish `base_link→torso_link` from joint states (`rt/lowstate` waist encoders → `joint_state_publisher` input), which slots into the same TF tree with no downstream change.
- The MJCF `<site name="mid360_link">` added to `scene_g1.xml` must match the xacro; a Phase-1 test asserts equality (the old branch's "extrinsic guard", H-1/§16.2-T7).
- `MID360_config.json` extrinsics stay identity (driver outputs in sensor frame; TF does the rest).

---

<a name="9-pipeline-detail"></a>
## 9. Perception Pipeline — Stage-by-Stage Design

### 9.1 Sources (stage 0)

Covered in §11 (sim) and §12 (hardware). Contract: §7.2.

### 9.2 Odometry (stage 1)

Hardware: DLIO consumes `/livox/lidar` + `/livox/imu`, publishes `/odom` + TF at LiDAR rate with internal deskew. Sim: GT. Requirement for downstream: TF `odom→base_footprint` available within 50 ms of a cloud's stamp (checked by the launch-level TF monitor, §16.2-T9).

### 9.3 Self-filter (stage 2) — `pcl_ros` CropBox component

Removes robot-body returns (arms swing through the Mid360's downward-sweeping FOV; hardware has no `group=2` trick, so H-4's masking is replaced by geometry filtering). CropBox in `mid360_link` frame, `negative=true`, box ≈ `x∈[−0.35,0.35], y∈[−0.35,0.35], z∈[−0.55,0.25]` m (sensor-frame; covers torso/head/upper arms below-sensor volume; tune in Phase 5 with real self-hit data). Optional VoxelGrid (leaf 0.05 m) if CPU requires — off by default at 20k pts/frame.

### 9.4 Projection (stage 3) — `pointcloud_to_laserscan`

- `target_frame: base_footprint` (gravity-stabilized, robot-centered — the binning origin), `min_height: 0.15`, `max_height: 1.60` (covers 1.5 m obstacles; excludes floor), `angle_increment: 0.0058` (≈0.33°, ~1080 bins — finer than the old branch's 1° since Mid360 azimuth density supports it; tune), `range_min: 0.3`, `range_max: 5.0` (DPCBF `p_max`=3 m + margin), `use_inf: true`, `inf_epsilon: 1.0`.
- Satisfies H-5: the detector sees uniform angular increments regardless of the Livox rosette.
- Known limitation: a 10 Hz Mid360 frame yields a partial vertical rosette; the height band collapses it to a dense 2D ring — measured ~8 800 valid pts/frame in the old branch's sim (H-4), ample for 1080 bins.

### 9.5 Detection & tracking (stage 4) — `obstacle_detector_2` fork

- `obstacle_extractor`: input `/scan`; params from Appendix A (H-7); `transform_coordinates: true`, `frame_id: odom` → obstacles leave the extractor already in DPCBF's world frame. `circles_from_visibles: true`, `discard_converted_segments: true`.
- `obstacle_tracker`: `loop_rate: 20` (2× scan rate — compromise between upstream's 100 Hz timer model and measurement-driven correctness), `tracking_duration: 1.0`, KF variances per Appendix A.
- **Fork patches (kept minimal & upstreamable):**
  - **P-1 (certain):** parameter defaults + QoS (SensorData on `/scan` sub) + Humble build fixes if needed.
  - **P-2 (conditional, gated by §17 metrics):** measurement-driven tracker update (H-6) — predict-only between scans, correct on arrival, dt from header stamps.
  - **P-3 (conditional):** publish per-track covariance (enables σ-aware inflation, §7.3).

### 9.6 Safety gating & inflation (stage 5) — `safety_obstacle_filter` (custom)

Subscribes `/tracked_obstacles`, publishes `/obstacles_safe` (same msg type). Per circle:

1. **Gate:** drop if `now − header.stamp > max_age (0.30 s)`; drop if `true_radius > max_circle_radius (0.60 m)` (spurious wall arcs); clamp `r ← max(true_radius, min_radius 0.20 m)`.
2. **Inflate (H-8):** `radius_safe = r + fixed (0.03 m) + k_v·|v|·latency_horizon (0.12 s)` — the σ terms activate only if P-3 lands; until then the fixed term is tuned to cover measured error (Phase 4 calibration re-derives it the way the old branch derived k_σ=2.748).
3. **Speed sanity:** clamp `|v|` to `v_max_obstacle` (1.5 m/s; arena max 0.8) to stop KF spikes from inflating radii absurdly or aiming constraints wrongly.

Single responsibility: it never associates, never predicts, never talks to DPCBF.

### 9.7 Ground segmentation (stage 2.5, Phase 7 only)

`patchwork-plusplus` inserted between CropBox and projection when leaving the flat arena; publishes `/points_no_ground`; `pointcloud_to_laserscan` input remapped — no other change. Height band then narrows (`min_height` can drop toward 0.05) because the ground is removed upstream.

---

<a name="10-dpcbf-integration"></a>
## 10. DPCBF Integration Design

### 10.1 Principle

`dpcbf/` remains byte-for-byte unchanged (D3). The **only** new code that touches DPCBF types is `dpcbf_ros_adapter`, a plain C++ static library (with an optional rclcpp component wrapper) linked into whichever process calls `Filter()` — `simulate` today, `deploy` in Phase 6. The 1 kHz safety loop never crosses a process boundary: pub/sub latency is confined to the 10–20 Hz obstacle stream, which the adapter extrapolates.

### 10.2 Adapter API (design)

```cpp
class ObstacleSource {                 // dpcbf_ros_adapter
 public:
  enum class Mode { kOracle, kShadow, kEstimated };
  struct Config { Mode mode; double max_age_s = 0.30; double fade_out_s = 0.30;
                  std::string topic = "/obstacles_safe"; /* … */ };

  // Non-blocking, wait-free for the 1 kHz caller:
  // returns obstacles extrapolated to t_query (const-velocity), or empty+flag.
  Snapshot GetObstacles(double t_query_s);
  struct Snapshot {
    std::vector<dpcbf::ObstacleState> obstacles;
    bool fresh;           // false ⇒ staleness policy applies (§10.3)
    double age_s;         // now − newest header.stamp
  };
};
```

Implementation: rclcpp subscription on its own executor thread → converts `Obstacles`→`ObstacleState[]` → swaps into a double buffer (the old branch's wait-free PerceptionFrame pattern, H-item §4.2). `GetObstacles` reads the buffer and applies `p += v·(t_query − stamp)` per obstacle. No locks shared with the physics or filter threads beyond the atomic buffer swap.

### 10.3 Staleness & fail-safe policy

- `age ≤ max_age (0.30 s)`: extrapolated obstacles, normal operation.
- `max_age < age ≤ max_age + fade_out`: **conservative degrade** — commands ramp linearly toward zero velocity (implemented at the call site by scaling the desired command before `Filter()`), obstacles still fed (extrapolated).
- `age > max_age + fade_out`: **fail-safe stop** — desired command forced to zero; DPCBF still runs (obstacles empty would silently disable safety — instead the last known set, further inflated by `k_v·|v|·age`, is retained up to 1.0 s, then the vehicle is already stopped).
- Never mix oracle and estimated obstacles in one `Filter()` call (H-9). Mode is per-process, set at launch.

### 10.4 Modes

| Mode | Obstacles fed to `Filter()` | Perception stack | Purpose |
|---|---|---|---|
| `kOracle` (sim default) | `DynamicObstacleManager::Snapshot()` — the exact baseline behavior | may run, ignored | Regression anchor; must be bit-identical to baseline (§16.2-T1) |
| `kShadow` | oracle | runs; adapter logs `/obstacles_safe` vs oracle per query | Risk-free evaluation of the full ROS2 stack in closed loop |
| `kEstimated` | `/obstacles_safe` via adapter | runs | Production path (sim first, then hardware) |

### 10.5 Threading at the call site (sim)

The `axis_filter` lambda (1 kHz, bridge thread) becomes:

```cpp
auto snap = obstacle_source.GetObstacles(d->time);   // wait-free
ApplyStalenessPolicy(desired, snap);                  // §10.3
const auto filtered = safety_filter.Filter(robot, desired, snap.obstacles);
```

`Filter()` stays on the single bridge thread (thread-safety fact §2.3-4 preserved). The pre-existing unlocked `mjData` read (§2.3) is unchanged in scope — not worsened, and flagged for an upstream fix (R-9).

---

<a name="11-simulation-architecture"></a>
## 11. Simulation Architecture

### 11.1 Design constraints

Physics, SDK2 bridge, and the 1 kHz loop stay exactly as audited (§2.5). MuJoCo-LiDAR is Python; the physics loop is C++; raycasting 24 000 rays must not run under `sim.mtx` in the physics thread (H-3 timing: ~5–9 ms — would blow the 2 ms physics budget).

### 11.2 Chosen structure: mirror-state sidecar

- **`simulate` (C++), optional ROS2 module** (`-DUNITREE_MUJOCO_WITH_ROS2=ON`): publishes `/clock` (from `d->time`, ≥100 Hz), `/sim/mj_state` (qpos + mocap_pos/quat + sim_time; ~8 KB @ 100 Hz — trivial), `/sim/gt_obstacles` (from `Snapshot()`, 50 Hz). Publishing happens in the bridge thread after its existing state reads; total added work µs-scale.
- **`sim_mjlidar_bridge` (Python, system Python 3.10 + rclpy):** loads the *same* `scene_g1.xml`, subscribes `/sim/mj_state`, writes `qpos/mocap_pos`, runs `mj_kinematics` (no dynamics — poses only), then `MjLidarWrapper.trace_rays()` with `LivoxGenerator("mid360")` at 10 Hz on the freshest state; publishes `/livox/lidar` (stamped with sim time), GT `/odom`, TF `odom→base_link`. Runs `use_sim_time=true`.
- Determinism note: the sidecar samples state asynchronously (one-frame-old poses possible — the old branch measured 4.7 mm effect, H-item §4.2). Accepted; recorded in §17.3 as a known sim-fidelity bound.

### 11.3 Scene changes (`scene_g1.xml`, Phase 1)

1. `<site name="mid360_link" pos="0.0002835 0.00003 0.428434" euler="3.14159265 -0.000892 0"/>` under `torso_link` (H-1), matching `g1_description` (asserted by test T7). **[Corrected 2026-08-01, Phase 1]** MJCF `euler` is intrinsic-xyz while URDF `rpy` is extrinsic-xyz; at roll=π these do not commute, so H-1's URDF pitch **+0.000892** becomes MJCF pitch **−0.000892** (verified: matrix-equal to 0.000000°, whereas the previously listed `+0.000892` differed from the URDF source by 0.102°). T7 compares rotation matrices, not raw numbers, for exactly this reason.
2. `group="2"` on `head_link` geoms (H-4) so the raycaster's `geomgroup` mask excludes the head shell. (MuJoCo-LiDAR exposes geom filtering; if its current API can't mask groups, that's a ~5-line sidecar-side patch — verify in Phase 1.)

### 11.4 Raycast correctness gate (Phase 1 acceptance)

Reproduce the old branch's wall test (8 m wall, rays at grazing incidence) through MuJoCo-LiDAR's CPU backend. Pass ⇔ zero through-wall rays. On failure: switch `backend="warp"`/`"taichi"` (independent BVH) or patch CPU backend to `mj_ray` per-ray (H-3). This gate is mandatory before any detection work — silent missed occlusions would poison every downstream evaluation.

### 11.5 Interface identity guarantee

The perception launch file (`perception.launch.py`) has **no sim/hw branches**. A separate `source_sim.launch.py` starts sidecar+simulate module; `source_hw.launch.py` starts driver+DLIO. Both produce the §7.1 contract. `ros2 launch g1_perception_bringup bringup.launch.py source:=sim|hw` is the only switch (D4).

---

<a name="12-deployment-architecture"></a>
## 12. Deployment Architecture (Real G1)

### 12.1 Process placement (G1 onboard PC, Ubuntu 22.04 / Humble)

| Process | Contents | CPU budget target |
|---|---|---|
| `livox_ros_driver2` | driver | <10% one core |
| DLIO | odometry | 1 core |
| perception container | CropBox + p2l + extractor + tracker + safety filter (composed, intra-process) | <1 core total (§17.4 benchmark) |
| `deploy` binary | FSM + policy + DPCBF + adapter | unchanged + ~2% |
| foxglove_bridge / rosbag2 | observability (optional) | best-effort |

### 12.2 DDS coexistence (the load-bearing deployment fact)

Unitree SDK2 uses CycloneDDS (0.10.x) with ROS2-compatible `rt/…` topic naming — with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the **same domain id**, SDK2 topics appear as ROS2 topics (`/lowstate`, …) and coexistence is the officially supported unitree_ros2 pattern. Configuration:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0            # match simulate/config.yaml & SDK2
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml   # pin NetworkInterface (eth0 on G1, lo for sim-only)
```

Risks: cyclonedds version skew between the SDK2-vendored lib and Humble's `rmw_cyclonedds_cpp` inside **one process** (relevant for `simulate` and `deploy`, which link both `unitree_sdk2` and rclcpp) — mitigation R-3: build those binaries against a single CycloneDDS (unitree_ros2's documented approach: compile cyclonedds 0.10.2 into the workspace and point both at it). Phase 0 proves this with a link-and-echo smoke test before anything else depends on it.

### 12.3 Odometry on hardware

DLIO primary (§5.7). Bringup order: driver → DLIO → TF sanity (robot stationary: `odom→base_link` drift <1 cm/min) → perception. FAST_LIO_LOCALIZATION_HUMANOID is the drop-in alternate (needs driver `xfer_format:1` dual-publish; perception unaffected per §7.2).

### 12.4 DPCBF in `deploy` (Phase 6)

`State_RLBase` (or the FSM layer above it) currently converts joystick input to the policy's velocity command. Insert the identical seam as sim: `ObstacleSource::GetObstacles` + staleness policy + `Filter()` before the command reaches the policy observation. Robot state for `Filter()`: `{x, y, phi}` from `/odom` (adapter subscribes; same extrapolation machinery), `{v_sag, v_lat}` from odometry twist rotated to body frame. The deploy binary gains rclcpp guarded by `-DDEPLOY_WITH_ROS2=ON` and links the adapter — no other deploy change.

---

<a name="13-directory-structure"></a>
## 13. Directory Structure and Build System

```
unitree_rl_mjlab_/
├── dpcbf/                          # FROZEN (D3)
├── simulate/
│   └── src/ros2_bridge.{h,cc}      # NEW, ~150 LOC, -DUNITREE_MUJOCO_WITH_ROS2
├── deploy/                         # Phase 6: -DDEPLOY_WITH_ROS2 + adapter link
├── src/, scripts/                  # training — untouched
├── ros2/                           # NEW: colcon workspace root
│   ├── deps.repos                  # vcstool pins for external sources
│   ├── src/
│   │   ├── g1_perception/
│   │   │   ├── g1_description/          # xacro; ament_cmake
│   │   │   ├── g1_perception_bringup/   # launch + params + rviz/foxglove
│   │   │   ├── g1_perception_utils/     # footprint pub, marker relay
│   │   │   ├── sim_msgs/                # MjState.msg (sim-only interface)
│   │   │   ├── sim_mjlidar_bridge/      # ament_python; imports mujoco_lidar
│   │   │   ├── safety_obstacle_filter/  # ament_cmake, composable
│   │   │   └── dpcbf_ros_adapter/       # ament_cmake lib + gtest
│   │   └── external/                    # vcs import target (gitignored)
│   │       ├── obstacle_detector_2/     # OUR FORK, pinned SHA
│   │       ├── MuJoCo-LiDAR/            # upstream, pinned SHA (pip -e or PYTHONPATH)
│   │       ├── direct_lidar_inertial_odometry/  # feature/ros2, pinned (hw phases)
│   │       └── patchwork-plusplus/      # pinned (Phase 7)
│   └── README.md                        # env hygiene: system python, RMW, domain
└── DPCBF_Perception_Subsystem_ROS2_Architecture.md   # this document
```

Build flow: `vcs import ros2/src/external < ros2/deps.repos` → `colcon build` in `ros2/` → `simulate`/`deploy` build via their existing CMake, with the ROS2 options ON locating rclcpp via `find_package` and the adapter via its install/export. Pinning policy: every external repo referenced **by SHA** in `deps.repos`; fork lives under the project's GitHub org; upstreamable patches PR'd upstream but never depended on until merged and re-pinned.

---

<a name="14-launch-architecture"></a>
## 14. Launch Architecture and ROS2 Composition Strategy

### 14.1 Launch tree

```
bringup.launch.py  (args: source:=sim|hw, mode:=oracle|shadow|estimated,
                    ground_seg:=off|patchwork, viz:=off|rviz|foxglove, record:=off|on)
├── source_sim.launch.py      # sidecar (+ expects simulate running with ROS2 module)
│      └── sets use_sim_time:=true globally
├── source_hw.launch.py       # livox_ros_driver2 + DLIO
├── description.launch.py     # robot_state_publisher(g1_mid360.xacro) + footprint pub
├── perception.launch.py      # THE SHARED STACK — no sim/hw conditionals inside
│      └── ComposableNodeContainer "perception_container":
│            [CropBox] → [patchwork++ (arg)] → [pointcloud_to_laserscan]
│            → [obstacle_extractor] → [obstacle_tracker] → [safety_obstacle_filter]
│            use_intra_process_comms:=true
├── viz.launch.py             # rviz2 -d perception.rviz | foxglove_bridge + relay
└── record.launch.py          # rosbag2 MCAP: /livox/lidar /odom /tf* /scan
                              #   /raw_obstacles /tracked_obstacles /obstacles_safe
                              #   /sim/gt_obstacles /dpcbf/status
```

### 14.2 Composition rationale

All 10 Hz pipeline stages share one container with intra-process comms: zero serialization on the three PointCloud2 hops (the only large messages), single scheduler, one process to monitor. The driver, DLIO, and sidecar stay standalone (different lifecycles/languages; DLIO isn't componentized). `obstacle_detector_2` nodes are already `rclcpp_components`-style in the port — if not, componentization joins patch P-1.

### 14.3 Time

Sim: `/clock` from `simulate`, `use_sim_time=true` for every perception node — header stamps are sim time end-to-end, making rosbag replay and oracle comparison exact. Hardware: system time; Mid360 timestamping per driver config (PTP if the G1 network supports it; else driver host-time mode) — consistency between `/livox/lidar` stamps and DLIO's `/odom` stamps is what matters, and both derive from the same source.

### 14.4 Visualization ownership

RViz2 (dev machine) and Foxglove (over `foxglove_bridge` from the robot) consume only public topics + the marker relay. The OpenCV `dpcbf_visualizer` stays as-is for sim (it reads the filter result in-process); it gains nothing from ROS and is not ported.

---

<a name="15-dependency-graph"></a>
## 15. Dependency Graph

```
                       ┌────────────┐
                       │  MuJoCo    │ (vendored 3.3.6)
                       └─────┬──────┘
              ┌──────────────┼───────────────┐
        ┌─────▼─────┐  ┌─────▼──────┐  ┌─────▼─────────────┐
        │ simulate  │  │ mjlab (py) │  │ MuJoCo-LiDAR (py) │
        │ (+SDK2,   │  │ training   │  └─────┬─────────────┘
        │  rclcpp°) │  └────────────┘        │
        └───┬───┬───┘                  ┌─────▼─────────────┐
   rt/* DDS │   │ /sim/mj_state,/clock │ sim_mjlidar_bridge│
            │   └──────────────────────►  (rclpy, sim_msgs)│
            │                          └─────┬─────────────┘
            │                                │ /livox/lidar /odom TF        hardware:
            │                                ▼                              livox_ros_driver2 ─► DLIO
            │                    ┌───────────────────────────┐                    │        │
            │                    │  perception container     │◄───────────────────┘        │
            │                    │  pcl_ros → (patchwork++)  │◄──── TF ◄───────────────────┘
            │                    │  → p2l → obstacle_det_2*  │
            │                    │  → safety_obstacle_filter │
            │                    └────────────┬──────────────┘
            │                                 │ /obstacles_safe
        ┌───▼──────────┐               ┌──────▼──────────┐
        │ deploy       │◄──────────────│ dpcbf_ros_adapter│ (lib; also linked by simulate)
        │ (+rclcpp°)   │    link       └──────┬──────────┘
        └───┬──────────┘                      │ std::vector<ObstacleState>
            │                          ┌──────▼──────────┐
            └─────────────────────────►│ dpcbf (FROZEN)  │  ° = optional CMake flag
                                       └─────────────────┘  * = pinned fork
```

Rules enforced in review: `dpcbf` depends on nothing new; `dpcbf_ros_adapter` is the only package including both `rclcpp` and `dpcbf` headers; `sim_mjlidar_bridge` never imports dpcbf or detector code; perception packages never include MuJoCo or SDK2 headers.

---

<a name="16-testing-roadmap"></a>
## 16. Testing Roadmap

### 16.1 Test taxonomy

| Level | Framework | Runs where |
|---|---|---|
| Unit (adapter math, inflation, footprint TF) | gtest / pytest | every build |
| Node integration (per-stage, bag-driven) | `launch_testing` + recorded MCAP fixtures | every build |
| Pipeline regression (bag → `/obstacles_safe` compared to committed reference) | script + `ros2 bag` | every build |
| Closed-loop sim (oracle / shadow / estimated) | `simulate` + bringup, scripted scenarios | phase gates |
| Hardware smoke | checklists + recorded bags | Phase 5+ |

### 16.2 Named gate tests (referenced by the phases)

- **T1 Oracle equivalence:** `mode:=oracle` produces byte-identical filter I/O to baseline `f111cfa` on a seeded scenario (the old branch's technique, H-10). Protects the adapter refactor of the `axis_filter` lambda.
- **T2 Wall-occlusion gate:** §11.4. Zero through-wall rays.
- **T3 Pattern-envelope gate:** `mid360.npy` angles ⊆ datasheet FOV (360° az; −7…+52 el, sensor frame).
- **T4 Static-obstacle accuracy:** single still cylinder at 1/2/3 m in sim → `/obstacles_safe` center error ≤0.10 m, `true_radius` error ≤0.05 m (pre-inflation), detection latency ≤2 frames.
- **T5 Dynamic tracking:** cylinder at 0.5/0.8 m/s crossing → velocity RMSE <0.1 m/s after confirmation (H-11), no track-ID flicker >1 swap per 10 s.
- **T6 Staleness policy:** kill the perception container mid-run → adapter enters degrade→stop per §10.3 timings; auto-recovers on restart.
- **T7 Extrinsic guard:** MJCF site pose == xacro pose (parsed, compared in CI script).
- **T8 Replay determinism:** same bag through the perception container twice → identical `/obstacles_safe` streams (requires P-2 or fixed-rate tracker; measured, not assumed).
- **T9 TF availability:** launch-level check that `odom→base_footprint` resolves within 50 ms for every cloud stamp.
- **T10 DDS coexistence smoke:** one process linking SDK2 + rclcpp echoes both `rt/lowstate` and a ROS2 topic without crash/version conflict (gates everything, run in Phase 0).

---

<a name="17-evaluation-roadmap"></a>
## 17. Evaluation and Benchmarking Roadmap

### 17.1 Datasets

- **Sim bags:** seeded scenarios (S1 static field, S2 single crosser, S3 20-obstacle swarm, S4 occlusion corridor, S5 robot-walking sway) recorded with `/sim/gt_obstacles` — ground truth travels with the bag.
- **Hardware bags:** stationary robot + walked props (Phase 5), then walking robot (Phase 6+); GT from motion capture if available, else hand-measured static layouts (§20 Q-6).

### 17.2 Perception metrics (vs oracle/GT, matched by nearest-neighbor ≤0.5 m)

Position RMSE (target ≤0.05 m, cf. training noise model σ=0.03), velocity RMSE (≤0.1 m/s, H-11), radius bias (|bias| ≤0.05 m pre-inflation), containment rate (true circle ⊆ safety circle ≥99.9%, H-8), detection latency (≤2 frames), track continuity (ID swaps/min), false-positive rate inside `p_max` (≤0.1/min after gating), pipeline latency (cloud stamp → `/obstacles_safe` stamp, ≤60 ms p95).

### 17.3 Closed-loop safety metrics (sim, oracle vs estimated A/B on identical seeds)

Collision rate, min clearance distribution, command-tracking MSE (safety conservatism cost), fraction of time in degrade/stop states, DPCBF active-constraint statistics. Acceptance for Phase 4: estimated-mode collision rate == oracle-mode (zero) on S1–S4 at ≥0.95 of oracle's command-tracking performance. Known fidelity bounds documented with results: sidecar one-frame state lag, no per-ray deskew in sim (real Mid360 + walking introduces intra-frame motion the sim doesn't model — DLIO deskews for odometry, but the perception cloud is used raw; measured impact goes in the log).

### 17.4 Benchmarks

Per-stage CPU/latency on dev machine and on the G1 Orin NX (`ros2 topic hz/bw`, container CPU via cgroup stats, adapter query timing via its own histogram in `/dpcbf/status`). Budget: perception container <1 Orin core; adapter `GetObstacles` <10 µs p99 (it sits in the 1 kHz loop).

---

<a name="18-implementation-phases"></a>
## 18. Implementation Phases

Each phase ends by appending a §21 log entry. Gates in **bold**.

- **Phase 0 — Workspace & coexistence skeleton.** Create `ros2/` workspace, `deps.repos` (pinned SHAs), install Humble deps (`rmw_cyclonedds_cpp`, `pointcloud_to_laserscan`, …), `g1_description` + `robot_state_publisher` + `base_footprint_publisher`, fork `obstacle_detector_2` and build it, `sim_msgs`. **Gate: T10 (DDS coexistence), T7 (extrinsic guard), all packages build with `colcon build`.**
- **Phase 1 — Simulated sensor source.** `simulate` ROS2 module (`/clock`, `/sim/mj_state`, `/sim/gt_obstacles`); scene edits (§11.3); `sim_mjlidar_bridge` publishing `/livox/lidar`, `/odom`, TF. **Gate: T2 (wall occlusion), T3 (pattern envelope), RViz shows a sane cloud while the G1 walks under policy control.**
- **Phase 2 — Projection chain.** CropBox + `pointcloud_to_laserscan` config; `/scan` validated against known scene geometry (wall at measured distance ±2 cm). **Gate: T9; `/scan` at 10 Hz with <20% dropped frames under full sim load.**
- **Phase 3 — Detection & tracking.** Fork params (Appendix A), extractor/tracker in the container, marker relay, bag fixtures recorded. **Gate: T4, T5 on scenarios S1–S2; T8 measured (decides whether P-2 is needed now).**
- **Phase 4 — Safety filter, adapter, closed loop in sim.** `safety_obstacle_filter`; `dpcbf_ros_adapter`; refactor `axis_filter` seam behind `ObstacleSource`; modes oracle/shadow/estimated. **Gate: T1 (oracle equivalence — mandatory before merging the seam refactor), T6, then §17.3 A/B acceptance on S1–S4.**
- **Phase 5 — Hardware sensor bring-up (robot stationary).** Driver + DLIO + same perception launch on the G1; extrinsic verification against walls/props; CropBox self-filter tuning with real self-hit data; hardware bags recorded. **Gate: T4-hardware (static props, ±0.10 m), DLIO stationary drift <1 cm/min, container CPU within §17.4 budget.**
- **Phase 6 — DPCBF on hardware.** Adapter into `deploy` (§12.4); walking robot among static props with safety filter active (speed-limited); then a single moving prop. **Gate: staleness drills (T6 on hardware), zero contacts across scripted trials, operator e-stop rehearsed.**
- **Phase 7 — Rough terrain & extensions.** patchwork++ insertion; parameter re-tuning outdoors; optional P-3 covariance inflation; Jazzy migration assessment. **Gate: §17.2 metrics maintained off-flat-ground.**

Explicit non-goals of all phases: no policy-observation perception (training contract untouched), no global mapping/localization, no Nav2.

---

<a name="19-risk-analysis"></a>
## 19. Risk Analysis

| # | Risk | L×S | Mitigation / trigger |
|---|---|---|---|
| R-1 | MuJoCo-LiDAR CPU backend inherits `mj_multiRay` occlusion misses (H-3) | H×H | Phase-1 gate T2; fallback = warp/taichi backend or per-ray patch; last resort = C++ `mj_ray` node reusing `mid360.npy` (~500 LOC, design in §5.1) |
| R-2 | `obstacle_detector_2` unmaintained / tracker timer model (H-6) noisy velocities | M×H | Fork+pin; conditional patches P-2/P-3; reserve `nav2_dynamic` swap behind the `/obstacles_safe` boundary |
| R-3 | CycloneDDS version conflict inside `simulate`/`deploy` (SDK2 lib vs rmw) | M×H | Phase-0 gate T10; unify on one cyclonedds build (unitree_ros2 pattern) |
| R-4 | 10 Hz perception too stale for 0.8 m/s obstacles (DPCBF audit's staleness finding) | M×H | Adapter extrapolation + inflation `k_v·|v|·latency` + §10.3 degrade policy; measured in §17.3; if insufficient: tracker rate ↑ (P-2 enables), `alpha` ↓, or `p_max` ↑ |
| R-5 | Sparse ring after height-band slicing on hardware (rosette ≠ sim uniform assumptions) | M×M | T4-hardware with real props; widen band / accumulate 2 frames in p2l input (small relay) if needed |
| R-6 | Walking-induced cloud distortion (no deskew on the perception path) | M×M | Detection at ≤3 m with 0.1 s frames bounds error ≈ v_robot×0.1 s ≈ 0.08 m — inside inflation budget; measure on hardware bags; escalate to LIO-deskewed cloud output if DLIO exposes it |
| R-7 | Python sidecar jitter → late `/livox/lidar` frames | L×M | Stamps come from sim state, not wall clock; consumers are stamp-driven; monitor hz in T-tests |
| R-8 | Fixed `base_link→torso_link` TF wrong under waist motion | M×M | Bounded by waist ROM; upgrade path via joint-state TF (§8.3) without downstream change |
| R-9 | Pre-existing unlocked `mjData` read in bridge thread | M×M | Not worsened by this design; upstream fix proposed separately; sidecar reads only the published state topic (no new race) |
| R-10 | G1 onboard CPU budget exceeded | L×M | §17.4 benchmarks at Phase 5 entry; VoxelGrid + tracker-rate knobs held in reserve |
| R-11 | Conda Python (3.12) shadowing Humble's 3.10 breaks rclpy/sidecar | M×L | Bringup README env hygiene; launch scripts pin `/usr/bin/python3` |
| R-12 | Humble EOL May 2027 | C×L | All chosen packages except the fork have Jazzy releases; migration checkpoint scheduled Phase 7 |
| R-13 | Mid360 pattern asset inaccurate (FOV metadata discrepancy) | L×M | Gate T3; backup pattern source: livox_laser_simulation CSVs |

(L/M/H/C = low/med/high/certain; S = severity.)

---

<a name="20-open-questions"></a>
## 20. Open Questions

- **Q-1** Which exact G1 variant/onboard PC is in this lab (EDU Orin NX vs Ultimate), and is its Mid360 the factory head mount matching H-1? → verify at Phase 5 entry; extrinsics re-measured if the mount differs.
- **Q-2** Will σ-aware inflation (P-3 covariance export) be needed, or does the fixed-margin calibration meet §17.2 containment? → decided by Phase 4 data.
- **Q-3** Does `mid360.npy` reproduce the real rosette closely enough for detection-density parity (T3 checks envelope, not density)? → compare sim vs hardware `/scan` bin-occupancy in Phase 5.
- **Q-4** DLIO vs FAST-LIO-humanoid on the G1 while walking (drift, CPU, robustness)? → bake-off during Phase 5 with both installed; decision recorded in the log.
- **Q-5** Are Unitree's onboard odometry topics (`rt/odommodestate`-class) usable as a cross-check or fallback odom source on G1? Semantics/drift unverified.
- **Q-6** Ground-truth source for hardware evaluation: is a motion-capture volume available, or do we accept surveyed static layouts + hand-timed prop motion?
- **Q-7** Should the trained navigation policy eventually consume perception-derived obstacle observations (closing the sim-to-real gap on the policy side, not just the safety side)? Out of scope here; flagged because the observation contract (§2.7) was designed for exactly the noise this pipeline will produce.
- **Q-8** Head-aperture ground truth (which part of the real head shell occludes the Mid360's lower rays): affects sim fidelity of near-body returns (old branch §7.1 assumption). Resolve with a hardware near-field scan in Phase 5.

---

<a name="21-final-recommendations"></a>
## 21. Final Recommendations

1. **Adopt this architecture as scoped:** 8 reused packages, 1 minimal fork, 4 small custom components, DPCBF frozen. The custom surface shrinks ~20× versus the abandoned branch while gaining hardware parity.
2. **Run Phase 0 and Phase 1 gates before believing anything else:** DDS coexistence (T10) and raycast correctness (T2) are the two facts the whole design leans on; both have concrete fallback plans if they fail.
3. **Never bypass the oracle ladder:** oracle → shadow → estimated, with T1 equivalence protecting the seam refactor. This is the cheapest insurance the old branch bought and proved.
4. **Keep the fork honest:** every `obstacle_detector_2` patch either goes upstream or stays in the pinned fork with a rationale in the log; no drift-by-convenience.
5. **Treat staleness as the first-class safety parameter:** the DPCBF audit showed the filter silently degrades with stale ground truth; §10.3's degrade→stop policy plus inflation is the answer — tune it with data, don't disable it.
6. **Document every measured number in §22's log** — the old branch's habit of recording measurements (H-1…H-11) is what made this restart cheap. Preserve it.

---

<a name="appendix-a"></a>
## Appendix A — Validated Constants Carried Forward

Single reference table for Phase 0 parameter files (provenance: `dpcbf_perception` branch, §4.2).

```yaml
# g1_description / scene_g1.xml  (H-1)
mid360_extrinsics: {parent: torso_link, xyz: [0.0002835, 0.00003, 0.428434],
                    rpy: [3.14159265, 0.000892, 0.0]}   # roll=π: upside-down mount
# URDF-rpy semantics (extrinsic xyz). In MJCF (intrinsic xyz) the SAME physical
# rotation is euler="3.14159265 -0.000892 0" — pitch sign flips at roll=π.
# [Corrected 2026-08-01, Phase 1; T7 verifies matrix equality.]

# pointcloud_to_laserscan
projection: {target_frame: base_footprint, min_height: 0.15, max_height: 1.60,
             range_min: 0.3, range_max: 5.0, use_inf: true}

# obstacle_extractor  (H-7; upstream defaults except noted)
extractor: {min_group_points: 5, max_group_distance: 0.10, distance_proportion: 0.01745,
            max_split_distance: 0.20, max_merge_separation: 0.20, max_merge_spread: 0.20,
            max_circle_radius: 0.60, radius_enlargement: 0.17,   # re-tuned from 0.25 (short-arc bias)
            circles_from_visibles: true, discard_converted_segments: true,
            transform_coordinates: true, frame_id: odom}

# obstacle_tracker  (H-6/H-7; per-second noise densities if P-2 lands)
tracker: {loop_rate: 20.0, tracking_duration: 1.0,
          process_variance: 0.0001, process_rate_variance: 0.03, measurement_variance: 1.0}
          # association gate 0.30 m, radius-residual weight 0.3, confirm_hits 3 → P-2 scope

# safety_obstacle_filter  (H-8)
safety: {max_age: 0.30, min_radius: 0.20, fixed_inflation: 0.03,
         latency_horizon: 0.12, v_max_obstacle: 1.5,
         k_sigma: 2.748}   # active only with P-3 covariance export

# dpcbf_ros_adapter  (§10.3)
adapter: {topic: /obstacles_safe, max_age: 0.30, fade_out: 0.30, hold_after_stale: 1.0}

# dpcbf_config.yaml cross-checks (frozen, for reference): p_max 3.0, max constraints 10,
# r_rob 0.30, s 1.05, reference_control_frequency_hz 500
```

<a name="appendix-b"></a>
## Appendix B — References

- Repo internals: `dpcbf/include/dpcbf/dpcbf_safety_filter.h`, `dpcbf/src/dpcbf_safety_filter.cpp`, `dpcbf/src/dynamic_obstacles.cpp`, `simulate/src/main.cc`, `simulate/src/unitree_sdk2_bridge.h`, `simulate/src/physics_joystick.h`, `src/assets/robots/unitree_g1/xmls/scene_g1.xml`, branch `dpcbf_perception` (`DPCBF_Perception_Subsystem_Architecture.md`, `g1_mid360_extrinsic.md`, `g1_mid360_sensor_model.md`).
- External: TATP-233/MuJoCo-LiDAR · harmony-eu/obstacle_detector_2 (`humble-devel`) · tysik/obstacle_detector (Przybyła 2017) · ros-perception/pointcloud_to_laserscan · Livox-SDK/livox_ros_driver2 · vectr-ucla/direct_lidar_inertial_odometry (`feature/ros2`) · deepglint/FAST_LIO_LOCALIZATION_HUMANOID · url-kaist/patchwork-plusplus · ros-navigation/navigation2_dynamic · unitreerobotics/unitree_ros2 (DDS coexistence pattern) · jkk-research/pointcloud_to_grid (evaluated, rejected) · rst-tu-dortmund/costmap_converter (evaluated, rejected).

---

<a name="implementation-progress-log"></a>
## Implementation Progress Log — PERMANENT, APPEND-ONLY

> **Rules.** This section is never deleted or rewritten. After every implementation phase (Phase 0, 1, …), append one entry using the template below, newest last. Corrections to an old entry are made by appending an addendum, not by editing history. Every measured number produced by the project lands here.

### Entry template

```markdown
### <date> — Phase <N>: <title>
- **Objective:**
- **Repository reconciliation findings:** (drift between this document and the tree discovered while working; doc sections updated)
- **Design decisions:** (with rationale; deviations from §18 called out)
- **Files added:**
- **Files modified:**
- **Validation methodology:**
- **Validation commands:**
- **Build commands:**
- **Test commands:**
- **Benchmark commands:**
- **Results:**
- **Performance numbers:**
- **Issues discovered:**
- **Root causes:**
- **Fixes applied:**
- **Remaining limitations:**
- **Lessons learned:**
- **Open follow-up items:**
```

### 2026-07-31 — Phase −1: Architecture research and master document (this document)

- **Objective:** restart the perception subsystem from baseline `f111cfa`; replace the from-scratch roadmap with a ROS2-package-reuse architecture; produce this document.
- **Repository reconciliation findings:** baseline verified to contain zero perception/ROS2 code; DPCBF filter invocation rate corrected from an initial ~50 Hz estimate to **1 kHz** (bridge `RecurrentThread` → `joystick->update()` → `axis_filter`, `simulate/src/unitree_sdk2_bridge.h:172-179`, `physics_joystick.h:56`); pre-existing unlocked `mjData` read in the bridge thread documented (R-9).
- **Design decisions:** D1–D6 (§1.3); package verdicts (§5); custom surface limited to 4 components (§6.3).
- **Files added:** `DPCBF_Perception_Subsystem_ROS2_Architecture.md`.
- **Files modified:** none (no implementation performed, per scope).
- **Validation methodology:** three source-grounded repository audits (DPCBF package, simulator/deploy, `dpcbf_perception` branch), three external-ecosystem investigations (MuJoCo-LiDAR; obstacle detection/tracking packages; driver/odometry/segmentation/distro landscape), plus direct verification of the filter call chain and of `obstacle_detector` message/parameter semantics against upstream documentation.
- **Results:** architecture approved for Phase 0; validated constants consolidated (Appendix A); risk register seeded (§19).
- **Issues discovered:** MuJoCo-LiDAR CPU backend rests on `mj_multiRay`, which the old branch measured missing up to 23.7% of occlusions on large flat geoms → made a Phase-1 gate (T2) rather than an assumption; `obstacle_detector_2` tracker inherits the timer-driven KF model the old branch proved incorrect → conditional patch P-2; dev machine NVIDIA driver currently broken (CPU raycasting unaffected); conda Python 3.12 shadows Humble's 3.10 (R-11).
- **Remaining limitations:** all §20 open questions; no gate test has been executed yet — every load-bearing external claim (DDS coexistence, raycast correctness) is scheduled for empirical verification in Phases 0–1 before dependent work begins.
- **Open follow-up items:** create work branch `dpcbf_perception_ros2` from `f111cfa`; execute Phase 0 (§18); create the `obstacle_detector_2` fork under the project org and pin SHAs in `deps.repos`.

### 2026-08-01 — Phase 0: Workspace & coexistence skeleton

- **Objective:** create the `ros2/` colcon workspace with pinned externals, the four skeleton packages (`g1_description`, `sim_msgs`, `g1_perception_utils`, `g1_perception_bringup`), and prove the two load-bearing facts: DDS coexistence (T10) and the extrinsic guard machinery (T7).
- **Repository reconciliation findings:**
  - **The dev machine has never built simulate or deploy.** `/opt/unitree_robotics` does not exist; `unitree_sdk2`, the C++ `unitree_dds_wrapper` and CycloneDDS were never installed; there are no build directories. The old branch's committed `simulate/build_off/` dependency manifests show the machine that did build it used `/home/kwan/.local/unitree_robotics` — a user-prefix install on a different machine. §2.6's "system-wide `/opt/unitree_robotics`" described the convention, not this machine.
  - **`sudo` requires a password** (non-interactive session ⇒ apt unusable). All packages the doc assumed as apt binaries are instead built from pinned source in the workspace: `cyclonedds` 0.10.2, `rmw_cyclonedds_cpp` (humble), `pointcloud_to_laserscan` (humble). `ros-humble-rosbag2-storage-mcap` and `foxglove_bridge` remain uninstallable → bags are sqlite3 for now.
  - **The C++ `unitree_dds_wrapper` no longer exists upstream.** `unitreerobotics/unitree_dds_wrapper` is python-only; the C++ tree lived in the Agnel-Wang fork and was deleted by commit `f20eb69 "delete cpp"`. Pinned the last-cpp commit `9dc107d2c163539ff48aa98454ca037815da821e`. Even that commit lacks the simulator/deploy-facing API this repo's code was written against (publisher `LowState::joystick`, `WirelessController`, `SportModeState`, public `SubscriptionBase::mutex_`, `KeyBase` with `pressed_time`, `LowCmd::check_mode_machine`) — the original build machine ran a locally-modified copy that was never pushed. Reconstructed as a recorded patch (`ros2/patches/0001-unitree-dds-wrapper-restore-sim-joystick-api.patch`, applied by `ros2/setup_external.sh`), byte-layout-faithful to the subscription-side parsing that *is* upstream.
  - §3 environment facts re-verified: Humble sourced; `rmw_cyclonedds_cpp`/`pointcloud_to_laserscan` missing (as documented); conda Python 3.12 shadows system 3.10 (R-11); NVIDIA driver broken (`nvidia-smi` fails); armadillo present (obstacle_detector dependency). New: **no `/dev/input/js0`** and `use_joystick: 1` with no CLI override means the stock simulator exits at startup on this machine — worked around at test time with an uncommitted shadow run-tree (copied binary + `use_joystick: 0` config), never by editing tracked config.
  - T7 ordering (§18 inconsistency flagged in the phase prompt): the comparison script was implemented in Phase 0 and became fully enforceable the moment the Phase-1 scene edit landed in this same bundle; it is wired into CTest (`g1_description` test `t7_extrinsic_guard`).
- **Design decisions:**
  - `colcon build --merge-install`: one prefix for the SDK2 stack and ROS packages, so simulate/deploy resolve everything through a single `CMAKE_PREFIX_PATH` entry and exactly one `libddsc.so.0` / one `libddscxx.so` exist at runtime — R-3's "unify on one CycloneDDS build" by construction. Build staged (`cyclonedds` → `unitree_sdk2` → rest) so the winner of same-name install collisions is deterministic. SDK2's vendored CycloneDDS is exactly 0.10.2 (== source pin), so either binary is ABI-safe.
  - `cyclonedds-cxx` source build skipped (COLCON_IGNORE): SDK2 vendors a prebuilt ddscxx 0.10.2 and nothing else consumes the C++ DDS API directly.
  - New helper package `unitree_dds_wrapper_vendor` installs the header-only C++ wrapper into the prefix (colcon-native, no manual copying). New test package `t10_dds_coexistence` (the T10 gate binary) — a §13 addition, justified by T10 being a named gate.
  - Fork policy: `obstacle_detector_2` (and the wrapper) are pinned checkouts under gitignored `src/external/`; org forks not yet created (follow-up). No obstacle_detector patches needed in this bundle — Appendix-A parameters live in `g1_perception_bringup/config/`.
- **Files added:** `ros2/deps.repos` (all SHAs below), `ros2/setup_external.sh`, `ros2/patches/0001-…`, `ros2/README.md`, `ros2/.gitignore`, packages `sim_msgs` (MjState.msg), `g1_description` (H-1 xacro + T7 script), `g1_perception_utils` (base_footprint_publisher, obstacles_marker_relay, footprint gtest), `g1_perception_bringup` (§14.1 launch tree + Appendix-A YAMLs + rviz layout), `unitree_dds_wrapper_vendor`, `t10_dds_coexistence`.
- **Files modified:** none outside `ros2/` in Phase 0.
- **Pinned SHAs (resolved 2026-08-01):** cyclonedds `9995905b` (0.10.2); cyclonedds-cxx `2a372d2c` (0.10.2, not built); unitree_sdk2 `21d0a3b2` (main); unitree_dds_wrapper `9dc107d2` (Agnel-Wang, pre-"delete cpp"); rmw_cyclonedds `fa8831b9` (humble); pointcloud_to_laserscan `59bf996f` (humble); obstacle_detector_2 `6ff7ac48` (humble-devel); MuJoCo-LiDAR `287b5012` (main); reserved: DLIO `c8acc371` (feature/ros2), patchwork-plusplus `3e6903a1`, rosbag2_storage_mcap `c1c21596`.
- **Validation methodology / commands:** `colcon build --merge-install` (12 packages); `colcon test` + `colcon test-result`; T10 run against the live simulator with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 CYCLONEDDS_URI=<lo config>`; `/proc/self/maps` audit inside T10.
- **Results — T10 (the headline):**
  1. Naïve order (SDK2 factory first, then rclcpp) **fails hard**: `rmw_create_node: failed to create domain, error Precondition Not Met` — rmw_cyclonedds explicitly `dds_create_domain()`s and aborts if the domain exists.
  2. Reversed order with SDK2 `Init(domain, "lo")` **also fails**: SDK2's ddscxx `DomainWrap` explicit-creates too (`PreconditionNotMetError`).
  3. **Working recipe (now the documented pattern): ROS2 initializes first; SDK2 `ChannelFactory::Init(domain, "")` joins the existing domain; the interface is pinned for both stacks via a shared `CYCLONEDDS_URI`** (plus `MaxAutoParticipantIndex` raised to 120 — cyclone's ~10 default is exhausted by the node fleet + CLI probes on one host). T10 PASS: SDK2 self-loopback 496 msgs, **`rt/lowstate` from the live simulator 9340 msgs/10 s ≈ 934 Hz**, ROS2 loopback 496 msgs, rmw = cyclonedds, and the process maps exactly one `libddsc.so.0.10.2` and one `libddscxx.so` (both from the merged prefix). SDK2 `rt/*` topics are visible in `ros2 topic list` (`/lowstate`, `/wirelesscontroller`, …) exactly per §12.2.
- **Issues discovered / fixes:** `std::atomic` streaming ambiguity against ddscxx operator overloads (fixed with `.load()`); participant-index exhaustion (config fix above); `--sdk2-first` flag retained in t10 to reproduce failure mode 1 for documentation.
- **Remaining limitations / follow-ups:** org forks + patch upstreaming for `obstacle_detector_2` and the wrapper; apt installs when sudo is available (`rmw` binaries optional now, `rosbag2-storage-mcap`, `foxglove-bridge`); `rosdep install` never run (deps satisfied manually); the init-order requirement must be honored by every future SDK2+rclcpp process (encoded in simulate's Phase-1 module and in `ros2/README.md`).

### 2026-08-01 — Phase 1: Simulated Mid360 sensor source

- **Objective:** simulate's optional ROS2 module (`/clock`, `/sim/mj_state`, `/sim/gt_obstacles` + mirror-model dump), the H-1/H-4 scene edits, the `sim_mjlidar_bridge` sidecar, and gates T2/T3/T7 + the visual check.
- **Repository reconciliation findings (doc corrections made in the body):**
  - **§11.3 / Appendix A euler correction:** MJCF `euler` is intrinsic-xyz, URDF `rpy` extrinsic-xyz; at roll=π they don't commute. H-1's URDF `rpy (π, +0.000892, 0)` is MJCF `euler (π, −0.000892, 0)`; the doc's original `+0.000892` MJCF line was off by 0.102° from its own source. Verified numerically (0.000000° with the sign flip); T7 compares rotation matrices, not literals, and would have caught the original at 1.8e-3 rad.
  - **The mirror cannot load `scene_g1.xml`** (§11.2 as written): the 90 obstacle mocap bodies enter the model via `AddToSpec()` at runtime and are absent from the scene file. Resolution: `LoadModel` (main.cc) dumps the **compiled** spec via `mj_saveXML` to `/tmp/unitree_mujoco_mirror_model.xml` (env-overridable `UNITREE_MUJOCO_MIRROR_XML`); the sidecar loads that. Two MuJoCo API facts cost time: `mj_saveXML` returns **0 on success**, and it keeps `meshdir` relative — the module rewrites `meshdir`/`texturedir` absolute post-save. Mirror verified: nq 36, nmocap 90, nsite 4.
  - `obstacle_detector/CircleObstacle` **does have identity/semantics fields** (`uid`, `semclass`, `confidence`) — §5.3/§7.3's "no ID field" is wrong for the ROS2 port; `/sim/gt_obstacles` fills `uid`, and the Phase-4 adapter can map `uid → ObstacleState::id`.
  - MuJoCo-LiDAR's CPU backend takes a `geomgroup` mask natively — the §11.3 "sidecar-side patch" for H-4 was unnecessary.
- **Design decisions:** ROS2 publishing reuses the otherwise-idle `UnitreeSdk2BridgeThread` loop (1 kHz `SpinOnce`, decimation driven by **sim time**, so rates are realtime-factor-invariant and pause-correct); mjData read lock-free — same pre-existing benign-race class as the SDK2 bridge's own reads (R-9 not worsened, no locks added). The simulate module applies the T10 recipe: constructs the ROS2 node **before** `ChannelFactory::Init(domain, "")` and derives a process-wide `CYCLONEDDS_URI` (interface from config.yaml, participant cap 120) when unset. Sidecar: the 24 000-ray raycast (~12 ms) runs on a worker thread paced by sim time; the rclpy executor keeps the 100 Hz odom/TF path unblocked (a single-threaded first cut starved /odom to 26 Hz).
- **Files added:** `simulate/src/ros2_bridge.h`, `simulate/src/ros2_bridge.cc`; packages `sim_mjlidar_bridge` (node + `mirror.py` + tests + `test_gates/{t2,t3}`), bringup launch/test additions (`test_bringup_sim.launch_test.py`); `ros2/evidence/phase1/cloud_vs_gt_topdown.png`; `ros2/test_fixtures/` (gitignored bag + README).
- **Files modified outside `ros2/` (complete list, per the constraint):** `simulate/CMakeLists.txt` (option `UNITREE_MUJOCO_WITH_ROS2`, default OFF), `simulate/src/main.cc` (guarded include; mirror dump in `LoadModel`; ROS2-first init + empty-interface factory join; idle loop → `SpinOnce`), `src/assets/robots/unitree_g1/xmls/scene_g1.xml` (mid360_link site with corrected euler; `group="2"` on both head_link geoms). `dpcbf/`, `deploy/`, `src/` (code), `scripts/` untouched. With the option OFF, simulate builds byte-equivalent to baseline behavior (verified by building and running both variants).
- **Gate results:**
  - **T2 wall occlusion: PASS — zero through-wall rays** across 17 608 analytically-expected hits in four configurations (8×3 m wall @ cutoff 100; 40×6 m @ 100 and @ 40; pathological 200×10 m @ 40), max range error 0.0 mm, per-frame raycast 1.3–4.1 ms on the bare wall scene. Root cause of the non-reproduction of H-3: the sidecar raycasts through **python mujoco 3.6.0**, where the `mj_multiRay` AABB-pruning miss (measured on 3.3.6) is evidently fixed. The vendored C-side 3.3.6 is not on the raycast path. Version-skew note: sim compiles the model with 3.3.6, mirror with 3.6.0 — same MJCF, poses agree (mirror test + T7); pin recorded as a watch item.
  - **T3 pattern envelope: PASS with measured discrepancy (Q-3 quantified):** 800 000-ray `mid360.npy`, azimuth encoded [0°, 360°] (full ring), elevation **−7.212°…+52.164°** vs datasheet −7°…+52° — 0.218% of rays overshoot by ≤0.22°. Gate tightened to its purpose: hard-fail beyond ±0.25°, ≥99% strict-envelope required (measured 99.78%). Physically negligible (≤3 cm at 8 m); Phase-5 bin-occupancy comparison stays the adjudicator.
  - **T7 extrinsic guard: PASS at 0.0e+00 m / 0.0e+00 rad** (matrix comparison), plus base→torso nominal chain check at 0.0e+00; in CTest.
  - **T9-style TF check:** `odom→base_footprint` resolves, z = 0, pure yaw (roll = pitch = −0.000); `odom→mid360_link` consistent with robot pose.
  - **launch_testing smoke: PASS live** (`test_topics_and_rates`, 15.4 s): `/livox/lidar` within [8, 12.5] Hz and `/odom` ≥ 50 Hz asserted against the running simulator; SKIPs cleanly when no simulator is up.
- **Performance numbers (dev machine, mesa software GL, sim ≈ 0.8–1.0× realtime):** raycast **11.6 ms mean / 13.4 ms p95** per 24 000-ray frame (2.08 Mrays/s) on the full G1+90-obstacle scene (bare-wall scene: 1.3–4 ms; an earlier 47 ms reading was build-load contention); `/livox/lidar` **9.95 Hz** (σ 5.6 ms), 10 700–20 200 valid pts/frame (vs H-4's ~8 800 reference — plausible: full scene incl. floor returns; head-shell mask active, no origin splat); `/odom` **99.2 Hz** (max gap 16 ms); `/clock` 166–202 Hz; `/sim/mj_state` 83–90 Hz; `/sim/gt_obstacles` 45.6–47 Hz (all sim-time-decimated, wall rates scale with RT factor); bridge-thread health ON vs OFF: `rt/lowstate` **903.5 Hz vs 922.3 Hz** (≤2%, within run-to-run noise — §11.2's µs-budget holds); sidecar CPU **32% of one core**; `/sim/mj_state` ≈ 5.3 KB/msg.
- **Evidence:** `ros2/evidence/phase1/cloud_vs_gt_topdown.png` — cloud projected to odom via the mirror: **23/27 GT obstacles within 6 m appear as point clusters on their GT circles** (the 4 low-count cases all at 5.1–6 m, 4–11 pts — range fade, not misses), occlusion shadows visible behind every obstacle, 58% of points in the ground band (H-2's downward sweep), side view shows the ground plane + 1.5 m obstacle columns. Reference bag `ros2/test_fixtures/s1_static_reference/` (sqlite3, 27.87 s, 92.9 MiB, md5 `d372a619c563e0b4953247dafdfc51a0`): 278 `/livox/lidar`, 2522 `/odom` + `/sim/mj_state`, 1322 `/sim/gt_obstacles`, `/tf`, `/tf_static`, `/clock`.
- **Issues discovered / root causes / fixes:** executor starvation (fix: worker thread, above); `mj_saveXML` return-value and meshdir behaviors (fixes above); GLFW cannot create a GL context on the broken NVIDIA driver — **run simulate with `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa`** (documented in README); deploy's `g1_ctrl` needed three more wrapper-API restorations (`KeyBase` + `pressed_time`, `check_mode_machine`) — it now **builds and its FSM runs** (Passive→FixStand transition observed via DSL over the sim's lowstate), which de-risks Phase 6 early.
- **Remaining limitations:**
  - **Walking-under-policy visual evidence not achieved on this machine:** no gamepad (`/dev/input/js*` empty), `/dev/uinput` root-only, no xdotool, GNOME blocks unattended screenshots, and FixStand cannot catch the uncontrolled post-spawn collapse (tested; robot reaches z≈0.06 before the ramp completes even at 0.5 s). The visual check was performed with the robot suspended upright by the sim's elastic band (sensor at 1.84 m) and the 90-obstacle field moving — every sensor-pipeline property the check targets (ground sweep, no self-hit splat, obstacle visibility, occlusion) is demonstrated. Follow-up: rerun with a gamepad attached (or interactively) for the walking variant; RViz2 runs under software GL (window verified) but the screenshot needs an interactive session.
  - Sidecar one-frame state lag accepted per §11.2/§17.3; bag is sqlite3 not MCAP; sim RT factor with software GL ≈0.8–1.0.
- **Lessons learned:** initialization order between rmw_cyclonedds and any other explicit CycloneDDS domain creator is load-bearing and asymmetric (both sides hard-fail as creators; exactly one may create); convention traps (MJCF euler, `mj_saveXML` retcode) are exactly what matrix-level guards like T7 are for; sim-time-based decimation makes every published rate honest under slow-motion and pause.
- **Open follow-up items:** org forks + upstream PRs (wrapper joystick API; none needed yet for obstacle_detector_2); MCAP + foxglove via apt when sudo available; investigate FixStand spawn-catch (or a paused-start option) so headless walking evidence becomes reproducible; consider pinning python `mujoco==3.3.6` to match the vendored C runtime (currently 3.6.0, where T2 passes — re-run T2 if the pin changes); Phase 2 (projection chain) is unblocked.
