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

Official driver, ROS2 Humble/Jazzy, requires Livox-SDK2. `xfer_format`: 0 = PointCloud2 (`PointXYZRTLT`: x,y,z,intensity,tag,line,per-point timestamp), 1 = `CustomMsg` (FAST-LIO's required input), 2 = PointCloud2 (plain XYZI). Publishes `/livox/lidar` and `/livox/imu` (built-in 6-axis IMU, ~200 Hz). Config `MID360_config.json`: lidar/host IPs, ports, extrinsics, timestamp mode. **Contract consequence:** the perception stack must depend only on `x,y,z` (+ optionally intensity) fields so that both sim (plain XYZ[I]) and any hardware `xfer_format` satisfy it (§7.2). ~~The driver can dual-publish CustomMsg for FAST-LIO without affecting the perception contract.~~

> **[Corrected 2026-08-01, Phase 5A — verified against the pinned checkout, tag 1.2.6 `13eb05e4`.]** Two statements above are wrong for the ROS2 build. (i) **`xfer_format` 2 does not work in ROS2**: `kPclPxyziMsg` is the ROS1 `pcl::PointCloud<PointXYZI>` path — `CreatePublisher` has that branch `#if 0`'d out and `PublishPclMsg()` prints *"pcl::PointCloud is not supported in ROS2"* and returns. **0 is the only usable value.** (ii) **The driver cannot dual-publish**: `transfer_format_` selects one publisher for one topic, so PointCloud2 and CustomMsg are mutually exclusive. The Q-4 FAST-LIO bake-off is therefore a *separate capture session with the driver reconfigured*, during which the perception stack has no cloud — not a co-run. Also verified: the ROS2 `frame_id` parameter default is `frame_default` (the example launch files say `livox_frame`), never `mid360_link`; `InitImuMsg` hardcoded `livox_frame` for `/livox/imu` regardless of the parameter (fixed by recorded patch 0005); publisher QoS is **Reliable depth 256** and not parameterised; and the driver is componentisable (`livox_ros::DriverNode`). Full audit: `ros2/doc/phase5a_seam_audit.md`.

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
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz | `mid360_link` | SensorData (hw: driver publishes Reliable) | sidecar (**never implemented — this topic has no sim counterpart**; bench tests synthesise it, Phase 5A) / driver (`frame_id` was hardcoded upstream; patch 0005) |
| `/odom` | `nav_msgs/Odometry` | ≥50 Hz | `odom`→`base_link` | Reliable, depth 10 | sidecar (GT) / DLIO |
| TF `odom→base_link` | tf2 | sim: with `/odom` (100 Hz); **hw: scan rate ~10 Hz** [corrected 2026-08-01, Phase 5A — DLIO broadcasts TF from its per-scan `publishToROS()` thread, not from the 100 Hz `publishPose()` timer] | — | tf default | sidecar / DLIO |
| TF static (robot) | tf2_static | latched | — | — | `robot_state_publisher` (both) |
| TF `odom→base_footprint` | tf2 | sim: 100 Hz; **hw: inherits the row above (~10 Hz)** [corrected 2026-08-01, Phase 5A] | — | — | `base_footprint_publisher` (both) |
| `/points_self_filtered` | PointCloud2 | 10 Hz | `mid360_link` | SensorData | CropBox component |
| `/points_no_ground` | PointCloud2 | 10 Hz | `mid360_link` | SensorData | patchwork++ (Phase 7 only) |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `base_footprint` | SensorData | `pointcloud_to_laserscan` |
| `/raw_obstacles` | `obstacle_detector/Obstacles` | 10 Hz | `odom` | Reliable, depth 5 | `obstacle_extractor` |
| `/tracked_obstacles` | `obstacle_detector/Obstacles` | 10 Hz (= scan rate; measurement-driven since P-2) [corrected 2026-08-01, Phase 3 — was "10–100 Hz (tracker `loop_rate`)"] | `odom` | Reliable, depth 5 | `obstacle_tracker` |
| `/obstacles_safe` | `obstacle_detector/Obstacles` | = tracker rate | `odom` | Reliable, depth 1 (latest wins) | `safety_obstacle_filter` |
| `/sim/mj_state` | custom `sim_msgs/MjState` (qpos[], mocap_pos[], mocap_quat[], sim_time) | 100 Hz | — | BestEffort depth 1 | simulate (sim only) |
| `/sim/gt_obstacles` | `obstacle_detector/Obstacles` | 50 Hz | `odom` | Reliable depth 1 | simulate (sim only; oracle/eval) |
| `/clock` | `rosgraph_msgs/Clock` | ≥100 Hz | — | Clock QoS | simulate (sim only) |
| `/dpcbf/status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | — | Reliable | adapter (both; mode, staleness, fallback state) |

### 7.2 The PointCloud2 contract (the sim/hardware seam)

Perception consumes **only** `x, y, z` (float32) and tolerates any extra fields. This makes all three hardware `xfer_format` variants and the sim sidecar output interchangeable. Per-point timestamps/ring/tag are *not* part of the perception contract (they remain available to LIO, which is a separate consumer). `frame_id` must be `mid360_link` in both worlds.

> **[Amended 2026-08-01, Phase 5A.]** "All three `xfer_format` variants" is really one (see the §5.6 correction) — but the *principle* held under test: the driver's actual record (7 fields, `point_step` **26** under `#pragma pack(1)`, with a FLOAT64 at the unaligned offset 18) traverses the entire unmodified perception chain, 277 `/obstacles_safe` out of 278 clouds, versus the sim sidecar's 3 fields / `point_step` 12. Gated by `test_hw_source_contract.launch_test.py`. QoS asymmetry recorded: the driver publishes **Reliable** while the sim sidecar publishes SensorData; the single contract survives because every subscriber in the stack is best-effort, which that test also asserts on the wire.

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

**Where the radius bound really is [corrected 2026-08-02, workstream C gap G2].** The 0.60 m above describes *this* node, but the binding cut happens two stages earlier and was measuring a different quantity. `obstacle_extractor::detectCircles()` adds `radius_enlargement` to the fitted radius **before** testing `max_circle_radius`, so with the shipped 0.60 / 0.17 it kept a circle only if `fit < 0.43 m`; the fit over-estimates a fully visible cylinder by `+0.084·r`, so the real limit was **≈ 0.397 m of true radius** — verified against 210 analytic-ray configurations, where the rule `keep ⟺ fit + 0.17 < 0.60` reproduces all 17 non-detections and all 43 detections with no exceptions. It was also *intermittent*: truncating the visible arc shortens the chord and brings the same object back under the cut, so an over-size obstacle appears and disappears as the robot moves (r = 0.50 m is invisible at full arc and visible at 60 % arc). Fork patch 0009 moves the gate onto the fit, so `max_circle_radius` now bounds the estimated obstacle radius as this section always claimed, and both drop paths are counted and throttle-logged. **Resulting sensing limit: true radius ≲ 0.55 m** (`0.60/1.084`, fully visible; partial occlusion only extends it). Objects above that — wide pillars, pallet stacks, vehicles, walls — are outside this pipeline's circle model by design, and DPCBF is told nothing about them. The filter's own `max_circle_radius` is now a near-redundant restatement of the same bound — it still catches the one thing the extractor cannot, a radius the *tracker's* KF has carried past the extractor's own cut (observed firing at `true_radius` 0.602 m) — so keep the two config values equal.
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

DLIO primary (§5.7). Bringup order: driver → DLIO → TF sanity (robot stationary: `odom→base_link` drift <1 cm/min) → perception. FAST_LIO_LOCALIZATION_HUMANOID is the drop-in alternate (needs driver `xfer_format:1` ~~dual-publish~~ **exclusively — the driver cannot dual-publish, so the alternate is a separate session, §5.6 correction**; perception unaffected per §7.2).

> **[Corrected 2026-08-01, Phase 5A — verified against `feature/ros2` @ `c8acc371`.]** Three DLIO facts this section assumed: (i) **DLIO has no TF listener.** `extrinsics/baselink2lidar` / `baselink2imu` in its config are the *only* definition of where `base_link` is; left at upstream's identity, `odom→base_link` is really `odom→mid360_link` (0.472 m and roll = π out). Our `dlio.yaml` derives both from `g1_mid360.xacro` and CTest `t7_hw_extrinsic_guard` re-derives them on every build. (ii) **DLIO broadcasts `base_link→frames/lidar` and `base_link→frames/imu` unconditionally**, so those names must be disjoint from the §8.2 tree or `mid360_link` gets two parents. (iii) **TF `odom→base_link` is published at scan rate (~10 Hz), not at `/odom`'s 100 Hz** — the broadcasts live in the per-scan `publishToROS()` thread, not the 100 Hz `publishPose()` timer. Measured cost on the bench: cloud→`/obstacles_safe` p95 rises from 1.76 ms (sim) to **13.19 ms**, still well inside the §17.2 60 ms budget, so the fix is held as conditional patch **P-5** pending the 5B/Orin numbers. Upstream also subscribes the cloud **Reliable** (never matches this architecture's best-effort publishers) and ships `use_sim_time: true`; both fixed by patch 0006 / our config.

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

Sim: `/clock` from `simulate`, `use_sim_time=true` for every perception node — header stamps are sim time end-to-end, making rosbag replay and oracle comparison exact. Hardware: system time; Mid360 timestamping ~~per driver config~~ (PTP if the G1 network supports it; else driver host-time mode) — consistency between `/livox/lidar` stamps and DLIO's `/odom` stamps is what matters, and both derive from the same source.

> **[Corrected 2026-08-01, Phase 5A.]** Timestamp mode is **not a driver configuration**: there is no PTP field in `MID360_config.json`. The driver reads the sync mode out of each packet header (`time_type`) and, when the LiDAR reports `kTimestampTypeNoSync`, stamps with the **host clock at packet reception** (`pub_handler.cpp::GetEthPacketTimestamp`). PTP/GPS is enabled on the *LiDAR*, out of band. Right in outcome, wrong in mechanism — and which mode is live is only observable at runtime, so it is a measurement in 5B block 1 (compare a cloud stamp against the host clock). Also: `bringup.launch.py` now defaults `use_sim_time` from `source` (true for sim, false for hw), which is the only sim/hw conditional outside the `source_*` launch files; DLIO's upstream `cfg/params.yaml` ships `use_sim_time: true` and is overridden in our config.

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
          # measurement_variance is R(0,0) and has units of m^2 (the KF's C = [1 0]
          # and its measurement is a circle centre in metres). P-2 seeds P(0,0) = R,
          # so the inherited 1.0 asserts a ONE-METRE 1-sigma LiDAR measurement and
          # makes the published sigma ~0.58 m. It is not a unitless knob and it is
          # not calibrated: derive it from data with
          # g1_perception_bringup/test/measure_measurement_variance.py before any
          # k_sigma is fitted [2026-08-02, gap G1].

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

### 2026-08-01 — Phase 2: Projection chain (CropBox + pointcloud_to_laserscan → `/scan`)

- **Objective:** the shared perception container's first two stages — `pcl_ros::CropBox` self-filter (§9.3) and `pointcloud_to_laserscan` (§9.4) under intra-process composition (§14.2) — validated by the measured-wall gate (±2 cm at 1/2/4 m), T9, the 10 Hz `/scan` rate gate under full sim load, and T8-groundwork replay determinism.
- **Repository reconciliation findings (this phase's "one waiting" turned out to be the headline):**
  - **The Humble binary `pcl_ros` (2.4.5) ships NO filter components at all.** `/opt/ros/humble` contains only `pcd_to_pointcloud` + `libpcl_ros_tf.a`; the `plugins/nodelet/*.xml` manifests are stale ROS1 leftovers with no compiled libraries behind them. §3's "pcl_ros binary-installed" is true but useless for §9.3 — the ROS2 CropBox port (perception_pcl PR #401) is present-but-commented-out of CMake on both the `humble` branch and the 2.5.x/iron line, and **first ships componentized in 2.6.1**. Resolution: pinned `perception_pcl` **2.6.1** (`9d078ebf`) in `deps.repos`; only `pcl_ros` is built (pcl_conversions stays the Humble binary, metapackage COLCON_IGNOREd); builds clean on Humble in 22 s (it still uses `.h` message_filters includes). **Componentization goal achieved — no standalone-node fallback needed**; `ros2 component` lists `pcl_ros::CropBox` + `pcl_ros::VoxelGrid` (+5 more filters).
  - **Upstream Filter output QoS violates §7.1:** `pcl_ros::Filter` publishes `output` Reliable. Recorded patch `ros2/patches/0002-pcl-ros-filter-output-sensor-data-qos.patch` (applied by `setup_external.sh`) makes it `SensorDataQoS`; verified on the wire (`/points_self_filtered` BEST_EFFORT/VOLATILE). Upstreamable — joins the org-fork/PR follow-up.
  - The source-built p2l **does** export `pointcloud_to_laserscan::PointCloudToLaserScanNode` (SensorData both ends) and subscribes **lazily**: `cloud_in` is only subscribed while `/scan` has ≥1 subscriber — every test/tool must hold a `/scan` subscription or the chain sits idle (bit us once; now documented in the tests).
  - **Fixture bag verified usable as the Phase-3 replay vehicle:** contains the complete TF tree (`odom→base_link` 2522, `odom→base_footprint` 2440, both statics) + recorded `/clock`; replaying it drives the container with zero helper nodes.
  - **Two live-path environment traps found and documented in `ros2/README.md`:** (1) simulate derives a `lo`-pinned `CYCLONEDDS_URI` internally — any process launched without the same URI binds the default NIC and sees simulate's topic *names* via discovery but **no data** (bag-replay-only sessions work either way, which is why Phases 0–2 tests never hit it until the live run); (2) SIGTERM on `ros2 launch` orphans the composable container — the stale `/perception_container` twin then races the next launch's LoadNode RPC and the new container stays empty (SIGINT + `pkill component_container` discipline in scripts).
  - **Pose-vs-band decision (suspension rig vs grounded):** the wall gate runs GROUNDED — zero-joint standing qpos with the lowest robot-AABB corner touching z=0 (pelvis z 0.8091 m, mid360 origin **1.2816 m** vs 1.2654 m H-1 nominal; AABB-corner grounding is conservative by ~1.6 cm — horizontal wall ranges are unaffected). No dynamics ever run (kinematic mirror + static published qpos), which sidesteps the no-gamepad/no-FixStand constraint entirely. S1-replay and live-load numbers remain in the Phase-1 suspension rig (sensor 1.84 m); each number below states its pose.
  - **Discovered issue (forward to Phase 3/5): wrist self-hits survive the Appendix-A CropBox.** In the grounded pose both `*_wrist_yaw_link`s return points at horizontal 0.29–0.36 m, z 0.87–0.94 m — 0.34–0.41 m below the sensor, outside the box's 0.25 m below-sensor coverage (the roll=π mount points the datasheet fan 52° *downward* in world), just past `range_min` 0.3 → they appear in `/scan` (visible in the evidence plot). H-4's head-shell mask never covered arms. Appendix-A values kept verbatim per the phase constraint; Phase-3 extractor tests must expect these returns, Phase-5 retunes the box with real self-hit data (likely `max_z` 0.25 → ~0.45).
- **Design decisions:** CropBox box coordinates applied in the cloud frame (`input_frame` empty ⇒ no TF in the filter — the cloud is already `mid360_link`, §9.3); container holds exactly [CropBox, p2l] with `use_intra_process_comms:=true` and a `voxel:=on` launch option wiring `pcl_ros::VoxelGrid` (leaf 0.05) between them — OFF by default per §9.3, no sim/hw conditionals (D4); **`angle_increment` 0.0058 KEPT** (Appendix-A latitude not used) — justification: measured points/occupied-bin at the gate targets is 8.7–20.6 (see Q-3 table), i.e. bins are saturated even at 4 m, and wall-window occupancy is 0.998–1.0, so finer binning has headroom and coarser is unnecessary; T9 probes count from the first successful resolution (TF published before the probe's DDS subscription matches is unobservable to it) and require zero misses thereafter.
- **Files added:** `ros2/patches/0002-…`, `g1_perception_bringup/config/cropbox_self_filter.yaml`, `test/test_wall_accuracy.launch_test.py`, `test/test_projection_replay.launch_test.py`, `test/wall_state_source.py` (mini-sim: /clock 200 Hz + static-qpos /sim/mj_state 100 Hz), `test/phase2_probe.py` (reusable §17.4 probe: rates/latency/T9/CropBox accounting/bin-occupancy/scan-hashing), `ros2/test_fixtures/wall_scene/{make_wall_scene.py,README.md}` (committed fixture; outputs regenerate to /tmp), `ros2/evidence/phase2/scan_vs_gt_topdown.png`.
- **Files modified (all inside `ros2/` — zero outside-`ros2/` edits confirmed by `git status`):** `deps.repos` (+perception_pcl pin + rationale), `setup_external.sh` (+COLCON_IGNOREs, +patch 0002), `perception.launch.py` (container populated), `g1_perception_bringup/CMakeLists.txt` (+2 launch tests), `ros2/README.md` (source-built list, gate table, 2 new runtime notes), `ros2/.gitignore` (wall_scene fixture negation, `__pycache__`), `test_fixtures/README.md` (+wall_scene entry).
- **Validation methodology:** measured-wall scene = scene_g1.xml + 3 surveyed walls (bearings 0°/90°/180°, faces at exactly 1.000/2.000/4.000 m from the base-footprint origin) + 3 cylinders r=0.15 (45°/135°/−45°, faces 1/2/3 m — the Q-3 baseline props), compiled via MjSpec with absolute meshdir (same fix simulate's dump needed); sidecar mirrors it directly — full production chain (rosette raycast → CropBox → p2l) with no simulate binary. Bag path: fixture replay through the container. Live path: simulate (shadow run-tree: copied `build_ros2` binary, `use_joystick 0`, `enable_elastic_band 1`, mesa software GL) + 90 moving obstacles + sidecar + container.
- **Gate results (all PASS):**
  - **Measured wall ±2 cm:** median `/scan` error **−0.04 / −0.07 / −0.15 mm** at 1/2/4 m (gate ±20 mm; 25 scans/target, grounded pose). Cylinders (reported, not gated): +0.95/+3.87/+8.24 mm at 1/2/3 m — small positive bias growing with range, consistent with tangent-point sampling of the rosette on a curved face.
  - **T9 (zero misses everywhere, warm-start discipline):** wall test 24/0; wall bench 299/0; replay test 277/0; **live full load 599/0**.
  - **`/scan` rate:** **live full sim load 600 frames / 59.9 s sim-time = 10.0 Hz, drop 0.0%** (wall-clock 9.998 Hz, RT≈1.0; suspension rig + 90 moving obstacles — walking-under-policy remains an open Phase-1 follow-up, not claimed); replay 277/278 = 9.96 Hz, 0.4% drop; wall run 300/300, 0.0%.
  - **Replay determinism (T8 groundwork): bitwise deterministic** — two container replays of the fixture bag produced 278/278 matching stamps with **278/278 identical md5(ranges)**. The projection stages will not poison Phase-3's T4/T5/T8.
  - `/scan` sanity (in CTest): frame `base_footprint`, uniform 0.0058 increment across all frames (H-5), monotonic stamps, empty bins +inf (`use_inf`), no NaN, finite ranges within [range_min, range_max].
- **Performance numbers (§17.4):** per-stage latency (probe reception deltas, stamp-matched): cloud→`/points_self_filtered` **p50 0.26 / p95 0.52 ms**, cloud→`/scan` **p50 0.66 / p95 1.12 ms** (live full load; wall run: 0.26/0.41 and 0.61/1.05). CPU during live run (ps avg): container **1.4%** of one core, sidecar 12%, simulate 372%. **Intra-process comms PROVEN, not asserted:** an LD_PRELOAD shim counting `rmw_publish` per topic inside the container (Humble `rmw_publisher_t.topic_name` at slot 2) shows — intra ON: `/scan` 278 publishes, `/points_self_filtered` **0** (all 278 filtered clouds delivered intra-process, zero DDS serializations); intra OFF control: both 278. (Shim crashes python/rclpy processes — apply to the container binary only.) CropBox removal (H-4 leaves arms/torso unmasked): live median **357** pts/frame (mean 409.8, max 571); replay median 356; wall run median 356.
- **Q-3 bin-occupancy baseline (grounded wall scene, 300 scans — Phase 5 compares the real rosette against exactly this table):** occupied-bin fraction / points-per-occupied-bin mean (p50, p95): wall_1m 1.0 / 20.37 (21, 27); wall_2m 1.0 / 13.77 (14, 22); wall_4m 0.9977 / 8.72 (9, 14); cyl_1m 1.0 / 20.56 (21, 28); cyl_2m 1.0 / 14.04 (14, 20); cyl_3m 1.0 / 10.17 (10, 16). Whole-scan occupied fraction: 0.359 (sparse wall scene), 0.706 mean (S1 live field).
- **Issues discovered / root causes / fixes:** live stack silent until the `CYCLONEDDS_URI` matched simulate's (root cause above; README note); determinism run 2 initially produced zero scans — orphaned container twin from run 1 captured the LoadNode RPC (fix: SIGINT + pkill between runs; README note); first T9 run showed 1 "miss" — probe-side DDS discovery artifact on the first cloud, not a system miss (fix: warm-start counting, documented in both tests); rmw shim segfaulted `ros2 launch` (python) — container-only preload.
- **Test counts:** **14 → 23** (0 errors, 0 failures, 1 skipped = Phase-1 live smoke when no simulator runs). New: wall gate (3 tests), replay integration (4 tests), both CTest-wired and SKIP-clean on bare machines.
- **Evidence:** `ros2/evidence/phase2/scan_vs_gt_topdown.png` — one grounded-pose `/scan` overlaid on surveyed GT: returns hug all six target faces, occlusion correct, wrist self-hit pair visible at ~0.32 m (the documented issue); probe JSONs regenerable via `test/phase2_probe.py`.
- **Remaining limitations:** walking-under-policy evidence still blocked (no gamepad/root — carried); bags still sqlite3 (MCAP blocked on apt); `ps`-averaged CPU is a lifetime mean, not instantaneous; the sidecar remains the rate bottleneck path (raycast ~12 ms) — container adds ≤1.1 ms p95.
- **Lessons learned:** "binary-installed" claims need a library-level audit, not a package-level one (pcl_ros existed; its components didn't); lazy subscriptions and interface pinning both produce *silent* no-data states that look identical from `ros2 topic list` — rate probes at every hop are the only honest smoke test; proving a launch-flag claim (intra-process) costs one 60-line shim and is worth it.
- **Open follow-up items:** carried — org forks + upstream PRs (now including the pcl_ros QoS patch 0002), MCAP/foxglove when sudo appears, walking evidence, python-mujoco pin decision (T2 validity still pinned to 3.6.0; the wall generator/mirror also run 3.6.0). New — **wrist self-hits**: Phase-3 extractor scenes must expect two near-range returns in the grounded pose (and moving ones once arms swing); Phase-5 CropBox retune owns the fix. **Forward note for Phase 3:** `CircleObstacle.uid` (discovered Phase 1) likely supersedes §7.3's index-as-id mapping — resolve deliberately in the Phase-3 prompt; `/sim/gt_obstacles` already fills `uid`.
- **Suggested next step:** **Phase 3 — detection & tracking** (all Phase-2 gates green): obstacle_detector_2 fork params live in the container (extractor + tracker per Appendix A), marker relay, T4/T5 on S1–S2 scenarios, T8 measured end-to-end (decides P-2 now), and the uid-vs-index decision above. The replay discipline (recorded /clock, warm-start T9, stamp-matched comparisons) and the deterministic projection front-end are in place.

### 2026-08-01 — Phase 3: Detection & tracking (`obstacle_extractor` + `obstacle_tracker` → `/tracked_obstacles`)

- **Objective:** the fork's extractor + tracker as components in `perception_container` (Appendix-A params), marker relay overlays, S1/S2 scenario fixtures, gates T4/T5, T8 measured end-to-end driving the P-2 decision, and the uid-vs-index resolution. Phase-2 work committed first as `08935bd`; this phase starts from a clean tree.
- **Repository reconciliation findings (the least-verified external held five, one of them load-bearing for T4):**
  - **Extractor TF lookup was `tf2::TimePointZero` (latest), not stamped** — §5.3's frame mechanism exists, but a latest-lookup is inconsistent with the pose at measurement time and nondeterministic under replay. P-1 makes it a stamped lookup (scan stamp, 0.1 s timeout — T9's guarantee makes the wait ~0).
  - **H-6 confirmed against this exact code, and it is worse than diagnosed:** `updateState()` re-applies the last stale measurement (predict **and** correct) on every timer tick; the timer is a **wall-clock** timer (ignores sim time), and output stamps are `now()`. Deterministic replay is structurally impossible as shipped, independent of `loop_rate` config.
  - **Not componentizable as shipped:** plain `main()` executables only; ROS1 nodelet code commented out. P-1 adds `rclcpp_components` wrappers (constructor signature changed from `shared_ptr<Node>` pair to `Node*` pair to avoid `shared_from_this`-in-constructor).
  - **`/scan` subscription QoS was default Reliable — incompatible with p2l's BestEffort SensorData publisher: the extractor receives NOTHING without the P-1 QoS fix.** Publishers set to Reliable depth 5 (§7.1). Wire-verified after patching (first replay smoke: 278/278 scans → raw msgs).
  - **NEW upstream bug (this phase's headline, found via T4):** the grouping loop starts with `input_points_.begin()++` — a no-op post-increment on a temporary — so the first point of every scan's **first group** is compared against itself and double-counted (`num_points` = span+1). `fitSegment` trusts `num_points` and reads **one point past the group**: in the S1 scene the 0.3 m wrist return entered the 3 m cylinder arc's algebraic line fit `Ax+By=1` (where a near point has ~10× leverage), rotating the line ~57° and compressing the chord 0.287→0.155 m → `true_radius` −62 mm (T4 gate is ±50). Found by scan-window measurement (input chord full at all ranges) → faithful python port (no divergence) → temporary debug prints in the node (`num_points=18 actual_span=17`, no split, corrupted fit len 0.155). Fixed in P-1 (loop starts at `std::next(begin())`); after the fix all three distances fit identically (+16…+20 mm, the √3-heuristic's expected small over-estimate).
- **uid-vs-index (D6 addendum, decided on source + measurement):** the tracker assigns `uid` from a monotonic counter once per track object; it is stable across frames for the track's lifetime, deliberately renewed on fission/fusion, and NOT preserved across track loss/re-acquisition — i.e., exact track-identity semantics. Measured: one uid per surveyed cylinder over 30 s (S1), one uid across the full crossing at both speeds (S2). **Decision: `uid` supersedes §7.3's index-as-id; the Phase-4 adapter maps `uid → ObstacleState::id`** (array indices permute when tracks are erased mid-vector — index-as-id would flicker). DPCBF treats id as informational (§2.3-6) either way.
- **Wrist phantom characterized, then removed by the sanctioned interim:** with the Appendix-A CropBox, the two wrist clusters (14 + 5 scan points at 0.28–0.34 m) each became circles and **merged into a single persistent phantom circle at 0.29 m, bearing ~0°, `true_radius` up to 0.42, `radius` 0.579** (just under `max_circle_radius` 0.60) — present in 294/300 raw frames, a permanent false obstacle inside `p_max` that would consume a DPCBF constraint slot at contact range from Phase 4 on. **Recorded sim-interim deviation** (config comment + this entry): CropBox xy ±0.35→±0.40, `max_z` 0.25→0.45 (covers wrist volume at 0.29–0.36 m horizontal / 0.34–0.41 m below sensor in the grounded pose). After: **zero circles within 0.7 m of origin and zero unmatched circles inside `p_max` across all S1/S2 runs.** Extractor params stayed Appendix-A verbatim (masking is the self-filter's job); the real retune remains Phase 5's with hardware self-hit data.
- **Scenario machinery (§17.1):** the Phase-2 mini-sim pattern extends exactly as hoped: `test_fixtures/scenarios/make_scenario_scene.py` (committed) builds scene_g1 + 4 mocap cylinders (one mirror model serves all scenarios; parked at 50 m); `test/scenario_state_source.py` publishes /clock 200 Hz + /sim/mj_state 100 Hz with **scripted mocap trajectories** + /sim/gt_obstacles 50 Hz (uid = mocap index, commanded velocity as GT). No gamepad, no dynamics, no simulate binary. **Scenario-fidelity lesson:** a first S2 cut parked the crosser in view for 3 s before moving — a velocity **step** onto a converged static track is a different, harder problem than T5's "crossing" and produced association losses at 0.8 m/s; the faithful scenario (enter already moving from outside `range_max`, so track birth is on a moving object) is what T4/T5's provenance measured. Recorded fixtures (sqlite3, gitignored, recipes in `test_fixtures/README.md`): `s1_surveyed` (75 MiB, md5 `64abe6da6688bd8b8a9b480294e6cb07`), `s2_cross_05` (53 MiB, `72b811a9f1c5211986fd50eaec098bb4`), `s2_cross_08` (35 MiB, `352f72adce964ffa2b84301e802ebd74`).
- **P-2 decision (measured per the §9.5/§18 rule, then fired):** with P-1 + config mitigation (`loop_rate` 20) on the fixtures: **T8 FAILED** — /raw_obstacles bitwise identical across two replays (the stamped-TF fix works) but /tracked_obstacles stamps were wall-derived and 600/600 disjoint between runs; **T5 FAILED** — velocity RMSE 0.102 (0.5 m/s) / 0.207 (0.8 m/s), dominated by a ~2 s convergence transient (the stale re-corrections double-weight old measurements). Both criteria failed → **P-2 implemented** (patch 0004): measurement-driven cycle (dt from header stamps with restart guard; predict-only for all tracks, associate, correct, prune faded, publish with the **measurement stamp**; wall timer deleted; promotion catch-up loops deleted), Appendix-A P-2-scope items (association gate 0.30 already default; **radius-residual weight 0.3** in the cost; covariances interpreted **per-step at `sensor_rate`, scaled linearly with actual dt** — recorded interpretation of the "per-second densities" note that preserves H-7 provenance at the nominal rate), plus two P-2 initialization designs: **two-point track initiation** (velocity seeded from the two measurements promotion already holds) and the **matching init covariance** P₀ = diag(R, 2R/dt²) (Bar-Shalom; upstream's arbitrary `P = I` made the filter ignore position innovations while a wrong velocity persisted — this single change took 0.8 m/s RMSE from 0.102 to 0.012). `confirm_hits` 3 (H-7) is applied as the **evaluation boundary** for T5, not a publication gate — gating publication at 3 hits would put T4's ≤2-frame latency out of reach; recorded as the reconciliation of those two Appendix-A numbers.
- **Gate results (all PASS; fixtures replayed through the full container):**

  | Gate | Result |
  |---|---|
  | **T4** center error | mean 40.9 / 38.4 / 40.1 mm at 1/2/3 m (max 41/46/51) — gate ≤100 mm |
  | **T4** `true_radius` error | +16.7 / +19.7 / +16.0 mm (pre-inflation, on `/tracked_obstacles`) — gate ≤50 mm |
  | **T4** detection latency | 1–2 scan frames (promotion needs 2 measurements) — gate ≤2 |
  | **T5** velocity RMSE | **0.009 (0.5 m/s), 0.012 (0.8 m/s)** after 3-hit confirmation — gate <0.1 |
  | **T5** ID swaps | **0 at both speeds** (single uid across each full crossing) — gate ≤1/10 s |
  | **T8** determinism | **bitwise identical** /raw_obstacles AND /tracked_obstacles across two replays (300/300 msgs) — now a **hard CTest gate** (`t8_replay_determinism`) |

- **§17.2 non-gating numbers (sim, this phase's honest snapshot):** position RMSE **0.041 m** on both S2 crossings (stretch target ≤0.05 ✓); radius bias +16…+20 mm (≤50 ✓); false-positive rate inside `p_max` **0/min** after the CropBox interim (it was ~1 persistent phantom before it — that is where the wrist finding shows up); track continuity: 0 swaps (S2), uids 0/1/2 stable for 30 s (S1); pipeline latency cloud→`/tracked_obstacles` **p50 1.14 / p95 1.71 ms** under full live load — the ≤60 ms §17.2 budget (to `/obstacles_safe`) has ~58 ms of headroom for Phase 4.
- **§17.4 benchmarks (live: simulate shadow-tree + 90 moving obstacles + sidecar + container, RT≈1.0, 60 s):** per-stage p50/p95: cloud→scan 0.48/0.95 ms, **scan→raw 0.36/0.52 ms (extractor), raw→tracked 0.27/0.43 ms (tracker)**, cloud→tracked 1.14/1.71 ms (max 3.0 ms). Rates: /scan 10.02 Hz (drop 0.17% = discovery edge), /raw 10.0 Hz, **/tracked_obstacles 10.0 Hz** (= scan rate; measurement-driven — §7.1 row corrected in the body from "10–100 Hz (tracker loop_rate)"). Track load on the 90-obstacle field: **mean 11.6 / p50 12 / p95 15 / max 22 simultaneous circle tracks** (+ ~9 segments), no dropped scans attributable to the new stages (600/601/600 cloud/scan/raw). CPU (instantaneous /proc deltas over 60 s): **container 4.0%** of one core with all four stages (Phase-2 two-stage: 1.4%), sidecar 28%, simulate 378%. S1 stability: exactly 3 tracks, no births/deaths over 30 s.
- **Patch series state:** `0003-obstacle-detector-p1-components-qos-stamped-tf.patch` (P-1: componentization + QoS + stamped TF + the `begin()++` grouping bug fix — all upstreamable) and `0004-obstacle-detector-p2-measurement-driven-tracker.patch` (P-2 as above — upstreamable as a rework proposal), both applied by `setup_external.sh` on the pinned `6ff7ac48`; no floating edits (fork checkout verified clean at pin + patches). P-3 (covariance export) untouched, still gated on Phase-4 containment data (Q-2).
- **Files added:** patches 0003/0004; `test_fixtures/scenarios/{make_scenario_scene.py}` (committed) + 3 gitignored bags; bringup `test/{scenario_state_source.py, tracking_dynamic_common.py, test_detection_static.launch_test.py, test_tracking_dynamic_05.launch_test.py, test_tracking_dynamic_08.launch_test.py, test_marker_relay.launch_test.py, test_t8_replay_determinism.py, t8_dump_probe.py}`; `evidence/phase3/gt_vs_tracked_overlay.png`.
- **Files modified (all inside `ros2/` — zero outside-`ros2/` edits, §21/§7.1-row append/correction excepted; confirmed by `git status`):** `perception.launch.py` (+extractor/tracker components in both container variants), `config/obstacle_detector.yaml` (+sensor_rate, +radius_residual_weight, +compensate_robot_velocity:false — input is odom-frame, compensation assumes robot-frame), `config/cropbox_self_filter.yaml` (recorded interim), `viz.launch.py` (3 relays: GT green / raw grey / tracked orange), `rviz/perception.rviz` (+2 MarkerArray displays), `obstacles_marker_relay.cpp` (+uid text labels, `show_ids` param), bringup `CMakeLists.txt` (+4 launch tests + T8 ctest), `setup_external.sh` (+patches 0003/0004), `deps.repos` (comment), `.gitignore` (+scenarios negation), `test_fixtures/README.md`, `ros2/README.md`.
- **Validation methodology:** every fork claim verified against the pinned checkout source before use; the T4 anomaly root-caused by measurement (scan-window probe → faithful python port → instrumented node) rather than parameter tuning; P-2 decision taken on recorded before/after numbers on the same fixtures; T4/T5/T8 wired as CTest with numeric assertions (SKIP-clean without bags); live-load numbers from the Phase-2 shadow-run-tree pattern (mesa GL, lo-pinned `CYCLONEDDS_URI`, SIGINT+pkill discipline).
- **Issues discovered / root causes / fixes:** the five fork findings above; launch_testing collects an imported base TestCase class (fixed: module-attribute import); a backgrounded benchmark script's deferred cleanup killed a later manual run's processes (fixed: foreground script; the orphaned-twin discipline caught the resulting duplicate sidecar via a 20 Hz /scan reading — rate probes remain the honest smoke test); simulate's shadow tree must mirror the repo layout (binary resolves `../config.yaml` and `../../dpcbf/...` relative to its directory).
- **Test counts:** **23 → 34** (colcon test-result --all: 0 errors, 0 failures, 1 skipped = Phase-1 live smoke without a simulator). New: T4 (3 asserts), T5×2 (3 each), marker relay (2), T8 hard CTest script (1, SKIP_RETURN_CODE 77 without fixtures).
- **Evidence:** `ros2/evidence/phase3/gt_vs_tracked_overlay.png` — left: S2 0.8 m/s tracked centers on the GT path with velocity arrows (post-exit coasting visible beyond range_max — correct predict-only fade behavior); right: S1 tracked circles hugging the three surveyed GT circles. RViz overlay config committed (`viz.launch.py` + perception.rviz); an interactive RViz screenshot remains blocked headless (GNOME; carried with the walking-evidence follow-up).
- **Remaining limitations:** walking-under-policy evidence still blocked (no gamepad/root); bags sqlite3 (MCAP blocked on apt); the extractor's circle-from-segment heuristic (radius = chord/√3) carries a systematic +16…+23 mm over-estimate on r=0.15 cylinders — inside the T4 gate, flagged for the Phase-5 hardware re-check; S2 velocity-step behavior (converged static track suddenly moving) converges in ~2–4 s and can swap ids — physical for this KF design, out of T5 scope, worth revisiting only if real obstacles start/stop abruptly inside p_max (the safety filter's `v_max` clamp and inflation bound the harm meanwhile).
- **Lessons learned:** "verify against the checkout, not the doc" keeps paying — all five fork findings were invisible from §5.3; a gate failure is a measurement tool: T4's −62 mm at exactly one range was the thread that unraveled an upstream one-character bug (`begin()++`); when a filter fails a convergence gate, check the **initialization covariance** before touching the tuned noise parameters — P₀ = diag(R, 2R/dt²) was worth 8× RMSE where no sanctioned knob would help; scenario fixtures must encode the *provenance* of their gate numbers (crossing ≠ velocity step).
- **Open follow-up items:** carried — org forks + upstream PRs (now 0002 pcl_ros QoS + 0003 + 0004), MCAP/foxglove when sudo appears, walking evidence + interactive RViz screenshot, python-mujoco pin decision, Phase-5 CropBox retune (interim ±0.40/0.45 is the starting point) and Phase-5 hardware re-check of the +16…+23 mm radius bias. New — none blocking.
- **Suggested next step:** **Phase 4 — safety filter + adapter + closed loop in sim** (all Phase-3 gates green): `safety_obstacle_filter` (§9.6), `dpcbf_ros_adapter` (§10) with `uid → id` per the D6 addendum, the `axis_filter` seam refactor behind `ObstacleSource` with **T1 oracle equivalence mandatory before the seam refactor merges**, T6 staleness drills, then the §17.3 A/B on S1–S4 — which needs **S3 (20-obstacle swarm) and S4 (occlusion corridor) fixtures**: extend `make_scenario_scene.py` (more mocap bodies + a wall body for S4); the scenario source already handles arbitrary trajectory lists. The 58 ms latency headroom and the deterministic front-end make the A/B comparisons clean; the wrist interim keeps `p_max` free of phantoms for the containment stats.

### 2026-08-01 — Phase 4: safety filter + dpcbf_ros_adapter + closed loop in sim (oracle → shadow → estimated)

- **Objective:** `safety_obstacle_filter` (§9.6), `dpcbf_ros_adapter` (§10.2–10.4), the `axis_filter` seam refactor behind `ObstacleSource` protected by T1, S3/S4 fixtures, the §9.6 containment calibration (Q-2), T6 drills, adapter micro-benchmark, shadow-mode deltas, and the §17.3 A/B at the fidelity §0 established. Two commits: seam refactor + T1 at `e853178` (Phase 4a); everything else in this commit (Phase 4b). **The oracle ladder was honored literally: T1 passed before the seam commit existed; shadow numbers existed before estimated mode was first switched on live.**
- **Repository reconciliation findings (§0):**
  - **The 1 kHz safety seam was DEAD CODE on this machine.** With `use_joystick: 0` (the only way simulate runs here — no gamepad), `UnitreeSDK2BridgeBase` never constructs a joystick, `lowstate->joystick` stays null, and `run()` skips `update()` — so `axis_filter`/`Filter()` never executed in any Phase 0–3 live session. Command injection is therefore a precondition for T1 itself, not merely for A/B fidelity. Resolution: a `ScriptedJoystick` (UnitreeJoystick subclass, ~60 LOC in main.cc) plays a sim-time axis profile ("t lx ly rx" breakpoints, piecewise-constant hold) through the exact `joystick->update()` path; it is installed from main.cc onto the bridge's **public** `lowstate`/`wireless_controller` members between construction and `start()` — `physics_joystick.h`/`unitree_sdk2_bridge.h` untouched, keeping the outside-`ros2/` budget to exactly main.cc + CMakeLists. Test-only flag: env `UNITREE_MUJOCO_SCRIPTED_COMMANDS`; inert when unset; tracked config untouched. Path (b) (`rt/wireless_controller` injection) was rejected on inspection: the filtered axes are *published to* that topic by the sim — external injection would bypass the seam entirely. Path (c) (walking under g1_ctrl) was not attempted beyond Phase-1's findings (bounded effort; buttons are not yet scriptable — noted as the missing piece for Phase-6 FSM driving).
  - **T1 methodology given wall-clock nondeterminism:** physics stepping and the bridge RecurrentThread are wall-clock paced, so two live runs are never byte-comparable. T1 is capture→replay (H-10): the pre-refactor tree (worktree @`9ca8a2e` + committed instrumentation patch that adds ONLY ScriptedJoystick + logging) captured every `Filter()` call (inputs, outputs, axes) on the seeded field under the committed profile; `t1_replay` then drives the refactored kOracle acquisition + the shared seam mapping + a fresh `DpcbfSafetyFilter` through the recorded input sequence and demands bit equality on everything. The seam's axes↔command code was moved verbatim into `dpcbf_ros_adapter/{axis_command_map,dpcbf_seam}.h` so production and harness compile the same translation unit — divergence there is what T1 exists to catch.
  - **Sim-time audit:** clean by construction — `GetObstacles(t_query)` takes the caller's `d->time`; ages are `t_query − header.stamp`, both sim time; no `now()` anywhere in the staleness path. §11.2 decision recorded: the adapter node runs `use_sim_time=false` — it must not consume the `/clock` its own process publishes; wall time paces only the 10 Hz diagnostics timer.
  - **uid → id:** `CircleObstacle.uid` is `uint64`, `ObstacleState::id` is 32-bit int. Mapping `id = uid & 0x7fffffff` (wrap into non-negative int31; `-1` stays reserved for "unset"; collision requires 2³¹ track births). Unit-tested at the edges.
  - **S4 geometry lesson:** the first S4 put the crosser on x=3.2 — outside `p_max` 3.0, where DPCBF culls it and occlusion containment has no safety meaning. Re-cut with the crosser at x=2.6 (occluded ~1.8 s > `tracking_duration` 1.0 s → track death + re-acquisition mid-scenario, deliberately the §10.3 worst case). Containment is correspondingly SCOPED to pairs whose GT is within `p_max` of the robot.
  - `pkill` trap for scripts: simulate's cmdline is `./unitree_mujoco`; a path-qualified kill pattern misses it and a stray 1 kHz sim keeps publishing (cost one shadow session; README note added).
- **Design decisions:** ObstacleSource mode fixed at construction, no setter, kOracle never subscribes, kEstimated never touches the oracle callback — H-9 is structural, plus an API-level test that topic data cannot surface in oracle mode. kOracle is a pass-through of an injected callback holding the *verbatim baseline conversion loop* (main.cc), keeping the adapter free of MuJoCo/manager types. Buffer: double-buffer with per-slot seqlock (single 10 Hz writer / single 1 kHz reader, reader wait-free, 2 s stress test). Staleness ladder per §10.3 with two recorded interpretations: position extrapolation capped at `max_age+fade_out` (retained set held at its 0.6 s point rather than chasing a stale velocity) and stop-regime inflation `|v|·min(age, 1.0)` (k_v=1); the set is retained indefinitely (inflation capped) — empty-on-stale forbidden; before any first frame: empty set + scale 0 (`kNoData` — robot stays stopped until perception is up). Mode selection via env `UNITREE_DPCBF_MODE`, **default oracle (D5)**; the non-ROS2 build keeps the baseline oracle path inline and refuses other modes. safety_obstacle_filter: measurement stamp passes through to `/obstacles_safe` (staleness accounting stays anchored to measurement time); segments not forwarded; v-clamp before inflation.
- **Files added:** packages `dpcbf_ros_adapter` (obstacle_buffer/source, axis_command_map, dpcbf_seam, filter_io_log; 18 gtests; tools/{t1_replay,ab_eval}.cc built by simulate CMake) and `safety_obstacle_filter` (composable node + gating.h; 8 gtests); bringup: `test/{phase4_obstacles_dump,phase4_containment,phase4_ab_export,phase4_ab_metrics,phase4_capture_stats,phase4_status_dump,phase4_latency_probe}.py`, `test/phase4_ab_run.sh` (one-command A/B), `test/phase4_live_session.sh` (live driver + T6 drill), `config/ab_profile.txt`; `test_fixtures/t1_baseline/{README,patch,profile}`; `evidence/phase4/*`; S3/S4 fixtures (gitignored bags, md5s in test_fixtures/README).
- **Files modified outside `ros2/` (complete list):** `simulate/src/main.cc` (seam refactor + ScriptedJoystick + capture hook), `simulate/CMakeLists.txt` (adapter include/link, t1_replay/ab_eval targets, T1 CTest). `dpcbf/` **byte-for-byte identical to f111cfa** (verified `git diff f111cfa -- dpcbf/` empty); `deploy/`, `scripts/`, `src/` untouched this phase.
- **Gate results:**

  | Gate | Result |
  |---|---|
  | **T1 oracle equivalence** | **PASS — 38 402 Filter() calls byte-identical** (commands, accelerations, constraint counts, solved flags, output axes; incl. OSQP kMaxIterations hold-last-feasible ticks). Seam commit `e853178`; capture md5 `e4a5caef830ce24cd05280467ac629a7`; CTest `t1_oracle_equivalence` |
  | **Adapter micro-benchmark** | **PASS — GetObstacles p99 ≤ 1 µs** (est. mode, live full stack, 111 145 queries @1 kHz; p99.9 ≤ 2 µs; 1 outlier >100 µs in 111 k; shadow mode p99 ≤ 10 µs incl. delta computation). Gate <10 µs |
  | **T6 staleness drill** | **PASS** — container SIGKILLed live: DEGRADE at age 0.3000 s (first query past the boundary), STOP at 0.6000 s, retained 11-obstacle set inflated and never emptied, auto-recovery to FRESH 0.03 s after the restarted container's first frame; drill repeated at shutdown (0.302/0.602). No orphan-twin race (KILLed container + fresh launch) |
  | **Containment ≥99.9 % (§9.6/Q-2)** | **S1 100 %, S2(0.5) 100 %, S2(0.8) 100 % at F=0** (max F_req −8.8/−67.6/−99.3 mm — Phase-3 steady-state errors leave margin to spare). **S3 85 %, S4 95 % at F=51 mm; the literal ≥99.9 % on S3/S4 is NOT achievable with any sane fixed term** (would need ~380 mm) — see Q-2 verdict below |
  | **§17.3 A/B (offline paired-filter, S1–S4)** | perf ratio est/oracle: **S1 0.9565 ✓, S2(0.5) 1.0000 ✓, S2(0.8) 0.9978 ✓, S4 0.9456 ✗ (marginal), S3 0.8235 ✗** (+206/30 000 est-arm OSQP maxIter ticks). Zero degrade/stop beyond startup in all replays |
  | **§17.3 A/B (live arena, suspension rig)** | est/oracle 0.686 (fresh-only 0.704); collision rate **not measurable** (no walking; robot base static/suspended — stated plainly, not substituted) |

- **Calibration (the Q-2 measurement, data in `evidence/phase4/containment_*.txt`):** `fixed_inflation` re-derived → **0.051 m** (recorded deviation from Appendix-A 0.03; config comment + this entry). Decomposition of required margins: (i) *steady-state converged tracks*: fully covered — the binding term is a **newly measured systematic: the extractor's circle-fit center bias grows with cylinder radius** (r_gt=0.30 blocker at 2 m: 83 mm center bias away from the sensor + 33 mm radius over-estimate → F_req max 51 mm; Phase 3's ≤41 mm was measured on r=0.15 only). Appendix-A's 0.03 was insufficient even for this steady case. (ii) *occlusion-coast and merged-arc transients* (S3 swarm mutual occlusion: 15 % of in-scope pairs; S4 crosser in the blocker's shadow: divergence to 0.38 m over ~0.5 s before track death): the tracker coasts **with fresh stamps**, so no age gate — adapter or filter — can see it. **Q-2 verdict: fixed-margin inflation suffices for visible converged tracks and fails structurally for coasting/merge transients; P-3 covariance export (k_σ·σ with KF covariance growing during predict-only coasting) is needed for the coast class; merged-arc corruption additionally needs an extractor-side fix or acceptance.** Sensitivity at +100 ms horizon: same shape (pooled p99.9 384 vs 364 mm).
- **Shadow-mode delta report (75 s live arena session, `evidence/phase4/microbenchmark_and_shadow.txt`):** 547 estimated frames compared at query time against oracle within `p_max`: matched 2.7/frame, oracle-unmatched 3.2/frame, est-unmatched 2.5/frame; matched pos err mean 0.167 m (capped NN at 0.5 m); complementary direct tracked→nearest-GT distribution **p50 94 mm / p90 186 mm / p99 549 mm** — 2–4× the fixture-replay errors. Attribution: 90-obstacle density (4.5× S3 → constant mutual occlusion/merge transients) **plus a rig artifact — the band-suspended robot swings ±0.45 m and yaws freely through ±π**, a sensor-platform motion no walking controller would produce. This is exactly what shadow mode is for: it priced the oracle→perception gap in closed loop *before* estimated mode ever drove a command, and says dense-field + spinning-sensor operation is not entitlement-grade yet, while S1/S2-class scenes are.
- **Performance numbers (§5):** GetObstacles histogram (est): ≤1 µs 110 540 / ≤2 µs 533 / ≤5 µs 66 / >10 µs 6 of 111 145. Bridge cadence **987 queries/sim-second in BOTH oracle and estimated live runs** (83 614 vs 84 189 records over ~85 s sim) — seam refactor cost indistinguishable; (`rt/lowstate` hz via ros2 CLI is unmeasurable — CLI lacks unitree_hg types; cadence above is the same loop). End-to-end cloud→`/obstacles_safe` **p50 1.28 / p95 1.76 / p99 2.19 ms** live under full load (budget ≤60 ms; Phase-3 headroom intact — the safety filter adds ~0.2 ms). Rates: cloud/scan/tracked/safe 601/600/600/600 in 60 s (10.0 Hz, zero drops). Container CPU **4.5 %** of one core with all Phase-4 stages (Phase 3: 4.0 %); sidecar 30 %. Capture logging (~4 KB/record @1 kHz oracle) did not measurably perturb cadence (986.5 vs 987.5 with 6× smaller records).
- **Test counts:** **34 → 65** colcon tests (0 errors, 0 failures, 1 skipped = Phase-1 live smoke without a simulator) **+ 1 simulate CTest (T1)**. New: adapter 18 (extrapolation, staleness boundaries, seqlock stress, uid edges, H-9 API, shadow accumulation, oracle bit-exactness), safety filter 8 (every §9.6 rule + boundary + passthrough).
- **Issues discovered / root causes / fixes:** dead seam (above; ScriptedJoystick); S4 outside `p_max` (re-cut); float-boundary test artifact (10.30−10.0 ≠ 0.30 in doubles — boundary asserted at exact-representable stamps); Reliable-QoS publish-before-discovery loses messages in tests (publish-until-received helper); stray-sim pkill pattern (above); SIGINT capture truncation (periodic flush every 256 records + truncation-tolerant replay tail).
- **Remaining limitations:** walking-under-policy closed loop still not achievable (no gamepad; g1_ctrl FSM needs scripted *buttons* — the ScriptedJoystick currently scripts axes only), so §17.3 collision-rate/min-clearance remain **deferred to walking evidence**; the live A/B rig (suspended, freely-yawing base) is harsher than the design point and its 0.686 ratio should not be read as the walking-robot number; S3/S4 containment residuals as per Q-2; bags still sqlite3; the offline A/B's estimated arm uses stamp+2 ms delivery (measured p95) rather than recorded arrival wall-times.
- **Lessons learned:** a safety seam that never executes is indistinguishable from a working one in every topic-level probe — only the capture instrument revealed that Filter() had never run live; capture→replay turns a wall-clock-nondeterministic system into a byte-exact regression (and proved OSQP determinism through its failure path for free); containment tails must be decomposed before calibrating — one number (p99.9) would have demanded a 380 mm margin that is really two distinct phenomena (a calibratable bias and an uncalibratable transient); scope safety metrics to the consumer's horizon (`p_max`) or they measure things the filter never eats.
- **Open follow-up items:** carried — org forks + upstream PRs (0002/0003/0004), MCAP/foxglove when sudo appears, walking evidence + interactive RViz screenshot, python-mujoco pin decision, Phase-5 CropBox retune + radius-bias hardware re-check (now with the NEW datum that the bias grows with radius: +33 mm at r=0.30). New — **P-3 covariance export** (fires on the Q-2 verdict; scope: tracker publishes per-track P, safety filter applies k_σ·σ, calibration re-run — proposed for the Phase-5 window or a 4b follow-on); **scripted buttons** for g1_ctrl FSM driving (Phase-6 groundwork); R-4 levers to consider *if* S3-class density is an actual deployment target: tracker rate ↑, `p_max` ↑, or accepting the measured residual; OSQP iteration budget under 10-constraint swarm load (oracle arm hit maxIter on 9.6 % of live-arena ticks — pre-existing, surfaced by the A/B instrumentation, worth a look before hardware).
- **Suggested next step:** **Phase 5 — hardware sensor bring-up** (driver + DLIO + the same perception launch on the G1, T4-hardware, extrinsic verification, CropBox retune with real self-hit data). All Phase-4 build/test gates are green; the two §17.3 sub-gates that failed (S3/S4 containment-to-99.9 % and S3 A/B ratio) are *measurement verdicts about swarm-density limits*, fully documented with the P-3 path proposed — they do not block hardware bring-up, whose scenarios (static props, single walked prop) are S1/S2-class where every gate passed with margin. Phase 5 needs: G1 access + Q-1 variant check (Orin NX? factory Mid360 mount?), props (cylinders r≈0.15–0.30 — include one ≥0.30 to check the radius-dependent bias on real data), Mid360 network config (`MID360_config.json` IPs, PTP availability), and a decision on landing P-3 before or after the hardware error data exists (recommended: after, so k_σ is calibrated on real covariances).

### 2026-08-01 — Phase 5A: hardware bring-up preparation (no robot; 5B not started)

- **Objective:** everything for Phase 5 that does not need the robot — pin and build the hardware stack (Livox-SDK2, `livox_ros_driver2`, DLIO), verify all three **from source** rather than from §5.6/§5.7, implement `source_hw.launch.py` + its configs, walk the §7.1 sim/hw seam row by row, gate the hardware cloud contract and the DLIO wiring on the bench, and write the 5B checklists. **5B did not run: no robot access and no Q-1 answers.** Per the phase scope this entry stops at the 5A gate and reports.
- **Q-1: UNANSWERED.** No operator input was available in this session, so nothing about the G1 variant, the onboard PC, the Mid360 mount, the robot's network, or sudo/apt on the robot has been verified. Every hardware-specific number in the shipped configs is therefore either derived from the xacro (extrinsics — safe) or an upstream placeholder that is *designed to fail loudly* (network — see `hw_config_check.py`). **Q-1 remains blocking for 5B.**
- **Topology decision: DEFERRED, because it is a function of Q-1.** Consequence recorded plainly: every latency and CPU number below is a **dev-machine (x86_64, 32 cores)** number. The §17.4 Orin budget has not been measured and cannot be until the topology is chosen. If 5B ends up running perception off-robot, "Orin benchmark" is a named follow-up, not a passed gate.
- **aarch64: BLOCKED on this machine, root-required.** No cross toolchain (`dpkg` foreign arch = i386 only), no `qemu-user-static`, and `binfmt_misc` has no aarch64 handler — registering one needs root. `podman` 3.4 and `buildah` exist and can *pull* `arm64v8/ubuntu:22.04`, but executing it fails with `Exec format error`. What was done instead is a static portability audit of the pinned sources: the only x86 intrinsics anywhere are rapidjson's `emmintrin.h` includes in Livox-SDK2 and the driver's vendored copies, both behind `RAPIDJSON_SSE2`/`SSE42` guards that nothing defines (rapidjson also carries a NEON path); `unitree_sdk2` already ships `lib/aarch64/libunitree_sdk2.a` and `thirdparty/lib/aarch64/libddsc*.so`; everything else is portable C++/CMake. Nothing found that *predicts* an aarch64 failure — but that is an audit, not a build, and it is the largest open risk carried into 5B.

- **Repository reconciliation findings (verified against the pinned checkouts, never the READMEs — the fork lesson generalised, and it paid again):**
  - **`livox_ros_driver2`, `xfer_format` 2 does not exist in ROS2.** `kPclPxyziMsg` is the ROS1 `pcl::PointCloud<PointXYZI>` path: `CreatePublisher`'s branch for it is `#if 0`'d out and `PublishPclMsg()` prints *"not supported in ROS2"* and returns. §5.6/§7.2's "0 or 2 for perception" → **0 is the only working value**. Doc corrected in the body.
  - **The driver cannot dual-publish.** `transfer_format_` selects one publisher for one topic; PointCloud2 and CustomMsg are mutually exclusive. §5.6's "can dual-publish CustomMsg for FAST-LIO without affecting the perception contract" is false, and it changes Q-4's shape: **the FAST-LIO bake-off is a separate capture session with the driver reconfigured, during which the perception stack has no cloud.** §5.6/§12.3 corrected.
  - **`/livox/imu` `frame_id` was hardcoded** to `livox_frame` in `InitImuMsg`, ignoring the `frame_id` parameter the cloud path honours — the IMU came out in a frame that exists nowhere in the TF tree. Patch 0005, upstreamable.
  - **The driver's ROS2 `frame_id` default is `frame_default`** (its example launch files say `livox_frame`); publisher QoS is **Reliable, depth 256, not parameterised**; the publisher is created **lazily on first data**; and it registers `livox_ros::DriverNode` as an `rclcpp_components` plugin (so §12.1 could compose it if the Orin budget ever demands).
  - **Upstream's `build.sh` cannot be used**: it `rm -rf`s `../../build`, `../../devel` and `../../install`, i.e. this workspace. Patch 0005 supplies `package.xml` (the ROS2 one verbatim) and a `colcon.pkg` carrying `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`; without `DISTRO_ROS` the humble typesupport-target branch is skipped and the link fails.
  - **DLIO has no TF listener at all.** `extrinsics/baselink2{lidar,imu}` are the *only* definition of where `base_link` sits relative to the sensor. Left at upstream's identity, `odom→base_link` is really `odom→mid360_link`: 0.472 m out in z and rotated by roll = π. In sim, `base_link` ≡ MuJoCo pelvis was free.
  - **DLIO broadcasts `base_link→frames/lidar` and `base_link→frames/imu` unconditionally**; naming either `mid360_link` gives that frame **two parents** alongside `robot_state_publisher`. Ours are `dlio_lidar_link` / `dlio_imu_link`.
  - **DLIO's cloud subscription is Reliable** — it never matches a best-effort cloud publisher, i.e. the sim sidecar, every bag recorded from it, and the bench stub. Exactly the failure class the extractor hit in Phase 3: connected topic, no data. Patch 0006 (SensorData, depth 1 kept) + `/odom` depth 1 → 10 per §7.1.
  - **DLIO's `cfg/params.yaml` ships `use_sim_time: true`.** On hardware there is no `/clock`, so the node's clock never advances and its 100 Hz publish timer never fires.
  - **DLIO publishes TF at SCAN rate (~10 Hz), not at `/odom`'s 100 Hz** — this phase's headline seam finding. `publishPose()` (100 Hz wall timer, `odom.cc:328-369`) emits only `/odom` and `/pose`; all three TF broadcasts live in `publishToROS()` (`odom.cc:371-444`), spawned as a thread once per scan (`odom.cc:852`). Measured, not assumed (numbers below). `base_footprint_publisher` inherits the 10 Hz, so §7.1's "TF `odom→base_footprint` 100 Hz" describes the sim sidecar only. Both rows corrected.
  - **Timestamp mode is not a driver configuration** (§14.3 corrected): no PTP field exists in `MID360_config.json`; the driver reads the sync mode from each packet header and falls back to the **host clock at packet reception** when the LiDAR reports `kTimestampTypeNoSync`. Which mode is live is only observable at runtime → 5B block 1 measures it.
  - **`/livox/imu` has no sim counterpart at all.** The sidecar never published it (§7.1 said "optional"), nothing in Phases 1–4 needed it, and DLIO does not start without it. The single largest "free in sim" gap.
- **Design decisions:**
  - **All hardware specifics live in `source_hw.launch.py` + three configs; `perception.launch.py` is byte-unchanged and still conditional-free (D4).** The one new sim/hw branch is the `use_sim_time` default in `bringup.launch.py`, derived from `source` via a `PythonExpression` — in the switchboard, not in the shared stack.
  - **DLIO's extrinsics are derived from `g1_mid360.xacro`, never hand-written** (§8.3 says that file is the only place extrinsics live for ROS). New CTest `t7_hw_extrinsic_guard` recomputes `baselink2lidar = (base_link→torso_link) ∘ H-1` and `baselink2imu = baselink2lidar ∘ (lidar→IMU)` from the xacro on every build and fails on drift; it also rejects any `frames/*` name from the §8.2 tree and any `use_sim_time: true`. The lidar→IMU offset `(0.011, 0.02329, −0.04412)` m, identity rotation, is the Livox Mid-360 manual constant (the value every Mid360 FAST-LIO config carries as `extrinsic_T = −that`, including deepglint's G1 config) — **not verified on this unit; 5B checklist item**, and a 5 cm lever arm.
  - **A new "one harness, two worlds" seam**: `test_detection_static.launch_test.py` (the sim T4 gate) now takes `T4_BAG` / `T4_LAYOUT` / `T4_USE_SIM_TIME` from the environment, defaulting to exactly the committed sim fixture and GT. T4-hardware is therefore the *same file* pointed at a hardware bag plus a surveyed-layout YAML, with per-target radii — so a layout mixing r=0.15 with r≥0.30 props measures the Phase-4 "bias grows with radius" finding directly. Verified the sim path is unchanged: it reproduces Phase 3's numbers to the tenth of a millimetre.
  - **`hw_source_stub.py` — a driver *output emulator*, not a LiDAR simulator.** It replays a sim fixture's geometry re-wrapped in the driver's exact wire format (7 fields, `point_step` 26 `#pragma pack(1)`, Reliable depth 256, wall-clock stamps) plus a synthetic stationary IMU. That is what makes the cloud contract, the QoS wire-check and the TF tree shape bench-gatable instead of robot-day discoveries. It is honest about being a rig: the tests it drives assert wiring and format, never odometry quality.
  - **`livox_sdk2_vendor`**: Livox-SDK2 is plain CMake with no `package.xml`, so it is COLCON_IGNOREd and driven into the merged prefix by an `ExternalProject_Add` vendor package — same shape as `unitree_dds_wrapper_vendor`, and it keeps the no-sudo/pinned-workspace discipline (upstream installs to `/usr/local`).
  - **P-5 recorded but NOT fired.** The obvious fix for the TF-rate finding (broadcast `odom→base_link` from `publishPose()` too, using the IMU-propagated state it already publishes) is held as a conditional patch, because the measured end-to-end cost leaves ~47 ms of the §17.2 budget unused. Same rule that gated P-2 in Phase 3: measure first, patch on a failed number. `test_dlio_wiring.launch_test.py` asserts the publish split so the premise cannot change silently.
- **Pinned SHAs added (resolved 2026-08-01):** Livox-SDK2 `f5d9375f` (**tag v1.3.1**, 2026-04-15 — deliberately the tag, not master, which is v1.3.1 + one Mid-360S/Ubuntu-24.04 commit this robot does not need); livox_ros_driver2 `13eb05e4` (tag 1.2.6, 2026-04-14); direct_lidar_inertial_odometry `c8acc371` (`feature/ros2`, 2024-11-21 — the SHA Phase 0 reserved, re-verified as branch tip). Reserved but **not imported**: FAST_LIO_LOCALIZATION_HUMANOID `df4772ec` (`humble`), with the reason it cannot be co-run recorded next to the pin.
- **Files added:** `ros2/patches/{0005-livox-driver-ros2-humble-package-and-imu-frame-id,0006-dlio-cloud-subscription-sensor-data-qos}.patch`; package `livox_sdk2_vendor`; `g1_description/test/t7_hw_extrinsic_guard.py`; bringup `config/{livox_driver.yaml,MID360_config.json,dlio.yaml}`; bringup `test/{hw_source_stub.py,hw_config_check.py,selfhit_analysis.py,isolate_domain.py,test_hw_source_contract.launch_test.py,test_dlio_wiring.launch_test.py}`; `ros2/doc/{phase5a_seam_audit.md,phase5b_checklists.md}`.
- **Files modified (all inside `ros2/` — zero outside-`ros2/` edits this phase apart from this §21 entry and the body corrections; confirmed by `git status`):** `deps.repos`, `setup_external.sh`, `README.md`, `.gitignore`, `test_fixtures/README.md`, `g1_description/CMakeLists.txt`, `g1_perception_bringup/CMakeLists.txt`, `launch/{source_hw,bringup,record}.launch.py`, and the eight launch tests (domain isolation; plus `test_detection_static.launch_test.py`'s env parameterisation). **`perception.launch.py`, `dpcbf/`, `deploy/`, `src/`, `scripts/` untouched.**
- **Build outcomes on the dev machine (x86_64, Humble, `--merge-install`, Release):** `livox_sdk2_vendor` 6.4 s (installs `liblivox_lidar_sdk_{shared.so,static.a}` + 3 headers into the prefix, where the driver's `find_library`/`find_path` resolve them); `livox_ros_driver2` 6.7 s clean, producing `livox_ros_driver2_node` + the registered component; `direct_lidar_inertial_odometry` 37.6 s clean. Full workspace: **18 packages, no errors.** `apr-1` is absent and harmlessly optional (`pkg_check_modules` is not REQUIRED). **On the target platform: not attempted — see the aarch64 blocker.**
- **Bench gate results (all PASS; every number dev machine):**

  | Check | Result |
  |---|---|
  | **Driver against no device** | Fails at the SDK: `bind failed` / `Init lds lidar fail!`. **Not "cleanly":** the node stays alive, creates **no `/livox/*` topic at all** (lazy publisher), and ignores SIGINT *and* SIGTERM — `ros2 launch` escalated to SIGKILL after 15 s. Recorded as a robot-day diagnostic rule and a `pkill -9` instruction |
  | **`hw_config_check.py`** | Correctly refuses the placeholder config: `192.168.1.5 is not assigned to any local interface`, exit 2 (CTest SKIP). Also checks ports free, LiDAR/host /24 agreement, `xfer_format == 0`, `frame_id == mid360_link`, and identity `extrinsic_parameter` (H-1 in the JSON *and* the xacro would apply the extrinsic twice) |
  | **Hardware cloud contract** | Driver-format cloud (7 fields, `point_step` 26) through the **unmodified** perception stack: 278 clouds → 274/269/277/277/277/277 cloud/filtered/scan/raw/tracked/safe. §7.2 holds against the real record, not just on paper |
  | **QoS wire-check** | Every `/livox/lidar` subscriber best-effort (`crop_box_self_filter`, `dlio_odom_node`); `/scan` publisher best-effort; `/obstacles_safe` publisher Reliable — all §7.1 |
  | **TF tree shape** | `odom→base_link→torso_link→mid360_link` and `odom→base_footprint` resolve; **every frame has exactly one parent**; `mid360_link`'s parent is `torso_link` with DLIO running |
  | **DLIO wiring** | Node up, subscribed to `/livox/lidar` (BEST_EFFORT — patch 0006 on the wire) and `/livox/imu`; `/odom` **100.0 Hz** Reliable, `odom`→`base_link`; `use_sim_time` **False**; `dlio_lidar_link`/`dlio_imu_link` present under `base_link` |
  | **T7-hw extrinsic guard** | PASS at 4.4e-13 m / 3.7e-13 rad against the xacro-derived transforms |
  | **T7 / T4 / T5 / T8 / T9 (sim, regression)** | All still PASS; T4 reproduces Phase 3 exactly: centre +40.9 / +38.4 / +40.1 mm and `true_radius` +16.7 / +19.7 / +16.0 mm at 1/2/3 m |

- **Performance numbers (dev machine; the seam's real cost):**

  | | sim (Phases 3–4) | hardware path, DLIO in the loop |
  |---|---|---|
  | cloud → `/scan` | p50 0.48 / p95 0.95 ms | **p50 9.24 / p95 12.81 ms** |
  | cloud → `/obstacles_safe` | p50 1.28 / p95 1.76 ms | **p50 9.61 / p95 13.19 ms** |
  | frames, steady state | — | 249 clouds → 249 scans → 249 `/obstacles_safe` over 25 s, **zero loss** |
  | TF `odom→base_link` | 100 Hz (sidecar) | **10.0 Hz** (measured; `/odom` 100.0 Hz in the same run) |

  The ~9 ms is the tf2 `MessageFilter` waiting for the TF sample that brackets the cloud stamp. Startup transient: **31 clouds** dropped with *"timestamp … earlier than all the data in the transform cache"* before DLIO's first TF — expected, and worth knowing so it is not read as a fault on robot day. §17.2's 60 ms budget retains ~47 ms of headroom, **on this CPU**; the Orin is the open question and the number that decides P-5.
- **Test counts: 65 → 80** (`colcon test-result --all` over the project packages: **0 errors, 0 failures, 2 skipped**). New: `t7_hw_extrinsic_guard`, `test_hw_source_contract` (5 asserts), `test_dlio_wiring` (6 asserts), `hw_config_check` (the second skip — expected while the config carries Q-1 placeholders). Which now also run against hardware bags: **`test_detection_static.launch_test.py`**, via `T4_BAG`/`T4_LAYOUT`/`T4_USE_SIM_TIME` — the only harness 5B needs to point at a session, by design.
- **Issues discovered / root causes / fixes:**
  - **`colcon test` was silently cross-talking.** Adding two launch tests to `g1_perception_bringup` pushed the concurrent count high enough that tests began seeing each other's topics: a 278-cloud replay probe reported **597** clouds and 320 T9 "misses", and the wall gate reported 51 clouds / 26 misses — while their *own* measurements (wall errors, `/scan` rate, drop fraction) stayed correct. Nothing about the product was wrong; the harness was talking to itself, and the pre-existing tests had simply been under the collision threshold. Fix: `isolate_domain.py`, a hand-assigned `ROS_DOMAIN_ID` per launch test, set at module import before rclpy or any launch action exists. Deliberately **unconditional** (the documented workspace default exports `ROS_DOMAIN_ID=0`, so an "only if unset" rule would never have fired), with `PERCEPTION_TEST_DOMAIN` as the override for pointing a single test at a live simulator.
  - **`pkill -f component_container` kills the shell that runs it** — the shell's own command line contains the pattern, so pkill matches itself and the rest of the script never executes. It presents as a silent hang and cost one debugging cycle (and one stale log read as a real failure). Always bracket: `pkill -9 -f 'component_containe[r]'`. Same family as Phase 4's `unitree_mujoc[o]` trap; both now in `ros2/README.md`.
  - **`launch_test` does not put the test file's directory on `sys.path`**, so the shared `isolate_domain` import needed an explicit `sys.path.insert` (the Phase-3 `tracking_dynamic_common` import already did this; the pattern was there to copy and I did not, at the cost of one full test cycle).
  - **`static_transform_publisher` is not a valid stand-in for odometry.** `base_footprint_publisher` deduplicates by stamp, so a *static* `odom→base_link` makes it emit `base_footprint` exactly once at time 0; every later lookup extrapolates into the future, `pointcloud_to_laserscan` produces nothing, and the failure looks like a broken cloud path. Real odometry — sidecar or DLIO — is dynamic; `hw_source_stub.py` grew an `odom_hz` mode. A rig lesson, not a product defect, but exactly the kind of thing that would have eaten robot time.
  - **CRLF**: `lddc.cpp` is CRLF; a naive rewrite turned a 1-line patch into a 1410-line diff. Patch 0005 preserves line endings.
- **Remaining limitations:**
  - **No robot, no Q-1, so 5B did not start** — the extrinsic verification, drift gate, self-hit capture, T4-hardware, Q-3/Q-5/Q-8 and the CPU/latency benchmark are all unmeasured.
  - **No aarch64 build** (root-required; audit only). The chosen-target build remains the largest 5A risk carried forward.
  - Every latency/CPU number is dev-machine. The §17.4 Orin budget is untested.
  - The DLIO IMU stream in the bench tests is synthetic and stationary, so DLIO's *odometry* has never been exercised against realistic motion — only its wiring. The bench numbers say nothing about drift.
  - `MID360_config.json` still carries upstream placeholder IPs by design; the CropBox box is still the Phase-3 **sim-interim**; the lidar→IMU offset is a datasheet constant, unverified on this unit.
  - Bags still sqlite3 (MCAP blocked on apt); walking evidence and the interactive RViz screenshot still blocked (carried from Phases 1–4).
- **Lessons learned:** "verify against the checkout, not the doc" produced eleven findings across two externals this phase, three of which (DLIO's Reliable subscription, its identity extrinsics, its `use_sim_time: true`) would each have produced a *silent* no-data or wrong-frame condition on robot day rather than an error message. A driver-output *emulator* is worth more than a device simulator for seam work: it costs 200 lines and converts "we will find out on the robot" into a CTest. And when a green test suite turns red after adding tests, suspect the harness's own concurrency before the product — the failing assertions here were T9 misses whose counts were arithmetically impossible for the bag being replayed, which is what pointed at cross-talk rather than a regression.
- **Open follow-up items:** carried — org forks + upstream PRs (now 0002/0003/0004 **+ 0005 driver + 0006 DLIO**), MCAP/foxglove when sudo appears, walking evidence + interactive RViz screenshot, python-mujoco pin decision, P-3 covariance export (still gated on hardware error data), R-4 levers if S3-class density becomes a target, OSQP maxIter under swarm load. New — **Q-1 answers and the topology decision (blocking for 5B)**; **aarch64/target build**; **Orin CPU + latency benchmark**; **P-5** (DLIO TF from `publishPose`, fires on a failed T9-hardware or Orin latency number); **Q-4 needs its own session** (the driver cannot dual-publish); **verify the Mid-360 lidar→IMU offset on this unit**; **`ros2 bag record` may not resolve Unitree message types** for the Q-5 cross-check (Phase 4 saw the same limit with `ros2 topic hz`) — an SDK2-linked recorder may be needed.
- **Suggested next step:** **Phase 5B — the robot session**, driven by `ros2/doc/phase5b_checklists.md` (8 blocks, each with preconditions, exact commands, expected observables, abort criteria and its bag). Its prerequisites are exactly three: **(1) Q-1 answered and the topology decision recorded; (2) the workspace built on whatever will run it — the aarch64 build is the open risk; (3) props sourced, including one r ≥ 0.30 m, plus a tape-measure survey of a flat wall.** With those, blocks 1–8 are a capture session, not a debugging session. If the robot is unavailable, the honest alternative is not to skip ahead: **Phase 6 depends on 5B's data** (the CropBox retune, the k_σ dataset, and the drift gate all feed it), so the next useful work without a robot is retiring the aarch64 risk — a container or an Orin dev kit — rather than starting Phase 6.

### 2026-08-02 — Interim block (between 5A and 5B): retire risk, close deferred gates, prepare P-3

- **Objective:** not a §18 phase. Four independent, timeboxed workstreams, none needing the robot: **A** retire the aarch64 risk, **B** close the walking gates deferred since Phases 1 and 4, **C** implement P-3 and characterise the circle-fit bias, **D** carried housekeeping. Reported per workstream against the evidence bar each was given.

- **OPERATOR INPUT NEEDED (answerable in one pass):**
  1. **One apt transaction with sudo** on the dev machine: `ros-humble-rosbag2-storage-mcap`, `ros-humble-foxglove-bridge`. **`qemu-user-static` and `binfmt-support` are NO LONGER needed** — see workstream A; that ask is withdrawn.
  2. **Q-1's full answer set for 5B** (unchanged, still blocking): G1 variant and onboard PC; Mid360 factory mount y/n; robot network and IPs; sudo + internet on the Orin; its shipped Ubuntu/ROS state.
  3. **arm64 hardware in the lab** (a Pi 4/5, a spare Jetson, any arm64 box). This would retire workstream A's remainder outright and is worth more than any further emulation work.
  4. **Props and robot-day scheduling.** Prop constraint has CHANGED — see workstream C: **r ≥ 0.30 m but not above ≈0.32 m**, or `max_circle_radius` must be raised in two config files first.
  5. **Fork/PR authorisation** — pushing this project's patches to public forks is an outward-facing action; it was not taken unilaterally (workstream D).

- **Workstream A — aarch64. Bar: full workspace building, tests run, native/emulated/cross stated. NOT MET. Achieved: 10 of 18 packages, EMULATED. Risk reduced, not retired.**
  - **5A's blocker was wrong and that part is retired.** 5A recorded "no cross toolchain, no `qemu-user-static`, no binfmt handler — root required". Every clause is true; the conclusion is not. `qemu-user-static` is a **static** binary needing only `dpkg-deb -x` into `$HOME`; **Linux ≥ 6.7 allows `binfmt_misc` to be mounted and registered inside a user+mount namespace** (this kernel is 6.8.0-124), so `unshare -Urm --map-root-user` is enough; and rootless podman can `create`+`export` an arm64 rootfs even though it cannot exec one. Step 1 of the phase prompt (ask for root) was therefore never needed and the ask is withdrawn. Committed as `tools/arm64_emulate.sh`: a full aarch64 Ubuntu 22.04 + ROS Humble environment, working `apt`, aarch64 gcc 11.4.0, 32 cores.
  - **Built (emulated, sequential, ~16 min):** cyclonedds (2m42), unitree_sdk2 (5m18), rmw_cyclonedds_cpp, obstacle_detector incl. the new P-3 message (2m42), sim_msgs, g1_description, dpcbf_ros_adapter, g1_perception_utils, livox_sdk2_vendor, unitree_dds_wrapper_vendor. **Not built:** pcl_ros, pointcloud_to_laserscan, livox_ros_driver2, t10_dds_coexistence, g1_perception_bringup, safety_obstacle_filter, sim_mjlidar_bridge, DLIO. **No test suite was run on aarch64** — with the workspace incomplete there was nothing meaningful to run, so that half of the bar is untouched too.
  - **Nothing arch-specific was found.** 5A's static portability audit holds: no endianness, alignment or intrinsics problem appeared. All five faults were environment/CMake plumbing, and four are fixed: **(1)** `livox_ros_driver2` never declared its build-order dependency on `livox_sdk2_vendor` — the dev machine's *parallel* build happened to order it correctly and hid this for an entire phase; a sequential build fails at `find_library(LIVOX_LIDAR_SDK_LIBRARY)`. **Patch 0008**, a genuine repo bug. **(2)** Ubuntu's `libvtk9-dev` ships imported targets for `RelWithDebInfo` only → `CMAKE_MAP_IMPORTED_CONFIG_RELEASE`. **(3)** PCL headers include Eigen unqualified → `-isystem /usr/include/eigen3`. **(4)** `find_package(OpenSSL)` inside the ROS config chain reports "missing: OPENSSL_CRYPTO_LIBRARY (found version 3.0.2)" while a bare `find_library(crypto)` in the same shell resolves it → name the paths.
  - **The fifth is NOT fixed and is the reason the bar is missed.** An upstream `ament_cmake` bug: `<pkg> exports the library '<pkg>' which couldn't be found` while the library is present and readable. Traced with `--trace-source` + `-DCMAKE_FIND_DEBUG_MODE=ON` — find_library prints *"The item was found at /opt/ros/humble/lib/liblaser_geometry.so"* on the line above the error. Mechanism: `set(_lib "NOTFOUND")` creates a NORMAL variable shadowing the CACHE variable that `find_library` writes; `_lib` is one cache slot shared by every such file in a configure; whether the shadow survives depends on CMP0126 in the enclosing scope, hence on the include stack, hence on **which packages are installed**. Same source, same CMake 3.22.1: fine on x86_64 here, fatal in the arm64 image, and the package it blames moves as the installed set changes. `unset(_lib CACHE)` before the search took the build from 3 to 10 packages, then the identical failure appeared in a second generated-file family (`rosidl_cmake_export_typesupport_libraries-extras.cmake`) that neither variant resolved; reverting all edits reproduced the failure **on pristine files at a different package**, confirming both the order-dependence and that the patch was not the cause. **Deliberately shipped as a diagnostic, not a fix** (`tools/diagnose_ament_export_libraries.py`): patching `/opt/ros` on the robot with something whose effect on link lines is not understood is worse than the failure. Upstream fix belongs in ros2/ament_cmake — stop sharing one cache variable.
  - **Deliverables per the bar's fallback:** `tools/build_target.sh` (on-target build; `--check` preflight; each fix annotated with the error it prevents; ends with a **robot-day first-hour triage list** keyed by error message), `tools/arm64_emulate.sh`, `tools/diagnose_ament_export_libraries.py`, patch 0008, and `evidence/interim/aarch64_build.md`.

- **Workstream B — walking. Bar: the policy locomoting stably long enough to run the A/B. NOT MET. The identified gap was closed; the run was not attempted.**
  - **Done:** `ScriptedJoystick` now takes **button events** — the precise missing piece Phase 4 diagnosed. Profile lines gain an optional 5th column, a comma-separated key list held to the next breakpoint (`L2 R2 L1 R1 A B X Y up down left right start select`, plus LT/RT/LB/RB aliases). Read out of `g1_pub.h`'s wireless_remote packing: the FSM's "L2"/"R2" are the joystick's **LT/RT axes**, not buttons, so they are driven to 1.0 with `smooth = 1.0` (the default 0.03 would need ~50 updates to cross the 0.5 press threshold, so a short scripted chord would silently never register). Chord keys deliberately bypass `axis_filter_`: DPCBF gates locomotion commands, not mode changes (§10.5). An unknown key name aborts at load — a typo'd key doing nothing is exactly how a scripted FSM run becomes an unexplained "the robot never stood up". `simulate` builds; documented in `ros2/README.md` with the `L2+up` → FixStand, `R2+A` → RLBase profile.
  - **Not done, and why:** the FSM drive-through and the post-spawn-collapse investigation were not started. Workstream A consumed the block's budget, and the phase brief says B must not consume the phase. **Consequently §17.3's collision rate and min-clearance remain deferred for a sixth phase**, the shadow-mode deltas are not re-priced, and **the rig-vs-density verdict on S3's 0.8235 is not delivered.** Robot state reached this block: none — the simulator was not run under policy. The remaining work is now genuinely small and fully specified: write the profile, run, instrument initial pose/settling time/gains.

- **Workstream C — P-3 and the circle-fit bias. Both bars MET.**
  - **P-3 implemented as fork patch 0007** (D6's one sanctioned message change): `CircleObstacle` gains `float64[3] covariance` = `[var_x, var_y, var_r]`, filled in `publishObstacles()` from the per-axis KFs' `P(0,0)`. Only meaningful post-P-2, which made `P` start from `R` instead of upstream's identity.
  - **σ path wired in `safety_obstacle_filter`, DEFAULT-DISABLED** (`use_covariance: false`), config-gated, with `sigma_max` capping a diverged track. `SurfaceSigma() = sqrt(max(var_x,var_y) + var_r)` — worst-case centre error over direction plus an independent radius error in quadrature. Producers that are not the tracker leave covariance zero, so the term **self-disables rather than mis-fires** (unit test). Shipped behaviour remains `fixed_inflation = 0.051`.
  - **Containment unchanged on S1/S2 — the bar.** Full `phase4_ab_run.sh` re-run reproduces Phase 4 exactly, **S3 perf ratio 0.8235**, the same figure the Phase-4 entry records. Containment with the σ term off: s1_static 100.000 %, s2_cross_05 100.000 %, s2_cross_08 100.000 %, s3_swarm 85.181 %, s4_occlusion 96.804 %.
  - **Calibration harness:** `test/calibrate_k_sigma.py`, run end-to-end on sim data; `phase4_obstacles_dump.py` now records `cov`, so 5B is a data drop. With `fixed = 51 mm`, `k_σ = 0.287` covers 99.9 % pooled; coverage 92.524 % → 99.946 %, s3_swarm 85.181 → 99.887, s4_occlusion 96.804 → 100.000, S1/S2 untouched. **The shape of the remedy is right.**
  - **The number is not, and this is the block's most useful finding: σ is not in metres.** σ p50 is **583 mm** (8.1 m at the S3 tail) because `obstacle_detector.yaml`'s `measurement_variance: 1.0` is a **unitless** knob that P-2 seeds `P(0,0)` from. So `k_σ` silently absorbs the missing scale, and **neither 0.287 nor the old branch's 2.748 is a sigma multiplier**. The fix is upstream of `k_σ`: set `measurement_variance` to the Mid-360's measured range-noise variance in m² (5B block 1, no extra robot time) *before* anyone calibrates. `corr(F_req, σ) = +0.339` — positive but weak; if hardware does not improve on it, the honest verdict is that per-track covariance is the wrong observable.
  - **Circle-fit bias — characterised, with a functional form.** New harness `test/circle_fit_sweep.py` drives the **real extractor** with analytic ray-cast scans against a known cylinder (210 configurations, 193 detections), isolating the fit from odometry, projection and tracking. It reproduces the project's existing numbers (r=0.15: 34–44 mm centre vs Phase 3's 40.9/38.4/40.1; r=0.30: 76–103 mm vs Phase 4's 83 mm). **Fit through the origin, full arc, r ∈ [0.10, 0.40], d ∈ [1, 4] m:** `e_r = +0.084·r` (rms 5.2 mm), `e_c = −0.278·r` (rms 6.0 mm) — **both proportional to true radius and independent of range**. One correction to the record: Phase 4 reported the centre bias as a magnitude; it is signed, and the centre is pulled **towards** the sensor, so near-surface error is conservative and only the far side is short. Safety-relevant combination `F_req = |e_c| − e_r ≈ 0.22·r`.

    | arc visible | e_r/r | e_c/r | F_req/r (r≥0.20) | max F_req |
    |---|---|---|---|---|
    | 1.00 | **+0.078** | −0.276 | +0.223 | 89 mm |
    | 0.90 | −0.034 | −0.366 | +0.465 | 172 mm |
    | 0.75 | −0.164 | −0.439 | +0.684 | 236 mm |
    | 0.60 | −0.284 | −0.480 | +0.847 | 286 mm |
    | 0.50 | −0.367 | −0.493 | +0.956 | 299 mm |

    **The radius error changes sign at ~10 % occlusion** — a fully visible cylinder reads slightly too large (safe), a partly occluded one reads up to 37 % too small while the centre error keeps growing. That is a quantitative account of exactly the residual Phase 4 could not cover (S3's 15 % of pairs, S4's crosser coast). It also says a *fixed* inflation is the wrong shape: the requirement is a product of radius and occlusion. No correction was implemented — the obvious tangent-pair estimator under-estimates under truncation, the unsafe direction, and needs hardware arc statistics first.
  - **A drop-out that must be fixed before robot day.** 17 of 210 configurations produced **no detection at all**, and they are the large radii: `detectCircles()` keeps a circle only if `1.084·r + radius_enlargement(0.17) < max_circle_radius(0.60)`, i.e. **r < 0.40 m**; `safety_obstacle_filter` then gates `true_radius > 0.60` again. **A prop at r = 0.40 m is invisible at 2–3 m with no warning**, and truncation perversely *restores* detection (shorter chord fits under the cut), so it presents intermittently as the robot moves. 5B's "one prop r ≥ 0.30 m" now carries an upper bound of ≈0.32 m unless both configs are changed first. Checklist updated.

- **Workstream D — housekeeping. Partly done; the fork/PR item is deliberately not done.**
  - **Org forks + upstream PRs: NOT DONE.** `gh` is not installed and no credentials for a project org exist here; more importantly, creating public forks and opening PRs under this project's name is an outward-facing action that was not authorised. Everything short of pushing is ready: eight recorded patches with measured evidence, and the genuinely upstreamable ones identified — 0003's `begin()++` double-count, 0004's KF initialisation, 0005's `/livox/imu` `frame_id`, 0006's DLIO QoS, 0008's missing dependency, plus the new ament_cmake bug (a separate upstream target). Carried as an operator ask.
  - **python-mujoco pin: DECIDED — pin 3.6.0**, deliberately *not* the 3.3.6 the simulator vendors on the C side. H-3 is a real `mj_multiRay` AABB-pruning miss measured on 3.3.6; T2 passes on 3.6.0 precisely because it is fixed there, so matching the C runtime for symmetry would trade a passing safety gate for a failing one and force the T2 backend switch. The skew is bounded: the C side does physics, python raycasts the mirror, and the only thing they exchange is pose, which `test_mirror.py` and T7 already assert agree. Made **enforceable** rather than a note: new test `test_python_mujoco_pin` fails loudly on drift below the pin with the reason. This closes a follow-up carried since Phase 1.
  - **MCAP/foxglove:** still apt-blocked; folded into the (now smaller) operator ask.
  - **`phase5b_checklists.md` updated:** the prop radius bound and its mechanism; the build-script/diagnostic preflight with an honest "10 of 18" expectation; range-noise capture for `measurement_variance` derived from bags block 1 and 2 already produce; P-3 as a data drop with the exact two commands; and an arc-truncation capture that tests the whole bias model in one 60 s bag.
  - **Unplanned but required: five fixture bags had to be regenerated.** Patch 0007's message change invalidates every recorded bag carrying `obstacle_detector/msg/Obstacles`; T5 went red with "no GT crosser samples in bag replay" — deserialisation, not a product regression. `s1_surveyed`, `s2_cross_05`, `s2_cross_08`, `s3_swarm`, `s4_occlusion` regenerated from `test_fixtures/README.md`'s recipe (which the exercise also validated); counts reproduce exactly (s2_cross_05 1076 GT / 216 clouds as before). **This is the standing cost of D6's sanctioned message change and should be expected again if the message ever changes.** New md5s: s2_cross_05 `3fde31e71d5b1124aa8a74fea9456850`, s2_cross_08 `3174aa0bf76d12dc41b7c0cdecbc6309`.
  - Also fixed while using it: `phase4_ab_run.sh` aborted immediately on any clean machine — `pkill` returns 1 when nothing matches, and the script runs under `set -e`.

- **Files added:** `ros2/patches/{0007-obstacle-detector-p3-per-track-covariance,0008-livox-driver-declare-sdk-vendor-dependency}.patch`; `ros2/tools/{arm64_emulate.sh,build_target.sh,diagnose_ament_export_libraries.py}`; `ros2/src/g1_perception/g1_perception_bringup/test/{circle_fit_sweep.py,calibrate_k_sigma.py}`; `ros2/evidence/interim/{aarch64_build.md,circle_fit_bias.md,p3_sigma_path.md,circle_fit_sweep.csv,k_sigma_sim.txt}`.
- **Files modified (inside `ros2/`):** `setup_external.sh` (patches 0007, 0008), `README.md`, `doc/phase5b_checklists.md`, `config/safety_obstacle_filter.yaml`, `safety_obstacle_filter/{include/safety_obstacle_filter/gating.h,src/safety_obstacle_filter_node.cpp,test/test_gating.cpp}`, `sim_mjlidar_bridge/test/test_gates_t2_t3.py`, `g1_perception_bringup/test/{phase4_obstacles_dump.py,phase4_ab_run.sh}`.
- **Complete outside-`ros2/` edit list:** `simulate/src/main.cc` (ScriptedJoystick buttons — the already-sanctioned file) and this §21 entry. **`simulate/CMakeLists.txt` was NOT touched** (no new file), nor `ros2_bridge.{h,cc}`. `dpcbf/`, `deploy/`, `src/`, `scripts/` untouched — confirmed by `git status`.
- **Test counts: 80 → 85**, over the project packages: **0 errors, 0 failures, 2 skipped** (dpcbf_ros_adapter 18, safety_obstacle_filter 12, g1_perception_utils 5, g1_description 2, g1_perception_bringup 41 (2 skipped), sim_mjlidar_bridge 7). New: four σ-path gating tests (off-by-default, enabled, self-disable on zero covariance, cap) and `test_python_mujoco_pin`. Per-test DDS domains preserved. T5 is green again only because the fixtures were regenerated — it was red on the old bags and that is recorded above, not papered over.
- **Evidence artefacts:** `ros2/evidence/interim/` — `aarch64_build.md`, `circle_fit_bias.md` + `circle_fit_sweep.csv` (210 rows), `p3_sigma_path.md` + `k_sigma_sim.txt`.
- **Issues discovered / root causes:** the five build faults above; the message-change fixture invalidation; `phase4_ab_run.sh`'s `set -e`/`pkill` abort; and one harness bug worth recording because it produced *plausible* numbers — the first bias sweep read the previous configuration's reply (the extractor stamps `/raw_obstacles` with `now()`, so a reply cannot be matched to its scan by header), giving impossible half-metre centre errors and identical rows for different arc fractions. Fixed by draining the subscription and publishing exactly one isolated scan. The tell was not that the numbers were large but that they were **identical across configurations that must differ**.
- **Remaining limitations:** no aarch64 test run and 8 of 18 packages unbuilt there; no walking, so collision rate and min-clearance are still unmeasured and S3's 0.8235 is still un-decomposed into rig vs density; `k_σ` underivable in physical units until `measurement_variance` is set from hardware; the circle-fit correction is characterised but not implemented; every number here is sim or emulation.
- **Lessons learned:** a blocker recorded as "root required" is worth re-deriving rather than inheriting — three of 5A's four aarch64 clauses were true and the conclusion still wrong, and the whole environment took under an hour once that was tested instead of assumed. A build that works only on the machine it was developed on is not a working build: **four of five faults were latent all along and hidden by parallel scheduling or a fuller package set**, which is precisely what a robot-day build will not have. And when a fix stops working after it worked once, check whether it changed the *failure* rather than the *outcome* — `unset(_lib CACHE)` genuinely advanced the build seven packages and still had to be withdrawn, because advancing is not the same as being correct.
- **Open follow-up items:** carried — org forks + upstream PRs (now 0002–0008, **plus the ament_cmake bug as a separate upstream target**), MCAP/foxglove, walking evidence + interactive RViz screenshot, Orin CPU + latency benchmark, P-5, Q-4's own session, Mid-360 lidar→IMU offset verification, SDK2-linked recorder for Q-5, R-4 levers, OSQP maxIter under swarm load. **Closed this block:** python-mujoco pin decision (3.6.0, enforced by a test); P-3 covariance export (implemented — only its *calibration* is still gated on hardware). New — **`measurement_variance` must be set from hardware range noise before `k_σ` means anything**; **`max_circle_radius` vs prop radius** (r < 0.40 m hard cut, two config files); **the ament `_lib` bug is an unresolved robot-day hazard**; **fixture bags must be regenerated on any `Obstacles` message change**; **circle-fit correction** (tangent-pair estimator, needs hardware arc statistics first).
- **Suggested next step:** **5B's three preconditions are still not met** — Q-1 unanswered, the target build unproven, props unsourced — so 5B is not yet a capture session. The largest remaining risk is unchanged in identity but much reduced in size: **the on-target build**. The highest-value next action is **an hour on any arm64 box in the lab** (a Pi is enough) running `tools/build_target.sh`, which would either finish the job natively or reproduce the ament bug on a second machine and settle whether it is image-specific. Failing that, **workstream B is now the best use of a no-robot session**: the missing piece is built and the remaining work is bounded, and it is the only path to the two safety numbers this project has never had.

### 2026-08-02 — Interim block 2 (still between 5A and 5B): walking, and two safety-relevant gaps

- **Objective:** workstream **B** (walking) as the main job — close the gates deferred since Phase 1 and produce the two §17.3 safety numbers this project has never had — plus two sharply-specified gaps carried from workstream C, both safety-relevant and both cheap: **G1** (σ's units) and **G2** (the silent large-radius drop). Workstream **A** was NOT resumed: no arm64 box appeared, so `build_target.sh` + the diagnostic + the robot-day triage list remain the deliverables and the risk carries unchanged.

- **OPERATOR INPUT NEEDED:**
  1. ~~sudo apt for MCAP/foxglove~~ — **DONE** (`ros-humble-rosbag2-storage-mcap`, `ros-humble-foxglove-bridge` installed). Switching the record configs over is now an ordinary follow-up, not an ask.
  2. **Q-1's full answer set for 5B** (unchanged, still blocking): G1 variant and onboard PC; Mid360 factory mount y/n; robot network and IPs; sudo + internet on the Orin; its shipped Ubuntu/ROS state.
  3. **arm64 hardware in the lab** (a Pi 4/5, a spare Jetson, any arm64 box) — unchanged, and still the cheapest way to retire workstream A outright.
  4. **Props and robot-day scheduling.** Constraint **RELAXED**: `r ≥ 0.30 m`, and the interim block's `≤ ≈0.32 m` upper bound is **WITHDRAWN** — see G2. Anything to `r ≈ 0.55 m` is now visible.
  5. **Fork/PR authorisation** — still not taken unilaterally; everything short of pushing is now done (`ros2/patches/pr/PR_READY.md`).

- **Reconciliation before writing anything:**
  - **`s1_static_reference` is a pre-0007 bag and was missed in the interim block's regeneration.** Six fixtures carry `obstacle_detector/msg/Obstacles`; five were regenerated, that one was not (md5 unchanged, timestamp 2026-08-01 15:15 vs 23:15–23:20 for the others). It is harmless *today* only because its three consumers (`test_projection_replay`, `test_hw_source_contract`, `test_dlio_wiring`) subscribe to the live stack's obstacle topics and never to the bag's `/sim/gt_obstacles`. Recorded in `test_fixtures/README.md` with the condition under which it stops being harmless.
  - **The interim block recorded only two of the five new md5s.** All five are now in `test_fixtures/README.md` alongside the pre-0007 values: `s1_surveyed 889d4c34…`, `s2_cross_05 3fde31e7…`, `s2_cross_08 3174aa0b…`, `s3_swarm 4282942c…`, `s4_occlusion 88511685…`. Every fixture the A/B harness consumes is confirmed post-0007.
  - **The §17.3 A/B harness still runs from one command** (`phase4_ab_run.sh`), verified before it had a walking robot to point at.
  - **`State_RLBase`'s entry preconditions, read from `deploy`'s source rather than from the button mapping:** the transitions are *not* hard-coded, they are DSL strings in `deploy/robots/g1/config/config.yaml` (`Passive→FixStand: "LT + up.on_pressed"`, `FixStand→Velocity: "RT + A.on_pressed"`), compiled in `FSMState`'s constructor and evaluated at 1 kHz against `lowstate->joystick`. `State_RLBase::enter()` requires nothing beyond a loaded policy and a live `lowstate`: it writes the policy's gains, calls `env->robot->update()` and spawns the policy thread, which calls `env->reset()` then steps at `step_dt = 0.02`. There is no settling-time or joint-position precondition — but there IS an exit condition, `bad_orientation(env, 1.0)`, which drops back to Passive past 1 rad of tilt. `mode_machine` must match (the sim's `G1Bridge` sets 5 for the 29-dof scene, so it does).

---

#### Workstream B — walking. **Bar MET.** Both safety numbers exist.

**The buttons never reached the FSM's decision, and the reason was arithmetic, not wiring.** The prompt's instruction to prove the chords arrive before debugging anything upstream was the right call, and it paid immediately.

New instrument, `simulate/build_ros2/fsm_button_probe` (source in `dpcbf_ros_adapter/tools/`, built by the sanctioned `simulate/CMakeLists.txt`): it runs the FSM's own `g1::subscription::LowState::update()` and the FSM's own compiled transition DSL — `deploy/include/unitree_joystick_dsl.hpp`, fed the condition strings out of `deploy`'s config — at 1 kHz against a live `rt/lowstate`. `deploy/` is read, never modified. It exits non-zero if a configured transition never fires, so it is a gate, not just a debugger.

With the profile the interim block documented (`L2,up` at one breakpoint), the probe shows the keys **arriving correctly** — `LT` and `up` each 601 pressed ticks and one `on_pressed` edge, receiver-side axis peak 0.999999 — and **all seven** transition predicates firing **zero** times. The mechanism, from the trace:

```
 t          tick   LT_p  LT_val    up_p  up_op   Passive->FixStand
 7.705     15000    0    0.03       1     1       0    <- the only edge
 7.727     15022    1    0.5037     1     0       0    <- 22 ticks later
```

`up` is a `Button<int>`, so `up.on_pressed` is true for exactly one tick. `LT` is an `Axis`, and `g1_sub.h` feeds it the raw L2 *bit* through `smooth = 0.03` against `threshold = 0.5`, needing `1 − 0.97ⁿ > 0.5` ⇒ **n = 23 ticks**. The AND can never be true. Setting `LT.smooth = 1.0` on the *sender* (the interim block's fix) cannot help — it is the receiver's filter that lags, and only the sender's `pressed` flag crosses the wire. Staggering the chord (axis half ≥ 25 ms before the button) fires both predicates exactly once, and `g1_ctrl` logs the transitions live. Evidence: `evidence/walking/fsm_chord_timing.md`, both probe logs, the trace window.

**Two further things a headless walking run needs, both learned the expensive way and both now in `README.md` and the harness:** start `g1_ctrl` *before* the simulator (it blocks in `wait_for_connection()`, so State_Passive is live at sim t≈0 and cannot lose the race to the profile's chords — one smoke run was lost exactly that way); and **hold the robot up until the policy is running**. `enable_elastic_band` is a hoist you toggle with GLFW key `9`, which is why every closed-loop session so far ran suspended; the other half of the operator procedure — lowering it — had no scripted form. `main.cc` (sanctioned file) gains `UNITREE_MUJOCO_BAND_LENGTH` and `UNITREE_MUJOCO_BAND_RELEASE=<t0>[,<ramp>]`, a sim-time linear ramp to zero, inert unless set and leaving stock behaviour untouched. Lowering onto FixStand's PD hold pose topples the robot every time (measured: tilt 0.02 → 0.33 rad within 3 s of the ramp starting); lowering it *after* `R2+A`, onto an actively balancing policy, works.

**Robot state reached, stated against the bar** ("stable locomotion long enough to run the A/B; standing, twitching or being dragged is not walking"): pelvis 0.771–0.788 m against a 0.793 m standing height, **maximum |roll,pitch| 0.046 rad**, 110 s of continuous locomotion, 47–52 m of path at 0.43–0.48 m/s under a scripted forward+yaw profile, `bad_orientation` never firing. That is walking.

- **The payload — §17.3 collision rate and min clearance, closed loop, for the first time.** `phase4_ab_run.sh` pins the robot at (0,0,0) and can only score command tracking; a robot that never moves has no collision rate. New one-command harness `test/walk_ab_run.sh` runs the real thing — policy walking, DPCBF in the 1 kHz seam, perception live — once per DPCBF mode on an identical seeded field, and `walk_ab_probe.py` measures, at 100 Hz from the simulator's own ground truth, `clearance(t) = min_i(|p_rob − p_i| − r_rob − r_i)` and, separately, falls. **The two are deliberately not merged into one "collision rate":** a negative clearance is a DPCBF *margin* violation (`r_rob` is a disc about the pelvis; the limbs reach outside it), while body contact against a 1.5 m cylinder shows up as a fall. Fields are the live `DynamicObstacleManager` arena at four densities — **not** the S1–S4 fixture bags, which are bag replays with the simulator out of the loop and cannot produce these numbers at all — with W4 the Phase-4 90-obstacle arena verbatim, seed 42. `dpcbf/` untouched: the per-field configs are copies in a scratch run tree.

  | field | mode | walked | fell | GT in p_max | margin viol. | viol. time | min clearance | p50 clearance |
  |---|---|---|---|---|---|---|---|---|
  | W1 sparse static | oracle | 110.4 s | no | 0.72 | **0** | 0 % | +0.020 m | 0.979 m |
  | W1 sparse static | estimated | 110.3 s | no | 0.71 | **0** | 0 % | +0.193 m | 0.899 m |
  | W2 sparse crossing | oracle | 110.3 s | no | 0.87 | 1 | 1.88 % | −0.135 m | 0.651 m |
  | W2 sparse crossing | estimated | 110.3 s | no | 1.48 | 5 | 5.78 % | −0.156 m | 0.970 m |
  | W3 swarm (run A) | oracle | 110.4 s | no | 2.79 | 7 | 3.04 % | −0.238 m | 0.981 m |
  | W3 swarm (run B) | oracle | 8.5 s | **53.5 s** | 10.24 | 9 | 18.56 % | −0.248 m | 0.075 m |
  | W3 swarm | estimated | — | **30.0 s** | — | — | — | — | — |
  | W4 Phase-4 arena | oracle | 110.3 s | no | 4.41 | **0** | 0 % | +0.021 m | 0.537 m |
  | W4 Phase-4 arena | estimated | 25.4 s | **70.4 s** | 7.52 | 5 | 12.91 % | −0.215 m | 0.374 m |

  **The W4 estimated fall at 70.4 s is a collision, not a stumble:** clearance crosses zero at t = 69.83 s and the robot pitches (0.078 → 0.093 → 0.177 → 0.268 → 0.397 rad) and drops (z 0.773 → 0.640 m) over the next 0.3 s. It walked into a cylinder. **The baseline it was scored against did not survive repetition — see the next item, which withdraws the "oracle is clean" reading of this table.**

- **Repetition, and two claims withdrawn because of it.** Two further W4 pairs were run on the identical seeded field (`evidence/walking/repeats/`). All three oracle runs and all three estimated runs:

  | run | oracle | estimated |
  |---|---|---|
  | matrix | 0 violations, no fall, min cl +0.021 m | 5 violations, min −0.215 m, **fell 70.4 s** (collision) |
  | r2 | **11 violations**, no fall, min −0.067 m | **fell 37.9 s** (pre-window) |
  | r3 | 2 violations, **fell 115.5 s**, min −0.217 m | **fell 35.3 s** (pre-window) |

  1. **WITHDRAWN — "oracle mode is clean on W4."** Across three runs of the same configuration the oracle arm took **0, 11 and 2** margin violations and fell once. The clean first run was luck.
  2. **WITHDRAWN — the margin-violation ordering on W4.** The oracle arm's own spread (0–11) brackets and exceeds the single scoreable estimated value (5); at n = 3 the counts cannot separate the arms, and quoting 0 vs 5 would be quoting noise.
  3. **SURVIVES — the fall ordering.** Estimated fell **3 of 3** (35.3, 37.9, 70.4 s); oracle **1 of 3**, and latest of all (115.5 s). Consistent across every pair — but two of the three estimated falls precede the first walking command and are the band-transfer class that `diag/` shows a later release largely removes, so **this must be re-run on the `34,6` bring-up before it carries weight**.

  **The delta metric, by contrast, IS reproducible.** The same three W4 oracle runs give tracked→GT p50 **104.8 / 99.8 / 103.9 mm** and p90 **321.7 / 293.4 / 280.9 mm** — ±3 % and ±7 % — over 27–32 k samples each, against margin-violation counts of 0 / 11 / 2 on those very runs. A percentile over ~30 000 per-obstacle comparisons averages the whole run; a violation count is a handful of rare events at the mercy of where a chaotic trajectory went. **The two halves of this block's walking result therefore carry very different weight: the shadow deltas and everything resting on them — including the rig-vs-density verdict on S3 — reproduce and are solid; the collision rate and clearance distribution are a first sample and gate nothing yet.**

  **What W4 therefore establishes** is that the estimated arm is markedly less stable, that at least one of its failures is a genuine collision, and that **the baseline is not clean either** — not a clean oracle-vs-estimated separation. Ordering across fields at the strength the data supports: W1 0 → 0, W2 1 → 5, W4 (0, 11, 2) → (5, –, –). Estimated is never *better* than oracle in any run, but nothing here is repeated enough to be a gate.

- **Two caveats stated up front, because they bound how far these numbers travel.** (i) **The comparison is distributional, not paired.** The arms share seeds and command profile, but diverge the instant the filters differ, so they do not meet the same obstacles at the same times; Phase 4 bought pairing by pinning the robot, and a moving robot cannot have both. (ii) **One 110 s run per cell is a sample, not a measurement.** W3 oracle was run twice on the identical seeded field and disagreed qualitatively — run A walked the full window at 2.79 obstacles inside `p_max`, run B wandered into a corner of the 8×8 arena at 10.24 and fell at 53.5 s. The loop is chaotic and live runs are wall-clock nondeterministic (H-10). The W3 cell therefore supports no verdict and is reported, not concluded from; the *ordering* above is the robust part, the individual counts are not. Repeats of the W4 pair are in `evidence/walking/repeats/`.

- **The oracle arm is not clean either, and after repetition this is the block's firmest closed-loop result — upstream of perception.** In W2, W3 and W4 the arm that sees exact ground truth with zero latency and zero association error still lets the pelvis disc overlap an obstacle disc by up to 0.25 m, in **4 of the 5 oracle runs that produced a scoring window**, and it fell once. **DPCBF's guarantee is on the commanded velocity; the realised base motion is the RL policy's, and the policy's velocity-tracking error is modelled nowhere in the chain.** Two further unmodelled terms: `r_rob = 0.30 m` is a disc about the pelvis while a walking G1's feet and arms swing outside it, and `s = 1.05` is sized against command error, not tracking error. No amount of perception improvement fixes this, and neither the offline A/B nor the containment sweep could have surfaced it.
- **The re-priced shadow deltas, and a control Phase 4 never ran.** `tracked → nearest-GT` per tracked circle inside `p_max`, capped at 0.5 m (Phase 4's NN cap — so **p50 and p90 are comparable with Phase 4's; p99 is cap-limited here and is not**). Rather than compare against a number recorded four phases ago, a **rig control** was run: identical field (90 obstacles, seed 42), identical code, identical probe, band never lowered and no `g1_ctrl`, so the robot hangs limp exactly as it did when the Phase-4 shadow deltas were taken (measured pelvis z 1.338 m, confirming the hoist). Its clearance/violation numbers are meaningless — a body hanging from a band under nobody's command — and only its delta row is a measurement.

  | condition | count | speeds | GT in p_max | p50 | p90 |
  |---|---|---|---|---|---|
  | **W4 rig control**, suspended, this code | 90 | 0–0.8 | 8.97 | **343 mm** | **470 mm** |
  | W4 oracle, same arena, **walking** | 90 | 0–0.8 | 4.41 | **105 mm** | **322 mm** |
  | W3 oracle, swarm, walking (run A) | 20 | 0.2–0.8 | 2.79 | 120 mm | 341 mm |
  | W2 oracle, sparse moving, walking | 6 | 0.5–0.8 | 0.87 | 116 mm | 311 mm |
  | W1 oracle, sparse static, walking | 6 | 0 | 0.72 | 80 mm | 209 mm |
  | *Phase-4 record, same field, suspended* | 90 | 0–0.8 | — | *94 mm* | *186 mm* |

  Decomposition (Δp50 / Δp90): **obstacle motion** (W1→W2, same 6 obstacles, same seed) **+36 / +102 mm**; density 6→20 (W2→W3) +4 / +29; density 20→90 (W3→W4) −15 / −19; **rig removed** (rig control → W4 walking, same field and code) **−238 / −148**.

- **The rig is a large artifact — larger than Phase 4 recorded, and I could not reconcile the two.** On identical code and field the suspended robot's p50 is **343 mm against walking's 105 mm**: the rig is 3.3× worse. **Phase 4's 94 / 186 mm was not reproduced**, and the gap is 3.6×. Two candidate causes, neither now verifiable: the numbers came from a different measurement path (the adapter's internal shadow accumulator at query time, against this ROS-level probe), and the limp robot's *attitude* is an uncontrolled variable — this control saw |roll,pitch| up to **0.456 rad (26°)**, and a 26°-tilted LiDAR projected into a 0.15–1.60 m band above `base_footprint` slices cylinders at heights the circle fit was never meant for. Phase 4 recorded the rig's ±0.45 m swing and free yaw but not its tilt. Stated plainly: **the Phase-4 live-arena figures — the 0.686 est/oracle ratio and the 94/186 mm deltas — should be treated as measured on an uncontrolled rig and superseded by the walking numbers, not averaged with them.** Reconciling or retiring them is a follow-up.

- **The rig-vs-density verdict on S3's 0.8235: it is a REAL PERCEPTION LIMIT, not an evaluation artifact. Phase 7 remediation, not re-measurement.** Three legs, in increasing independence from anything measured this block:
  1. **By construction S3 contains no rig at all.** The S1–S4 fixture bags were recorded with a constant grounded `qpos` (`make_scenario_scene.py::grounded_qpos` — pelvis z = 0.793, identity quaternion, never updated). The suspension rig existed only in the Phase-4 *live-arena* session, which is where the 0.686 and the 94/186 mm came from. **S3's 0.8235 was measured with a perfectly stationary sensor: there was never a rig artifact in it to remove.** This leg settles the question on its own and depends on no measurement taken this block — it is a fact about how the fixture was generated, and the prompt's premise that S3 might be partly a rig artifact is refuted by it.
  2. **Among the things that ARE in S3, the dominant term is obstacle motion, not density.** Holding count and seed fixed and only starting the obstacles moving costs +36 mm at p50 and +102 mm at p90 — more than the entire density span from 6 to 90 obstacles. That is tracker coast-and-re-acquire plus extractor arc truncation on moving targets: exactly the Q-2 residual class Phase 4 identified and could not cover with a fixed inflation.
  3. **S3's number is a LOWER bound.** Its sensor is perfectly still; a walking sensor is 2–4× worse on the same scene class. A hardware capture with the robot walking should be expected to be worse than S3, not better, and must not be compared with the fixture numbers as if they were like-for-like.
- **Walking visual evidence (carried since Phase 1): produced, offscreen.** `test/walk_overlay.py` + `walk_overlay_run.sh` render the same three layers RViz would show — `/scan` in the odom frame, `/tracked_obstacles`, `/sim/gt_obstacles`, plus the base pose and its `p_max` horizon — sampled live from the running walking stack through **matplotlib Agg**: no display, no RViz, no GL. Say which was used, so: **offscreen render from a live session**, not a bag replay and not an interactive screenshot. It shows more than the screenshot would have, because tracked and GT circles are overlaid per obstacle rather than summarised as a percentile. `evidence/walking/walk_overlay.png`.

---

#### G1 — σ's units. Resolved, and the interim block's diagnosis was half wrong.

- **σ has always been in metres; the VALUE was never set.** The tracker's KF is textbook (`kalman.h`, `tracked_circle_obstacle.h`): `C = [1 0]`, the measurement `y` is a circle-centre coordinate **in metres**, so `R(0,0)`, `P(0,0)` and `SurfaceSigma() = sqrt(max(var_x,var_y)+var_r)` are m², m² and m by construction. There is no missing scale factor and there never was. What is wrong is the number: `measurement_variance: 1.0` asserts a **one-metre 1σ** LiDAR measurement, P-2 seeds `P(0,0) = R` from it, every track is born at σ = 1.0 m and relaxes to ≈0.58 m — exactly the p50 the interim block measured — and `k_σ` silently absorbs the difference. The interim block's conclusion that neither 0.287 nor H-8's 2.748 is a sigma multiplier **stands**; its reason ("a unitless knob") does not, and the correction changes the fix from a unit conversion to a calibration. Recorded because it is the general lesson: **1.0 m² is a perfectly well-typed variance** — no unit check, no type system and no dimensional analysis can tell a correct m² from one half a million times too large. Only data can.
- **The measurement path now exists and is exercised.** `phase4_obstacles_dump.py` records `/raw_obstacles` (the KF's actual measurements — without them a robot-day capture cannot derive R at all), and new `test/measure_measurement_variance.py` clusters detections per physical target on a static scene and reports the scatter about each target's mean, so the *systematic* circle-fit bias — which `fixed_inflation` already covers and which would be double-counted here — is absorbed by the mean rather than folded into R. **Measured on `s1_surveyed`: pooled position variance 1.775e-06 m² (1σ = 1.3 mm), i.e. the shipped 1.0 is 563 431× too large.** But 1.3 mm is not a candidate value either — the simulated Mid-360 is an analytic raycast with no range-noise model, so that scatter is a discretisation artefact, and setting R to it would tell the tracker to believe every measurement absolutely and ignore its own process model. **Both endpoints are wrong for shipping, which is exactly why what ships is machinery rather than a number.**
- **Decision: measured and recorded, NOT shipped from sim data.** `measurement_variance` is a *sensor* property. Deriving it from the simulator's noise model and shipping it would re-tune the tracker — R sets how far every position innovation is trusted, so it moves tracked positions, velocities, T5's RMSE and every containment number downstream — on the strength of a number describing a simulator rather than a Mid-360. That is the same "looks authoritative, isn't" failure this gap is about, committed in the other direction. 5B block 1 sets the shipped value from hardware, as a joint recalibration with the P-2 tuning, re-verified against S1/S2 containment afterwards.
- **What ships instead is that σ can no longer be used silently while it is wrong.** (i) `calibrate_k_sigma.py` **refuses** to emit a `k_σ` when pooled σ p50 exceeds `SIGMA_PLAUSIBLE_MAX = 0.25 m`, printing why instead of an authoritative-looking number — so a robot-day data drop lands in a correct path or a loud one, never a plausible wrong one. (ii) `gating.h` gains `kSigmaPlausibleMax` and a `Stats` struct; `Apply()` counts `sigma_implausible` and `sigma_clamped` **separately**, because the remedies are opposite (fix R, versus trust the cap), and the node emits throttled warnings naming the offending σ. (iii) The unit is now written where the value lives — Appendix A, `obstacle_detector.yaml` and the 5B checklist all say `m²` and say what 1.0 asserts.
- **Containment re-verified, and it is unchanged where it had to be.** Full `phase4_ab_run.sh` re-run after patch 0009: **s1_static, s2_cross_05 and s2_cross_08 all 100.000 % at F = 0, to the last recorded digit**, and the §17.3 offline ratios reproduce (S1 0.9565, S2 1.0000 / 0.9978, S4 0.9456). S3 moves *slightly better* — containment 67.195 → 70.895 % at F = 0 and A/B ratio 0.8235 → **0.8265** — which is the predicted direction, since 0009 can only admit circles and a swarm has merged arcs that used to exceed the old 0.43 m cut. Nothing regressed. (The tracker itself is untouched: `measurement_variance` is unchanged, so this re-verification is about 0009, and G1 ships no behaviour change to re-verify.)
- **Consequence for P-3, stated plainly:** the interim block's "coverage 92.5 % → 99.9 %, S3 85.2 % → 99.9 % with `k_σ = 0.287`" was computed on σ ≈ 0.58 m. Once R is real, σ falls by orders of magnitude and `k_σ` must rise by the same factor to buy the same inflation. Rescaling does not change a correlation, so the *shape* of the remedy is unaffected and its *magnitude* is simply uncalibrated — and `corr(F_req, σ) = +0.339` remains the number that decides whether per-track covariance is the right observable at all.

---

#### G2 — obstacles at r ≳ 0.40 m were silently dropped. Hypothesis confirmed; gate aligned; drops made observable.

- **The arithmetic hypothesis is confirmed, and it was checked rather than adopted.** `detectCircles()` does `circle.radius += p_radius_enlargement_` **before** `if (circle.radius < p_max_circle_radius_)`, so the kept condition is `fit < 0.60 − 0.17 = 0.43 m`; with the interim block's measured fit bias (`+0.084·r` on a fully visible cylinder) that is **≈ 0.397 m of true radius**. Verified against every row of the 210-configuration analytic-ray sweep: the rule `keep ⟺ fit + 0.17 < 0.60` reproduces **all 17 non-detections and all 43 detections at r ≥ 0.4 with no exceptions**. One further sweep miss (r = 0.10 m, d = 4 m, 50 % arc) is correctly *not* explained by it — that one is `min_group_points`.
- **The cutoff is not a single radius, which is the safety-relevant part.** Truncating the visible arc shortens the chord, shrinks the fit and brings the same physical object back under the cut: r = 0.40 m at 2 m is invisible at full arc and visible at 90 % arc; r = 0.50 m needs 25 % occlusion to appear. Range matters too through `sqrt(1−(r/d)²)` — r = 0.40 m is seen at 1.0 and 1.5 m and dropped at 2.0, 2.5 and 3.0 m. The failure is not "objects above X are missing", which an operator could work around, but "objects near X flicker as the robot moves", which looks like a tracking problem and is not one.
- **Three mismatches with §9.6, all now corrected in the body.** (1) the gate was on `fit + radius_enlargement`, so `max_circle_radius` really meant *"max radius minus radius_enlargement"* and **making the conservative safety inflation larger silently made the sensor blinder** — that backwards coupling is the actual defect; (2) the published `true_radius` is the *fitted* radius, +8.4 %·r biased, not the true one; (3) `safety_obstacle_filter`'s own `true_radius > 0.60` gate was therefore **effectively unreachable** — the extractor could never emit a `true_radius ≥ 0.43`. Half wrong, and the new counter is what showed it: post-patch that gate fired once on a `true_radius` of 0.602 m, because the extractor bounds its own *fit* while the filter sees a value that has been through the tracker's KF. It catches a KF-drifted radius, which is a real thing the extractor cannot.
- **Decision: align the semantics** (fork patch **0009**), not raise the threshold and not document the accident. The plausibility test ("this arc is too big to be an obstacle") and the safety margin ("report obstacles slightly larger than measured") are different ideas and one had been made a function of the other. The patch moves the enlargement to after the gate; downstream is untouched, because `circle.radius` still carries the enlargement when pushed, so `mergeCircles()` and `true_radius = radius − enlargement` behave exactly as before for every circle already being kept — the change can only *admit* circles that were previously dropped. `mergeCircles()`'s own test is left arithmetically alone (its quantity is a deliberate over-bound, and a refused merge leaves both parents alive) and only made visible.
- **Resulting sensing limit, in metres of true radius: ≲ 0.55 m** (`0.60/1.084`, fully visible; partial occlusion only extends it). **Verified after the patch on the same harness:** r = 0.30 … 0.55 m now all detect at 2 m and 3 m (r = 0.40 m was dropped at both before), the last kept fit is 0.5796 m just under the 0.60 threshold, and 0.60/0.65 m drop — an empirical limit of 0.55 m against the predicted 0.553 m. In physical terms this pipeline can see standing and walking people (r ≈ 0.20–0.30 m) comfortably, and seated people, office chairs, bins, cones and crates up to ~1.1 m across; it **cannot** see structural pillars, pallet stacks, vehicles or any wall — by design, because they are not circles and the gate exists to stop the extractor handing DPCBF a fictitious three-metre constraint fitted to a wall arc. That last row is a limitation of the circle model, not of the threshold, and belongs with §9.7's Phase-7 work.
- **The prop bound is reconciled and relaxed.** The interim block's `r ≥ 0.30 m but ≤ ≈0.32 m` was a consequence of the accidental 0.397 m cut plus margin for its arc dependence. **Withdrawn.** The constraint on props is now only `r ≥ 0.30 m` (to exercise the radius-dependent bias) and `r ≤ 0.55 m`. `phase5b_checklists.md` block 0 updated.
- **Every drop is now observable, which was required whatever the threshold.** Extractor: `dropped_large_circles_` and `refused_large_merges_`, each with a throttled `WARN` naming the offending radius, the threshold and the running count. Filter: `Stats{stale_messages, dropped_large_radius, sigma_clamped, sigma_implausible, radius_max_dropped, sigma_max_seen}` filled by `Apply()` and reported by the node, with four new unit tests including one asserting that passing no `Stats` leaves behaviour byte-identical. Both fire live (`evidence/interim/g2_drop_warnings.log`): *"dropped a circle of fitted radius 0.720 m … an obstacle this large is INVISIBLE downstream; 22 dropped since start"*. Before this block that whole sequence was silent.
---

#### Workstream A — aarch64. **Not resumed, as scoped.** No arm64 box appeared, so `tools/build_target.sh`, `tools/arm64_emulate.sh`, `tools/diagnose_ament_export_libraries.py`, patch 0008 and the robot-day first-hour triage list remain the deliverables, and the risk — the on-target build, with the `ament_cmake` `_lib` cache-shadowing bug as its sharpest edge — carries forward unchanged. The withdrawn workaround stays withdrawn. An hour on any arm64 box still retires it.

#### Housekeeping

- **PR-ready artifacts prepared, nothing pushed.** `ros2/patches/pr/` holds six minimal, single-purpose patches split out of the workspace series, each **verified by `git ls-remote` + `git apply --check` against a fresh clone of the upstream tip**, with its commit message, reproduction and the measurement that justifies it (`PR_READY.md`). Three of the four target repos are pinned *at* their upstream tip, so those apply directly. Two findings came out of doing it:
  - **Patch 0002 (`pcl_ros` output QoS) must NOT be offered upstream — it is obsolete.** The `ros2` tip (`e264aff1`) has since added `QosOverridingOptions` to `pub_output_`, so the reliability is a parameter (`qos_overrides./output.publisher.reliability: best_effort`) and no source change is needed. Our pin (`9d078eb`, 2.6.1) predates it. The action is on our side: when the pin moves past `e264aff1`, delete patch 0002 and set the override. This is the second time "verify against the checkout, not the doc" has paid, in the mirror-image direction — verify against the *tip*, not against your own fork.
  - The genuinely general fixes are extracted from their bundles: `upstream-01` is P-1's `begin()++` double-count alone (27 lines, not the componentisation/QoS/TF patch it was carried inside), `upstream-02` is P-2's two-point initiation covariance alone (31 lines), `upstream-05` is the `/livox/imu` `frame_id` fix alone (not the `package.xml` workspace plumbing). `upstream-03` (0008) and `upstream-04` (0006) were already minimal; `upstream-06` is the new 0009. 0007 stays fork-only (it changes a public message). The `ament_cmake` finding is an **issue report, not a PR**, and deliberately carries no patch: a fix that changes link lines on a robot without being understood is worse than the failure, which is why the local workaround was withdrawn in the first place.
- **MCAP/foxglove installed.** The recording configs still default to sqlite3; switching them is now an ordinary follow-up rather than an operator ask.
- **`phase5b_checklists.md` updated:** the prop bound relaxed with G2's mechanism and the real limit in metres of true radius; block 1's σ section rewritten with the corrected units, the two ways to derive R in preference order, and the warning that R is not a display parameter; block 5's calibration note pointing at the new refusal; and a new **block 9** on what walking changed about the A/B procedure — that the fixture bags have a perfectly stationary sensor and are therefore a lower bound, that bring-up on hardware is the same hold-then-lower procedure the scripted profile encodes, and that `walk_ab_run.sh` needs no bags so it is the cheapest way to re-verify the chain on the target.

- **Files added:** `ros2/patches/0009-obstacle-detector-p4-fit-radius-gate-and-drop-observability.patch`; `ros2/patches/pr/{PR_READY.md,upstream-01…06}`; `dpcbf_ros_adapter/tools/fsm_button_probe.cc`; `g1_perception_bringup/test/{walk_ab_run.sh,walk_ab_probe.py,walk_ab_report.py,walk_scenarios.py,walk_state_probe.py,walk_overlay.py,walk_overlay_run.sh,measure_measurement_variance.py}`; `g1_perception_bringup/config/walk_profile.txt`; `ros2/evidence/walking/**` and `ros2/evidence/interim/{g1_sigma_units.md,g2_radius_gate.md}`.
- **Files modified (inside `ros2/`):** `setup_external.sh` (patch 0009), `README.md`, `test_fixtures/README.md`, `doc/phase5b_checklists.md`, `config/{obstacle_detector.yaml,safety_obstacle_filter.yaml}`, `safety_obstacle_filter/{include/safety_obstacle_filter/gating.h,src/safety_obstacle_filter_node.cpp,test/test_gating.cpp}`, `g1_perception_bringup/test/{phase4_obstacles_dump.py,calibrate_k_sigma.py}`.
- **Complete outside-`ros2/` edit list:** `simulate/src/main.cc` (the band-lowering ramp + its two env hooks) and `simulate/CMakeLists.txt` (the `fsm_button_probe` target) — both already-sanctioned files — plus this §21 entry and the §9.6 / Appendix-A corrections in the body. **`dpcbf/` byte-for-byte identical to `f111cfa`** (verified: `git diff f111cfa -- dpcbf/` empty), and `deploy/`, `src/`, `scripts/` untouched — `deploy/include/unitree_joystick_dsl.hpp` is *included* by the probe, never modified. `simulate/src/ros2_bridge.{h,cc}` not needed. Confirmed by `git status`.
- **Test counts: 85 → 89**, over the project packages: **0 errors, 0 failures, 2 skipped** (dpcbf_ros_adapter 18, safety_obstacle_filter 12 → **16**, g1_perception_utils 5, g1_description 2, g1_perception_bringup 41 with the 2 skips, sim_mjlidar_bridge 7). One trap worth recording, because it looks alarming: `colcon test-result --all` from the workspace root reports *1466 tests, 815 failures* — every one of them a **lint test in an external package** (`rmw_cyclonedds_cpp`'s `xmllint` cannot fetch `package_format3.xsd` without network, and so on). Those have never been in this project's count; scope the query to the project build dirs, as the earlier entries did. New: four `safety_obstacle_filter` observability tests (large-radius drop counted with the largest offender, stale message counted, implausible σ flagged separately from a legitimately capped one, and `Apply()` with no `Stats` byte-identical to `Apply()` without). `t1_oracle_equivalence` (simulate CTest) still **PASS** after the `main.cc` band changes — the 1 kHz seam is untouched by them. Per-test DDS domains preserved.

- **Evidence artefacts:** `ros2/evidence/walking/` — `walking_ab.md` (the full analysis), `ab/` (9 runs + the rig control: metrics, traces, per-run logs, `walk_ab_report.txt`), `repeats/{r2,r3}` (the two extra W4 pairs + `repeats_report.txt`), `diag/` (the late-band discriminator), `fsm_chord_timing.md` + the two probe logs + the trace window, `walk_overlay.png`, `test_counts.txt`. `ros2/evidence/interim/` — `g1_sigma_units.md`, `g1_measurement_variance.txt`, `g2_radius_gate.md`, `g2_cutoff_sweep_post0009.csv`.

- **Issues discovered / root causes:**
  - The chord-timing bug above — arrival and satisfaction are different questions, and only an instrument that evaluates the *predicate* can tell them apart.
  - **The band-lowering transient is a failure mode of the harness, not of the product**, and it took a discriminator to say so: two dense-field estimated runs went down at t ≈ 30 s, 2 s after the band finished lowering and 10 s before the first walking command. Lowering 10 s later turned one of them into 73 s of walking. The W4 estimated fall at 70.4 s is *not* in that class and is a real collision.
  - **`walk_ab_probe.py` crashed with `IndexError` when a run fell before the scoring window opened** — the one run whose trace was most worth reading was the one that produced none. Fixed twice over: the fall is now detected across the whole trace rather than after the settle filter, and the raw base trace is always written.
  - **I clobbered W3 oracle's own artefacts** by re-running a scenario pair when I only meant to re-run one arm. `WALK_MODES` now exists; the lost run's numbers are transcribed into `W3_oracle_runA_transcribed.json` with that provenance stated, and the replacement run is the file-backed one.
  - `measure_measurement_variance.py` assumed a `center` key the dump does not use; `circle_fit_sweep.py` needs an extractor running and silently reports "NO DETECTION" for every row when there is none — which is indistinguishable from a real total drop-out, and briefly looked like patch 0009 had broken everything.
  - `set -u` plus `source /opt/ros/humble/setup.bash` aborts on unbound `AMENT_TRACE_SETUP_FILES`.

- **Remaining limitations:**
  - **The closed-loop metric has enormous run-to-run variance and this block does not have enough samples to beat it.** W3 oracle run twice on the identical seeded field gave a clean walk and a fall; W4 oracle run three times gave 0, 11 and 2 margin violations and one fall. Only the W4 fall ordering (estimated 3/3, oracle 1/3) is consistent across repeats, and even that is contaminated by the bring-up transient. **Nothing in the walking A/B is a gate yet.** The harness is the deliverable; the numbers are a first sample.
  - The walking A/B is a **distributional, not paired**, comparison — the arms diverge as soon as the filters differ.
  - The A/B matrix was taken with the **pre-0009** extractor and the **24,4** band release; 0009 is strictly permissive for these fields (max true radius 0.30 m ⇒ fit ≈ 0.33 m, far under the 0.43 m cut) but the merged-arc path could differ, and 34,6 is the better bring-up. Re-running the matrix on both is a named follow-up rather than a silent change.
  - **The Phase-4 rig deltas (94/186 mm) were not reproduced** by a controlled rig run on the same field and code, which measured 343/470 mm. The discrepancy is unexplained; the limp robot's *tilt* is an uncontrolled variable that Phase 4 did not record.
  - The tracked→GT p99 in this harness is truncated by the 0.5 m association cap and is not comparable with Phase 4's.
  - `measurement_variance` is still the inherited 1.0 m². The tripwires make that observable; only hardware can fix it.
  - No aarch64 build or test run; every number here is sim.

- **Lessons learned:** *arrival is not satisfaction* — the buttons were received perfectly and the transition was still arithmetically impossible, and no topic-level probe could have shown that; only running the consumer's own predicate could. *A single run of a chaotic closed loop is an anecdote* — the first W4 pair read as a clean oracle-vs-estimated separation and two repeats dissolved it, while the same repeats left the delta percentiles intact to ±3 %; which half of a result survives repetition is not obvious in advance, and the only way to find out is to repeat it before reporting it. *A well-typed number can be nonsense* — `measurement_variance: 1.0` is a valid m² variance half a million times out, and nothing in the type system, the units or the code review could catch it, which is why the fix that ships is a tripwire and not a value. *A plausibility gate must not consume a safety margin* — folding `radius_enlargement` into the `max_circle_radius` test made the pipeline blinder every time it was made more conservative, which is exactly backwards, and it hid behind an *intermittent* symptom because occlusion restores the detection. And *the control you did not run is the number you cannot use*: the Phase-4 rig figure looked like it settled the S3 question until a controlled rig run disagreed with it by 3.6×.
- **Open follow-up items:** carried — org forks + upstream PRs (now **0003–0009**, with 0002 **withdrawn as obsolete against the upstream tip** and the `ament_cmake` bug still a separate upstream *issue*), Orin CPU + latency benchmark, P-5, Q-4's own session, Mid-360 lidar→IMU offset verification, SDK2-linked recorder for Q-5, R-4 levers, OSQP maxIter under swarm load, aarch64/target build. **Closed this block:** walking evidence and the interactive-RViz screenshot (offscreen render, carried since Phase 1); §17.3 collision rate and min-clearance (carried since Phase 4); the rig-vs-density question on S3; MCAP/foxglove apt (installed); `max_circle_radius` vs prop radius (patch 0009, bound relaxed); `measurement_variance`'s units (they were always m²; the *value* is now a named 5B task with a measurement path and two tripwires). New —
  - **DPCBF's guarantee does not cover the policy's velocity-tracking error**, and the oracle arm violates the margin because of it. This is upstream of perception and is the single most consequential new finding for the safety case: sizing `s`, `r_rob` or the horizon against *tracking* error, or closing the loop on realised velocity, is a design question for §10 rather than a tuning one.
  - **Re-run the walking matrix** on the post-0009 extractor and the `34,6` bring-up, with **≥3 repeats per cell — and treat that as required, not optional.** The three W4 pairs run this block are the evidence for why: the oracle arm alone ranges 0–11 margin violations on an identical seed. A safety claim needs a distribution, not a run.
  - **Attribute the W4 collision** to a specific tracking failure: the probe should dump the per-obstacle association at the moment clearance crosses zero, which it currently does not.
  - **Reconcile or retire the Phase-4 rig deltas** (94/186 mm vs a controlled 343/470 mm on the same field and code).
  - **`s1_static_reference` is still a pre-0007 bag**; regenerate it before anything reads `/sim/gt_obstacles` from it.
  - **`simulate/build_ros2/` is untracked and not in `.gitignore`** — noted rather than fixed, because `.gitignore` is outside this phase's sanctioned edit list.
  - Switch the record configs to MCAP now that the packages exist.

- **Suggested next step: 5B is still not a capture session, and its preconditions are unchanged** — Q-1 unanswered, the target build unproven, props unsourced (though the prop constraint is now much easier: `r ≥ 0.30 m`, up to ≈0.55 m). The largest remaining risk is unchanged in identity and unchanged in size: **the on-target build**, and an hour on any arm64 box still retires it.

  But the ranking of the *other* work has changed. **The most consequential thing this block found is not a perception result** — it is that the oracle arm, with exact ground truth and zero latency, still lets the robot's safety disc overlap an obstacle by up to 0.25 m — in 4 of 5 scoreable runs, up to 11 violations in a single 110 s run, and one fall — because DPCBF filters the *commanded* velocity and nothing in the chain models the policy's tracking error. Repetition made this finding *stronger* while dissolving the perception comparison it was found alongside. Perception improvements cannot touch that, and it bounds what the whole subsystem can promise. If a no-robot session comes before 5B, the highest-value use of it is **§10: quantify the policy's velocity-tracking error under the filtered command, and decide whether `s`, `r_rob` and `latency_horizon` should be sized against it** — with the walking harness that now exists, that is a measurement rather than a research question. The second-highest is re-running the walking matrix with repeats, so the safety numbers this block produced stop being single observations.

### 2026-08-02 — Interim block 3 (still between 5A and 5B): the operator runbook

- **Objective:** not a §18 phase and not a feature block. One deliverable — `ros2/doc/operator_runbook.md`, a procedure document that lets an operator with no context sit down at this machine and *run* what Phases 0–5A and the two interim blocks built, see it, and reproduce any §21 number, without reconstructing a command line from nine phase entries. The bar was not "write it" but **"execute it"**: every command in the order the document presents it, with its actual output as the expected observable. That bar is what produced the findings below; a runbook assembled from the prose would have contained none of them.

- **OPERATOR INPUT NEEDED** (unchanged except item 4):
  1. ~~sudo apt for MCAP/foxglove~~ — DONE in the previous block; both packages verified present this session.
  2. **Q-1's full answer set for 5B** (still blocking): G1 variant and onboard PC; Mid360 factory mount y/n; robot network and IPs; sudo + internet on the Orin; its shipped Ubuntu/ROS state.
  3. **arm64 hardware in the lab** — unchanged, and still the cheapest way to retire the on-target build risk.
  4. **Props: `r ≥ 0.30 m` and `r ≤ 0.52 m`.** The previous block's `≤ ≈0.55 m` is **refined, not withdrawn** — see the range-dependence finding below. Checklist block 0 updated.
  5. **Fork/PR authorisation** — unchanged; everything short of pushing is done (`ros2/patches/pr/PR_READY.md`).

- **Coverage, stated as the headline because it is the honest measure of the deliverable: 27 of the 34 runnable blocks were executed this session; 7 were not and each says so at its own block; 1 more is partially verified and says that.** Not executed, with reasons: the from-scratch build (§2.2) and the `simulate`/`deploy` builds (§2.4) — both trees were already built and every binary in them was exercised, but a cold rebuild was out of budget; the one-line `bringup.launch.py` (§3.3) — every live block here drives the four launches separately, as every harness in the tree does; `foxglove_bridge` (§4.5) — no browser client to connect with, and starting the bridge proves nothing on its own; `fsm_button_probe` standalone (§6.3) — the walking run it gates transitioned correctly on the first attempt, so there was no failure to point it at; the T6 staleness drill (§6.6); and `hw_source_stub.py` invoked by hand (§8.2) — the two launch tests that drive it passed inside the suite. Partial: the OpenCV `dpcbf_visualizer` (§4.4) — the window was observed on a headless Xvfb rendering its `Waiting for MuJoCo state...` frame while the seam was demonstrably running, and I could not confirm it *refreshes* there; it is documented as a real-display tool.

- **`ros2/README.md`: split, not extended.** It had accumulated two jobs across six phases — workspace provenance and an operational trap list — and was doing both badly for a reader who arrives with a task. Decision: **README keeps provenance** (what is built from source and why, the nine recorded patches with one line each on what they fix, the `tools/` story for other machines, the layout) and **every procedure moved to the runbook**, wholesale rather than duplicated. What moved: the environment-variable block, all eleven runtime notes, the "Run (sim)" section, and the gate-tests table. It went 268 → 115 lines, and the two documents now share no fact, which is the point: two files that disagree about `CYCLONEDDS_URI` in six months would be worse than either alone.

---

#### Defects found by running the documented paths

**Six, of which four had been broken for multiple phases and none of which was visible from reading.**

- **The README's own `colcon test` line does not work in this workspace, and its failure is louder than its consequence.** It was the *first* command executed this session. `colcon test` without `--merge-install` exits immediately with *"The install directory 'install' was created with the layout 'merged'"* — and then the documented follow-up, `colcon test-result`, reads whatever XML is already on disk and cheerfully prints the **previous** run's green summary. I recorded 89/0/2 from a run that had not happened. **Broken since Phase 0** (the workspace has always been merge-install) and invisible because every phase entry ran a working variant while the README carried the broken one, and because the stale-green follow-up makes the mistake self-concealing. Fixed in the README and written up as a trap in the runbook.
- **The T10 invocation path in the README was wrong** — `install/t10_dds_coexistence/lib/t10_dds_coexistence/t10_smoke` against the real `install/lib/t10_dds_coexistence/t10_smoke`, because `--merge-install` flattens the layout. Same age, same cause: nobody re-ran the documented string.
- **`perception.rviz` had no display bound to `/obstacles_safe`.** `viz.launch.py` has published that relay since Phase 4, so **the one stream with safety meaning — what DPCBF is actually told — was the only one you could not see.** Fixed (fifth display, `SafeObstacles (Phase 4)`) and verified on a screenshot: the inflated red layer is now visible around each tracked circle.
- **Two config comments were wrong in opposite directions about the same fact.** The previous block's entry states that the `m²` unit "is now written where the value lives — Appendix A, `obstacle_detector.yaml` and the 5B checklist". Appendix A and the checklist have it; **`obstacle_detector.yaml` carries no unit and no warning at all** — the file was modified in that block, but only its G2 half landed. Worse, **`safety_obstacle_filter.yaml` still carried the *withdrawn* G1 diagnosis** ("the tracker's covariance is NOT in m^2 … a unitless knob"), which that same block corrected. A reader consulting the config would have got the retracted explanation. Both fixed to say the same thing: the maths was always metres, the **value** `1.0 m²` is what is wrong, and it must come from hardware.
- **The shadow run-tree — a hard precondition of `phase4_live_session.sh` — was documented nowhere.** The script's own header says "see ros2/README.md runtime notes"; the README never gave the commands. `walk_ab_run.sh` builds its own tree inline, which is why this survived: the harness that needed it most already had it. Now runbook §3.2, with the reason the tree exists (the simulator resolves `config.yaml` relative to its own executable and `dpcbf/config/` one level above, so it cannot be reconfigured without editing tracked files, and `dpcbf/` is frozen).
- **`phase4_live_session.sh` hardcodes `DISPLAY=:1`** where `walk_ab_run.sh` and `walk_overlay_run.sh` both use `${DISPLAY:-:1}`, so it opens the simulator on the operator's desktop rather than honouring an Xvfb. **Recorded, not fixed** — it changes harness behaviour, which is outside a documentation block's remit. The runbook says so at the block.

#### Two measurements taken because a documented claim did not reproduce

- **The G2 sensing limit is RANGE-dependent, and `0.55 m` is only the 2 m figure.** The previous block records "r = 0.30 … 0.55 m now all detect at 2 m and 3 m". Re-run on the same harness (`circle_fit_sweep.py`, analytic rays into the real extractor, full arc): **r = 0.55 m detects at d = 2 m — fit 0.5796, just under the 0.60 gate — and is DROPPED at d = 3 m and d = 4 m**; r = 0.52 m detects at 2, 3 and 4 m (fit 0.5624 / 0.5699). The mechanism is the block's own geometry read in the other axis: the visible chord, hence the fit, grows with range through `sqrt(1−(r/d)²)`, so the same prop passes the gate close in and fails it further out. It is the same intermittency G2 described for occlusion, in range instead. **Props are surveyed at 1–3 m, so the operator constraint is `r ≤ 0.52 m`**; 0.55 m is the close-range-only number. `phase5b_checklists.md` block 0 corrected; rows in `evidence/runbook/circle_fit_limit_by_range.csv`.
- **`s1_static_reference` was still the pre-0007 bag** (a named follow-up from the previous block). **Regenerated:** 28.885 s, 98.1 MiB, 289 clouds, md5 `3466e2cf54b1f3374497796c33fbcbbb` (pre-0007: 27.87 s / 92.9 MiB / `d372a619…`), and its consumer re-verified — `test_projection_replay` gives 289 `/scan` frames over 28.8 s sim time → 10.00 Hz, 0.0 % drop, T9 288/0 misses. One thing worth recording because it cost two runs: **match the record wall-clock to the realtime factor, not to the target duration.** With the perception container also running the sim goes at ≈0.44 RT and 30 s of wall clock bought a 13.1 s bag; without it the factor is ≈1.0. Noted in `test_fixtures/README.md`.

#### The RViz screenshot, carried since Phase 1, is closed

Phases 1–4 recorded the interactive RViz screenshot as blocked ("GNOME blocks unattended screenshots"); interim block 2 closed the *evidence* half with an offscreen matplotlib render and explicitly said which it was. The screenshot itself was never blocked in the way recorded: **run RViz against a private `Xvfb` and grab that display** — `Xvfb :77`, `DISPLAY=:77` with the software-GL variables, then `PIL.ImageGrab.grab(xdisplay=':77')` (Pillow 9.0.1, the system one). No compositor, no desktop, nothing on the operator's screen. RViz renders at **31 fps** under software GL there, which is interactive enough to be worth using and not only screenshotting. Evidence: `evidence/runbook/rviz_bag_replay.png` — the `s1_surveyed` replay with all five layers: cloud arcs with occlusion shadows behind every obstacle, three tracked cylinders sitting on their GT circles with uid labels, and the newly-visible safe inflation. *`ImageGrab` grabs the whole display, so give RViz its own — pointing it at `:1` captures the operator's desktop.*

#### Measured this session (all of it reproduction, none of it new capability)

| block | result | cost |
|---|---|---|
| full test suite | **89 tests, 0 errors, 0 failures, 2 skipped** (`test_bringup_sim` — no live sim; `hw_config_check` — placeholder IPs) | **4 min 54 s**, twice, unchanged before and after this block's edits |
| bag replay, projection probe | 10.0 Hz on all three topics, drop 0.0 %, T9 289/0, cloud→scan p50 0.60 / p95 1.04 ms | 44 s |
| bag replay, full chain | cloud→safe p50 **0.76** / p95 **1.28** ms, container **2.9 %** of one core | 44 s |
| live sim stack | cloud→safe p50 **1.48** / p95 **2.18** ms, container **4.6 %**, sidecar **30.3 %**, 451 clouds / 45 s, T9 449/0 | ~1 min to first frame |
| **walking, W1 oracle** | window 35.2 s, path 15.14 m at 0.43 m/s, pelvis min **0.777 m**, tilt max **0.044 rad**, no fall, **0 margin violations**, min clearance **+0.019 m**, tracked→GT p50 **74.2 mm** | **1 min 29 s** |
| walking overlay, offscreen | 5 panels from 2747 frames over 50.0–80.5 s | 1 min 32 s |
| offline A/B, S1–S4 | ratios **0.9565 / 1.0000 / 0.9978 / 0.8265 / 0.9456** — every one identical to the post-0009 record; containment at F=0 s1/s2 100.000 %, s3 70.895 % (record: 70.895 %) | 3 min 48 s |
| calibrators | pooled position variance **1.775e-06 m²** (1σ 1.3 mm), "563431× too large"; `calibrate_k_sigma` **refuses** at σ p50 583.6 mm | seconds |
| T1 oracle equivalence | **PASS** | 3.79 s |
| T2 wall occlusion / T3 pattern envelope | **PASS** / **PASS** (17 608 expected hits, 0 through; −7.2123…+52.1640°, 99.782 % strict) | **0.11 s / 0.09 s** |
| T10 DDS coexistence | **PASS**, one `libddsc.so.0.10.2` in the maps | 10.4 s |
| driver with no device | `bind failed` / `Init lds lidar fail!`, **no `/livox/*` topic at all**, survived SIGINT — `pkill -9` required | 25 s |
| `hw_config_check.py` | exit **2**, naming the unassigned placeholder IP | instant |

The two gates that cost **0.1 s each** are worth calling out on their own: T2 and T3 are the raycast-correctness and pattern-envelope guarantees the entire simulated evaluation rests on, and there has never been a cost-based reason not to run them.

- **Helper script added: none.** The one plausible candidate — the walking startup — already has `walk_ab_run.sh`, and it works: one command, 89 s, correct FSM transitions, band procedure and all. What was missing was never a script, it was the *explanation* of the four constraints it encodes (`g1_ctrl` first, staggered chords, hold-then-lower, `34,6` over `24,4`), which is now runbook §6.1 with each constraint's failure signature. The shadow run-tree (§3.2) is documented as explicit commands rather than wrapped, so the pieces stay runnable by hand.
- **Files added:** `ros2/doc/operator_runbook.md` (1191 lines); `ros2/evidence/runbook/{rviz_bag_replay.png, walk_overlay_reproduced.png, circle_fit_limit_by_range.csv}`.
- **Files modified (all inside `ros2/`):** `README.md` (rewritten, 268 → 115 lines), `doc/phase5b_checklists.md` (prop bound), `test_fixtures/README.md` (`s1_static_reference` regenerated + the realtime-factor note), `g1_perception_bringup/rviz/perception.rviz` (+`SafeObstacles` display), `g1_perception_bringup/config/{obstacle_detector.yaml, safety_obstacle_filter.yaml}` (the two wrong σ comments), and the regenerated `test_fixtures/s1_static_reference/` bag (gitignored).
- **Complete outside-`ros2/` edit list: this §21 entry, and nothing else.** `dpcbf/` byte-for-byte identical to `f111cfa` (`git diff f111cfa -- dpcbf/` empty), and `deploy/`, `src/`, `scripts/` have zero diff — verified, not asserted. `simulate/` was not touched this block (the `M` on `main.cc` and `CMakeLists.txt` in `git status` is the previous block's uncommitted work).
- **Test counts: 89 → 89.** No tests were added; a documentation block should not need any. The suite was run twice — once as the first executed command of the session and once after every edit — with identical results, which is the check that the config-comment, RViz-layout and fixture changes are inert.
- **Lessons learned:** *a documented command that nobody re-runs decays silently, and the decay can be self-concealing* — the broken `colcon test` line hid behind a follow-up command that prints the previous run's success, so the documentation was wrong AND the wrongness produced a green result. *"Verify against the checkout, not the doc" applies to your own log too*: the previous entry's claim about where the `m²` unit was written did not survive `grep`, and one of the two files named carried the retracted version of the finding. *A visualisation config is documentation*: the layout had silently disagreed with the launch file for two phases, and the effect was that the only stream with safety meaning was invisible by default. And *reproduction is cheap and worth budgeting for* — every §17.3 ratio and every gate came back identical, which is what makes the one number that did **not** reproduce (the 0.55 m radius limit at 3 m) worth acting on rather than explaining away.
- **Open follow-up items:** carried unchanged — org forks + upstream PRs (0003–0009, 0002 withdrawn as obsolete, the `ament_cmake` bug as a separate issue), Orin CPU + latency benchmark, P-5, Q-4's own session, Mid-360 lidar→IMU offset verification, SDK2-linked recorder for Q-5, R-4 levers, OSQP maxIter under swarm load, aarch64/target build, re-running the walking matrix with ≥3 repeats on the post-0009 extractor and the `34,6` bring-up, attributing the W4 collision, reconciling or retiring the Phase-4 rig deltas, switching the record configs to MCAP, and `simulate/build_ros2/` not being in `.gitignore`. **Closed this block:** the interactive-RViz screenshot (carried since Phase 1 — it was never actually blocked); `s1_static_reference`'s pre-0007 status; the undocumented shadow run-tree. **New —** `phase4_live_session.sh` should honour `$DISPLAY` like its two siblings (one-word change, deliberately not made here); the runbook's seven unexecuted blocks are worth closing opportunistically, the cheapest being the T6 drill and the one-line bringup; and **the from-scratch build time (~40 min) is still an estimate, not a measurement** — the next machine that builds this workspace should time it and put the number in the runbook.
- **Suggested next step: unchanged, and the runbook does not displace it.** 5B's three preconditions are still unmet (Q-1, the target build, props — now `r ∈ [0.30, 0.52] m`). The largest risk is still the on-target build and an hour on any arm64 box still retires it. The highest-value no-robot work is still the previous block's verdict: **§10 — quantify the policy's velocity-tracking error under the filtered command and decide whether `s`, `r_rob` and `latency_horizon` should be sized against it**, since the oracle arm violates the margin for reasons no perception work can touch. That measurement is now *cheaper* than it was: the walking harness is documented end to end, a 70 s single-arm run costs 89 s of wall clock, and §6.5 says what a good run looks like — so the loop of "run, read the trace, change a constant, run again" is a thing an operator can now do without reconstructing it first.

---

### Interim block 3 — Live 2-D overlay: estimated vs ground-truth obstacles, and the DPCBF constraint boundary

**Operator ask, carried unchanged:** *"2D로 estimated obstacle + 실제 obstacle 약간
겹쳐 보이게끔, + DPCBF로 parabola 어디로 생기는지도 라이브로 비주얼라이즈"* — three
things, live, during a walking run: (1) estimated obstacles drawn on top of
ground truth, deliberately overlapping, so per-obstacle error is readable at a
glance rather than as a percentile in a probe's JSON; (2) the DPCBF constraint
boundary, where it sits relative to those obstacles, updating live; (3)
top-down 2-D, because that is the plane the filter reasons in.

#### The headline: the "parabola" is in VELOCITY space, and that changed the design

This was the first task and it invalidated the plan's constraint layer, exactly
as the plan allowed for. Derived from the running source, not the name:

- The barrier is `h = x̃ + A·(λ·ỹ² + k_μ·d_safe)` with `A = √(s²−1)/r_safe` and
  `λ = k_λ·d_safe/v_safe` — `dpcbf/src/dpcbf_safety_filter.cpp:404`. So `h = 0`
  is `x̃ = vertex_x − curvature·ỹ²`, and `Filter()` itself already names those
  two coefficients at `dpcbf_safety_filter.cpp:590-591`.
- **`(x̃, ỹ)` is the relative velocity `v_obs − v_robot` rotated into the
  obstacle line of sight** (`dpcbf_safety_filter.cpp:385-386`). Both axes are
  **m/s**. `dpcbf/include/dpcbf/dpcbf_safety_filter.h:47` states it outright:
  *"Frozen-parameter h=0 parabola in the obstacle Line-of-Sight velocity frame."*

It is "both" only weakly: the parabola's *shape* is parameterised by workspace
quantities (`d_safe`, `r_safe`) so it deforms as the robot moves, but the plane
it lives in is velocity. **Drawing it in `odom` would be a category error**, and
the operator was asked before anything was built. Decision taken: **workspace
layers in `odom`; the parabolas in their own `dpcbf_velocity_plane` TF frame
beside the robot, axes labelled m/s on every card. No world-frame parabola.**

Two corollaries the plan did not anticipate, both from source:

- **The existing OpenCV `dpcbf_visualizer` is a trap, not a reference.** It
  *does* draw a world-frame curve (`dpcbf_visualizer.cpp:204-225`) — but it is
  the same velocity-space curve scaled by `velocity_arrow_seconds = 1.0` and
  anchored at the **robot**, i.e. metres = (m/s)×(1 s). A one-second-lookahead
  diagram, not an envelope around the obstacle. Its bounding config values
  (`world_parabola_lateral_limit: 1.0`) are m/s on a pane scaled in metres.
  Porting it into RViz would have reproduced the error on a metre grid, where
  it reads as far more authoritative.
- **There is no single constraint boundary.** The shipped config runs
  `ecbf_enabled: true` *and* `slack_enabled: true`, so every selected obstacle
  contributes a second row — a distance barrier `|p|² − (r_rob+r_obs)²`
  (`dpcbf_safety_filter.cpp:438-480`) whose zero set **is** honest world
  geometry — and both families are soft. The operator chose to draw both. The
  eCBF circle is the only constraint geometry that belongs on the metre grid.

Also settled from source, because drawing envelopes around obstacles the filter
ignored is the second way an overlay lies: **selection is not nearest-first.**
The gate is centre-to-centre `distance ≤ p_max` (radius *not* subtracted), and
the shipped `obstacle_priority: 1` orders by **closing alignment** descending
with distance only as tie-break, then truncates to `default_num_constraints`.

#### The validation gate — two numbers, deliberately not one

Blurring "is the math right" and "are the inputs good" into a single tolerance
would have hidden both. `boundary_check` therefore has two modes.

**`math` (the CTest, `dpcbf_boundary_recomputation`) — BIT-EXACT.** It replays
every recorded `Filter()` call, runs the **frozen** `DpcbfSafetyFilter` as the
oracle and the overlay's own `dpcbf_boundary.h` on identical inputs, and
compares selected set, order, and every coefficient:

```
BOUNDARY PASS: 38402 ticks checked (of 38402 records, stride 1), 214085 selected-obstacle rows
  selected set + order: exact on every tick
  max |d vertex_x|  = 0 m/s   |d curvature| = 0 s/m   |d h| = 0   |d eCBF h| = 0 m^2   (tol 1e-12)
```

Every delta is **identically zero** — not "within tolerance". The stated `1e-12`
is a round-off budget, not a tuned threshold: both sides perform the same
operations in the same order on the same doubles, so exact equality is the
expected outcome and the tolerance was never widened to make anything fit.
Cost **3.07 s**, stride 1 (no sampling), SKIP 77 without the fixture.

**`join` (a measurement, not a gate) — this is where the honest limit is.**
Join rule: **nearest-preceding capture tick** (last 1 kHz record with
`t ≤ t_safe`), max gap 0.15 s, **no interpolation** — the capture's obstacle set
is piecewise-constant between perception frames, so interpolating would invent
states the filter never saw. Keyed on the sim-time stamp of the
`/obstacles_safe` frame the overlay consumed. On a W4 walking run (35.8 m, no
fall): all 1186 records joined, 0 dropped; **selected set identical on 63.3 %**
of ticks; Jaccard disagreement mean 0.061 (≈94 % membership overlap);
`|d body speed|` mean **0.048** m/s; `|d vertex_x|` mean 0.191 m/s;
`|d curvature|` mean 0.234 s/m.

**Why, and why no tuning closes it.** The seam gets the 1 kHz instantaneous
pelvis twist; the overlay must **differentiate `/odom` pose at 100 Hz**, because
`sim_mjlidar_bridge` publishes **no twist** — `MjState` carries no `qvel` by
design (`bridge_node.py:9`). *The plan's §2 assumed `/odom` twist was
available; it is not.* Membership is close; it is the **ordering** that
diverges, because ranking by closing alignment reshuffles on millimetre-per-
second differences. Consequence, applied: the workspace layers are
authoritative (read straight off the topics) and **the `selected` and `vel_*`
layers ship labelled INDICATIVE in the marker text itself, on screen** — the
`calibrate_k_sigma.py` precedent, not a quiet caveat in a doc.

#### A tuning decision that was wrong until it was measured

The first join reported mean `|dv|` **0.554** m/s and 48.9 % set agreement, and
an offline sweep of the smoothing constant appeared to show smoothing did not
matter. **Both were taken from a run in which the robot fell at 40.9 s** (path
0.569 m — it never walked); a thrashing pelvis is not a walking one. Re-swept
against a genuine walking window: mean `|dv|` **0.032** m/s at `τ=0`, 0.032 at
0.02, 0.049 at 0.05, **0.077 at 0.15** — smoothing *costs* accuracy here,
because `/odom` pose is exact and lag, not noise, dominates. The shipped
`vel_tau_s` was changed **0.15 → 0.02** (τ=0's mean, better p95: 0.055 vs
0.094 m/s). Recorded because the failure mode generalises: **check `fell_at_s`
before quoting any number derived from a walking capture** — now a trap-index
entry.

#### Cost, against the §17.4 budget

Measured over 45 s of a walking run, % of one core, from `/proc` directly:

| Process | W1 | W4 (90 obstacles) | §17.4 record |
|---|---|---|---|
| **`dpcbf_overlay`** | **1.44 %** | **2.02 %** | new |
| `component_container` | 4.68 % | 5.31 % | 4.6 % |
| `sim_mjlidar_bridge` | 27.75 % | 35.92 % | 30.3 % |
| `rviz2` | 119 % | 191 % | — |

Container and sidecar sit on their budget, so **the overlay moves neither** — it
is a separate process outside the perception container, publishes at 10 Hz (not
GT's 50 Hz) and subscribes only. The genuinely expensive thing on this machine
is **RViz under software GL at 119–191 % of a core**, which is the pre-existing
cost of looking and not of this layer.

#### Executed versus written-but-unverified

**Executed, on this machine, with output recorded:** the §0 source derivation;
the `math` gate (38 402 ticks); 7 new unit tests; the full suite twice; **six
live walking runs** (W1 ×2, W4 ×4 — W4 fell at band release in 3 of 4, W1
walked every time); the `join` on four captures; the Xvfb + `PIL.ImageGrab`
screenshot path; the **bag-replay GT-absent degradation**, verified by topic
inspection rather than by eye — with `/sim/gt_obstacles` excluded the banner
reads `GT UNAVAILABLE - estimated only` and the `gt`, `error`, `unpaired_gt`
and `unpaired_tracked` namespaces are **absent** while every estimated layer
stays; the CPU measurement; `dpcbf/` and `deploy/` verified untouched.

**Written but NOT verified:** the **T6 staleness rendering**. The purple
colour-shift and `*** stop: SET RETAINED ***` banner are implemented and driven
by `/dpcbf/status`'s level, and the banner was observed rendering its live
(`fresh`) and its no-status (`?`) forms — but the drill itself (§6.6) was not
run, so the DEGRADE/STOP appearance is **unverified**. Marked as such here and
in the runbook. Also unverified: hardware behaviour (no hardware), and the
`error_magnify` debug path beyond its always-visible warning marker.

#### Defects found by running the documented paths

- **The documented rebuild trap bit, and cost a live run.** `viz.launch.py` and
  `perception.rviz` are **installed, not read from source** (runbook §2.3). Both
  were edited and neither rebuilt, so the run came up with the old four relays
  and no overlay node — **silently**, because a launch argument that selects
  nothing looks exactly like one that is off. The trap was already documented
  for *configs*; it is now documented for *launch files* too, with this
  failure signature, because "four relays and no error" is not obviously a
  build-staleness symptom.
- **`proc_cpu.py` matched the measuring process itself.** Its own argv carries
  every name it was asked to search for, so each target reported a phantom
  extra pid — the same self-match class as the `pkill -f component_container`
  trap in §3.4. Found because the first CPU report said `2 pid(s)` for a
  single-process node. **Fixed** (exclude `os.getpid()`); all figures above are
  post-fix.
- **RViz's committed layout was unreadable in 2-D.** With `TopDownOrtho` the
  point cloud covers every other layer from above, and the `Odometry` display's
  100-arrow trail fans across exactly the area the new layers occupy. Three
  defaults changed for legibility, all one tick away and all documented:
  `LivoxCloud` and `RawObstacles` off, `Odom` `Keep: 100 → 12`.

#### Deliverables and counts

- **Node:** `g1_perception_utils/src/dpcbf_overlay_node.cpp` — placed beside
  `obstacles_marker_relay` rather than in a new package, because it is the
  sixth marker layer alongside the four relays and a package for one executable
  buys nothing. Outside the perception container. Read-only w.r.t. the running
  system: it subscribes, publishes one `MarkerArray`, one visualization TF, and
  an optional JSONL. **No new workspace dependency** — `yaml-cpp` is already
  built against by `dpcbf`, `obstacle_detector` and ROS 2's own vendor package;
  it is used so the overlay reads the **same** `dpcbf_config.yaml` the simulator
  loaded instead of re-declaring the parameters as ROS defaults, which is how a
  viewer starts lying confidently. The `dpcbf_config` parameter is **required**;
  the node refuses to start without it.
- **Shared math:** `dpcbf_ros_adapter/include/dpcbf_ros_adapter/dpcbf_boundary.h`
  (+ `_config.h`) — one implementation consumed by both the node and the gate,
  so the picture and its proof cannot drift. Same arrangement as `dpcbf_seam.h`.
- **Gate:** `dpcbf_ros_adapter/tools/boundary_check.cc`, built from
  `simulate/CMakeLists.txt` because it needs the frozen dpcbf static libs —
  the established `t1_replay.cc` / `ab_eval.cc` arrangement, source in the
  adapter package to respect the outside-`ros2/` budget.
- **Harnesses:** `walk_overlay_rviz_run.sh` (one command: shadow tree, bring-up,
  overlay, capture, JSONL, screenshots, CPU — **2 min 22 s**) and `proc_cpu.py`.
- **Docs:** runbook **§4.6** (preconditions → command → layer/z table → how to
  read a correct picture → what failure looks like → the gate → cost), plus
  §4.1's layout note, the §5.2 gate table, §11.1 topics and **five new trap-index
  entries**.
- **Evidence:** `ros2/evidence/overlay/rviz_dpcbf_overlay_w4.png` (W4 walking:
  five blue `selected` rings, cyan `MISS` badges, orange discs on green rings,
  three velocity cards with the shaded `h < 0` half), plus the gate and join
  outputs and the run metrics.
- **Test counts: 89 → 97**, 0 errors, 0 failures, 2 expected skips —
  `dpcbf_ros_adapter` 18 → 26. Plus the new **CTest
  `dpcbf_boundary_recomputation`** in `simulate/build_ros2` (all 3 there pass),
  which is outside the colcon count exactly as `t1_oracle_equivalence` is.
- **Complete outside-`ros2/` edit list: `simulate/CMakeLists.txt`** (one target,
  one `add_test`) **and this §21 entry.** `dpcbf/` and `deploy/` are
  byte-for-byte untouched — `git status --porcelain dpcbf deploy` empty,
  verified not asserted. `perception.launch.py` gained no conditional (D4); the
  overlay is GT-consuming and lives in `viz.launch.py` behind `overlay:=on|off`.

#### Follow-ups

**New:** run the **T6 drill with the overlay up** to verify the staleness
rendering (the one written-but-unverified piece; cheap, and §6.6 is already an
unexecuted block worth closing anyway); consider publishing `qvel` in `MjState`
or twist in `/odom`, which would make the constraint layer authoritative rather
than indicative — but it changes a message every fixture bag carries, so it is a
deliberate decision, not a tidy-up; the overlay cannot plot **commanded vs
filtered velocity** inside the cards because neither is on any topic (the seam
holds them), which would need a small diagnostic publisher; and W4's **fall at
band release in 3 of 4 attempts** deserves attribution — it is a harness
question, not a product one, but it makes W4 expensive to use for evidence.
**Carried unchanged:** everything in the previous block.

**Suggested next step — unchanged, and now better instrumented.** 5B's three
preconditions are still unmet (Q-1, the target build, props `r ∈ [0.30, 0.52] m`);
the arm64 box still retires the largest risk in an hour. The highest-value
no-robot work is still **the oracle arm's margin violations under walking**, and
this overlay is now the natural instrument for it: the `selected` layer shows
*which* obstacles became QP rows and in what order, the velocity cards show
`h` going negative *as it happens*, and the eCBF circle shows the distance
barrier being breached in metres. One caveat to carry into that work, and it is
the reason the gate was split in two: **under `estimated` the constraint layers
are indicative, not authoritative** — for oracle-arm forensics, drive them from
the 1 kHz capture through `boundary_check`, where the geometry is bit-exact,
and use the live view to know where to look.
