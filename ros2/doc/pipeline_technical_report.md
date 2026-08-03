# DPCBF perception subsystem — technical report

**Scope.** What is actually running on branch `dpcbf_perception_ros2`, reconciled
against the architecture document. Every claim below was read out of source,
launch files, package manifests, `deps.repos`, the patch files, and the YAML —
not from the architecture doc, which is cited only where it is the *design
intent* and is flagged wherever the implementation has diverged from it.

**Method disclosure.** This is a **source audit**. Nothing was executed for this
report: no builds, no launches, no tests. Every *number* attributed to a
measurement is quoted from [`operator_runbook.md`](operator_runbook.md) or
`../evidence/`, which record a session dated 2026-08-02. §11 separates what was
measured from what is only implemented.

---

## 0. One-paragraph summary

A MuJoCo raycast (sim) or a Livox Mid-360 (hardware) produces a 10 Hz
`PointCloud2`. Five composable components in one process crop the robot out of
it, flatten it into a 2-D `LaserScan` in a ground-aligned frame, fit circles,
Kalman-track them, and inflate them into a safety set. A **C++ library linked
directly into the simulator binary** — not a node — hands that set to the frozen
DPCBF QP at 1 kHz through a wait-free double buffer, constant-velocity
extrapolating positions to the exact query instant and ramping the command to
zero when the data goes stale. The whole thing is oracle-by-default: unless
`UNITREE_DPCBF_MODE` says otherwise, the QP eats MuJoCo ground truth and the
perception stack is scenery.

---

## 1. Package provenance and ownership

### 1.1 Unmodified upstream (imported, pinned, not patched)

| Package | Upstream | Pin | Local path | Role | Runs as |
|---|---|---|---|---|---|
| `cyclonedds` | `eclipse-cyclonedds/cyclonedds` | `9995905b` (tag 0.10.2) | `ros2/src/external/cyclonedds` | the one DDS impl in the process (R-3) | shared library |
| `cyclonedds-cxx` | `eclipse-cyclonedds/cyclonedds-cxx` | `2a372d2c` (tag 0.10.2) | `.../cyclonedds-cxx` | C++ binding for SDK2 | library, `COLCON_IGNORE`d, built via SDK2 |
| `rmw_cyclonedds` | `ros2/rmw_cyclonedds` | `fa8831b9` (humble) | `.../rmw_cyclonedds` | ROS 2 RMW | shared library |
| `unitree_sdk2` | `unitreerobotics/unitree_sdk2` | `21d0a3b2` (main) | `.../unitree_sdk2` | `rt/lowcmd`, `rt/lowstate`, `RecurrentThread` | static/shared lib into `simulate` and `g1_ctrl` |
| `pointcloud_to_laserscan` | `ros-perception/pointcloud_to_laserscan` | `59bf996f` (humble) | `.../pointcloud_to_laserscan` | 3-D → 2-D projection | **composable component** in `perception_container` |
| `MuJoCo-LiDAR` | `TATP-233/MuJoCo-LiDAR` | `287b5012` (main) | `.../MuJoCo-LiDAR` | Mid-360 rosette raycast (`LivoxGenerator('mid360')`, `MjLidarWrapper` cpu backend) | **pure-Python library**, `COLCON_IGNORE`d, injected on `PYTHONPATH` by [`source_sim.launch.py:17-31`](../src/g1_perception/g1_perception_bringup/launch/source_sim.launch.py#L17-L31) |
| `Livox-SDK2` | `Livox-SDK/Livox-SDK2` | `f5d9375f` (tag v1.3.1) | `.../Livox-SDK2` | driver's transport SDK | plain CMake, `COLCON_IGNORE`d, driven into the merged prefix by `livox_sdk2_vendor` |

`Livox-SDK2` is pinned at the **tag**, not master, deliberately —
`deps.repos:78-84` records master as v1.3.1 + one Mid-360S/Ubuntu-24.04 commit
this robot does not need.

### 1.2 Patched upstream

Patches are applied by [`setup_external.sh`](../setup_external.sh); each is
`git apply --check`ed first, so re-running is idempotent.

| Package | Upstream / pin | Patches | What the patch changes at runtime | Runs as |
|---|---|---|---|---|
| `pcl_ros` (from `perception_pcl`) | `ros-perception/perception_pcl` `9d078ebf` (tag 2.6.1) | **0002** | `Filter`'s `output` publisher Reliable → `SensorDataQoS().keep_last(max_queue_size_)` ([`filter.cpp:174-177`](../src/external/perception_pcl/pcl_ros/src/pcl_ros/filters/filter.cpp#L174-L177)). Without it the CropBox output does not match the best-effort subscribers downstream. | **composable components** (`pcl_ros::CropBox`, `pcl_ros::VoxelGrid`) |
| `obstacle_detector_2` | `harmony-eu/obstacle_detector_2` `6ff7ac48` (humble-devel) | **0003, 0004, 0007, 0009** | see below | **composable components** (`obstacle_detector::ObstacleExtractorComponent`, `::ObstacleTrackerComponent`) |
| `livox_ros_driver2` | `Livox-SDK/livox_ros_driver2` `13eb05e4` (tag 1.2.6) | **0005, 0008** | 0005 adds `package.xml` (upstream ships `package_ROS1.xml`/`package_ROS2.xml` and expects `build.sh`, which `rm -rf`s the workspace's build/install trees) + `colcon.pkg` carrying `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`, and fixes `InitImuMsg` hardcoding `frame_id="livox_frame"` on `/livox/imu`. 0008 declares the build-order dependency on `livox_sdk2_vendor`. | standalone node `livox_ros_driver2_node` |
| `direct_lidar_inertial_odometry` (DLIO) | `vectr-ucla/direct_lidar_inertial_odometry` `c8acc371` (feature/ros2) | **0006** | cloud subscription Reliable → SensorData depth 1; `/odom` Reliable depth 10. Upstream's Reliable subscription cannot match the best-effort cloud publishers this stack uses everywhere — DLIO would show a connected topic and receive nothing. | standalone node `dlio_odom_node` (+ optional `dlio_map_node`) |
| `unitree_dds_wrapper` | `Agnel-Wang/unitree_dds_wrapper` `9dc107d2` | **0001** | restores the simulator-facing joystick API (`LowState::joystick`, `WirelessController::joystick`, `UnitreeJoystick::update()`) the pinned SHA lacks | **header-only**; installed by `unitree_dds_wrapper_vendor` |

The four `obstacle_detector_2` patches, in order of what they do to behaviour:

- **0003 (P-1)** — componentization (upstream ships plain executables only);
  `/scan` subscription → `SensorDataQoS` ([`obstacle_extractor.cpp:120-123`](../src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L120-L123));
  obstacle publishers Reliable depth 5; **TF lookup at the scan stamp** rather
  than latest ([`obstacle_extractor.cpp:587`](../src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L587)),
  which is what makes replay deterministic; and a real upstream bug — a
  `begin()++` that double-counted the first point of every first group and
  corrupted its circle fit.
- **0004 (P-2)** — the tracker becomes **measurement-driven**: no wall timer,
  `predict()` + `correct()` on `/raw_obstacles` arrival with `dt` taken from
  header stamps ([`obstacle_tracker.cpp:155-170`](../src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L155-L170)),
  output stamped with the measurement time, two-point initiation seeding
  velocity as a finite difference, `P` initialised from `R` (`diag(R, 2R/dt²)`),
  radius-residual weight 0.3 in the association cost.
- **0007 (P-3)** — adds `float64[3] covariance` to `CircleObstacle`
  (`[var_x, var_y, var_r]`, filled from the per-axis KFs' `P(0,0)` at
  [`obstacle_tracker.cpp:737-739`](../src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L737-L739)).
  **This is D6's one sanctioned message change and it invalidates every fixture
  bag that carries `Obstacles`.**
- **0009 (P-4)** — `max_circle_radius` is tested against the **fit** instead of
  `fit + radius_enlargement`. Before it, the shipped 0.60/0.17 really cut at a
  0.43 m fit, so *raising the safety inflation made the sensor blinder*. Plus
  drop counters and throttled warnings on both silent drop paths.

### 1.3 Custom ROS 2 packages (written here)

All under `ros2/src/g1_perception/`.

| Package | Role | Runs as |
|---|---|---|
| `dpcbf_ros_adapter` | **the seam.** `ObstacleSource` (modes, subscription, `/dpcbf/status`), `ObstacleBuffer` (wait-free double buffer + extrapolation + staleness ladder), `dpcbf_seam.h` (axes↔command), `axis_command_map.h`, `filter_io_log.h`, `dpcbf_boundary.h` (overlay's re-implementation of the barrier geometry) | **static library linked into `simulate`** ([`CMakeLists.txt:25`](../src/g1_perception/dpcbf_ros_adapter/CMakeLists.txt#L25)) — *not a node*. Also builds four tools: `t1_replay`, `ab_eval`, `boundary_check`, `fsm_button_probe` |
| `safety_obstacle_filter` | §9.6 gating + inflation | **composable component** |
| `g1_perception_utils` | `base_footprint_publisher`, `obstacles_marker_relay` (×4), `dpcbf_overlay` | standalone nodes |
| `sim_mjlidar_bridge` | the simulated Mid-360 | **Python sidecar** (`ament_python`, system python 3.10) |
| `sim_msgs` | `MjState{sim_time, qpos[], mocap_pos[], mocap_quat[]}` | message package |
| `g1_description` | `g1_mid360.xacro` (the only ROS-side home of the H-1 extrinsic) + T7 guards | xacro → `robot_state_publisher` |
| `g1_perception_bringup` | all launch files, all YAML, the RViz layout, the whole test/harness fleet | launch + data |
| `livox_sdk2_vendor` | drives Livox-SDK2's plain CMake into the merged prefix (upstream installs to `/usr/local`, which needs sudo) | build-only |
| `unitree_dds_wrapper_vendor` | installs the header-only C++ wrapper | build-only |
| `t10_dds_coexistence` | one process linking SDK2 **and** rclcpp; asserts exactly one `libddsc.so.0` | test executable |

### 1.4 Custom simulator integration

`simulate/` is the upstream `unitree_mujoco` with a compile-time-optional ROS 2
module. With `-DUNITREE_MUJOCO_WITH_ROS2=OFF` it builds and behaves exactly as
at baseline `f111cfa`.

- [`simulate/src/ros2_bridge.{h,cc}`](../../simulate/src/ros2_bridge.cc) — the
  simulator's **entire** ROS 2 surface: `/clock` (250 Hz), `/sim/mj_state`
  (100 Hz), `/sim/gt_obstacles` (50 Hz), plus a one-shot dump of the *compiled*
  model XML for the sidecar's mirror.
- [`simulate/src/main.cc:952-991`](../../simulate/src/main.cc#L952-L991) — the
  1 kHz seam lambda (`axis_filter`).
- [`simulate/src/main.cc:676-793`](../../simulate/src/main.cc#L676-L793) —
  `ScriptedJoystick`, a `UnitreeJoystick` subclass that plays a sim-time profile
  through the identical `joystick->update()` path.
- `simulate/src/physics_joystick.h`, `src/joystick/` — the real-device
  `XBoxJoystick` / `SwitchJoystick`, both of which take the same
  `JoystickAxisFilter`.

### 1.5 Frozen safety-critical code

`dpcbf/` is **decision D3: byte-for-byte untouched.** It is the DPCBF QP
(`DpcbfSafetyFilter`, OSQP), the ground-truth obstacle manager
(`DynamicObstacleManager`), and the OpenCV `dpcbf_visualizer`. Nothing in this
project edits it. Every live run that needs different settings copies
`dpcbf/config/dpcbf_config.yaml` into a **shadow run-tree** — the simulator
resolves it as `<exe_dir>/../../dpcbf/config/dpcbf_config.yaml`, so it cannot be
reconfigured in place without editing a tracked file.

Gate **T1** (`t1_replay`, `ctest -R t1` in `simulate/build_ros2`) is what keeps
the freeze honest: it replays 38 402 recorded `Filter()` calls and compares I/O
byte-for-byte against the pre-refactor capture.

### 1.6 Evaluated and rejected

From §5 of the architecture doc, with the current state added:

| Package | Verdict | Why |
|---|---|---|
| `pointcloud_to_grid` | **rejected** | rasterization only — no ground handling, no detection, no tracking, no primitives. A grid intermediate is the wrong shape for a circle-detector pipeline. |
| `nav2_dynamic` | **reserved** | 3-D clustering + Hungarian KF, actively maintained; heavier to adapt (clusters → circles, different message). Kept as the fallback if the fork underperforms on real Mid-360 data; the swap point is `safety_obstacle_filter`, invisible to DPCBF. |
| `costmap_converter` | rejected | no Humble release, ROS 2 port unmaintained. |
| `nav2_costmap_2d` layers | rejected | only sensible when running Nav2; adds raster latency and still needs custom primitive extraction. |
| `grid_map` / `elevation_mapping` | deferred | 2.5-D terrain is locomotion-facing, not CBF-facing. Tap point left at `/points_no_ground`. |
| Autoware perception | rejected | vastly over-scoped for 2-D circles on an embedded PC. |
| MuJoCo built-in `rangefinder`, `mujoco_ros2_control` lidar plugin, `livox_laser_simulation_ros2` | rejected | one ray per sensor / owns the sim loop / Gazebo-classic only. |
| `FAST_LIO_LOCALIZATION_HUMANOID` | **deferred, not imported** | strongest humanoid evidence, but needs livox `CustomMsg`, i.e. driver `xfer_format 1`, which the driver publishes **instead of** (not alongside) PointCloud2. Q-4 is therefore a separate capture session, not a co-run. SHA resolved in `deps.repos:100-106` but commented out. |
| `patchwork-plusplus` | **deferred, not imported** | see §4. |
| `rosbag2_storage_mcap` | now an apt package | `ros-humble-rosbag2-storage-mcap` installed; the vcs entry stays commented out. |

---

## 2. End-to-end data flow

### 2.1 The active chain

```
                    ┌──────────────── SIM ─────────────────┐
 simulate (1 kHz SDK2 thread, 500 Hz physics)
   ├─ /clock            250 Hz   rosgraph_msgs/Clock
   ├─ /sim/mj_state     100 Hz   sim_msgs/MjState        BestEffort d1
   └─ /sim/gt_obstacles  50 Hz   obstacle_detector/Obstacles (odom) Reliable d1
                              │
        sim_mjlidar_bridge (Python sidecar, separate process)
          ├─ /odom           100 Hz  nav_msgs/Odometry      Reliable d10
          ├─ TF odom→base_link 100 Hz
          └─ /livox/lidar     10 Hz  PointCloud2 (mid360_link) SensorData
                    └──────────────────────────────────────┘
                    ┌──────────────── HW ──────────────────┐
 livox_ros_driver2_node ──► /livox/lidar 10 Hz  PointCloud2 (mid360_link) RELIABLE d256
                       └──► /livox/imu  ~200 Hz sensor_msgs/Imu           RELIABLE d256
 dlio_odom_node (subscribes both) ──► /odom 100 Hz + TF odom→base_link ~10 Hz
                    └──────────────────────────────────────┘
                              │
 ══════════ perception_container (ONE process, intra-process comms) ══════════
   pcl_ros::CropBox            /livox/lidar        → /points_self_filtered
   [pcl_ros::VoxelGrid  OFF]   /points_self_filtered → /points_voxel
   pointcloud_to_laserscan     /points_self_filtered → /scan
   obstacle_extractor          /scan               → /raw_obstacles
   obstacle_tracker            /raw_obstacles      → /tracked_obstacles
   safety_obstacle_filter      /tracked_obstacles  → /obstacles_safe
 ══════════════════════════════════════════════════════════════════════════════
                              │
             /obstacles_safe  │  Reliable depth 1 (latest wins)
                              ▼
 dpcbf_ros_adapter::ObstacleSource  ── in-process inside `simulate` ──
   executor thread (spin_some 50 ms) writes ObstacleBuffer
   1 kHz seam thread reads it, extrapolates, calls DpcbfSafetyFilter::Filter()
   ── publishes /dpcbf/status @10 Hz (wall-clock timer)
```

**Simulation and hardware differ in exactly two launch files.**
`perception.launch.py` has **zero** source conditionals (decision D4); the only
sim/hw branch outside `source_sim.launch.py` / `source_hw.launch.py` is the
`use_sim_time` default in
[`bringup.launch.py:51-54`](../src/g1_perception/g1_perception_bringup/launch/bringup.launch.py#L51-L54).

### 2.2 Stage-by-stage

**Sensor source → self-filter.**
`/livox/lidar` → `pcl_ros::CropBox` (`crop_box_self_filter`), remapped at
[`perception.launch.py:36-37`](../src/g1_perception/g1_perception_bringup/launch/perception.launch.py#L36-L37).
Subscriber QoS `SensorDataQoS().keep_last(max_queue_size_)`
([`filter.cpp:152-153`](../src/external/perception_pcl/pcl_ros/src/pcl_ros/filters/filter.cpp#L152-L153)).
`input_frame: ''` and `output_frame: ''` in
[`cropbox_self_filter.yaml`](../src/g1_perception/g1_perception_bringup/config/cropbox_self_filter.yaml)
mean **no TF is used at this stage** — the box is applied in the cloud's own
frame, `mid360_link`, and the output keeps that frame
([`filter.cpp:322-346`](../src/external/perception_pcl/pcl_ros/src/pcl_ros/filters/filter.cpp#L322-L346)).
Timestamp: passed through from the input header. Not lazy. Config loaded from
the **installed** `share/g1_perception_bringup/config/`.

**Optional downsample.** `pcl_ros::VoxelGrid`, `leaf_size: 0.05`, **off by
default**. `voxel:=on` swaps in a second container variant in which projection
reads `/points_voxel`
([`perception.launch.py:105-131`](../src/g1_perception/g1_perception_bringup/launch/perception.launch.py#L105-L131)).
It is a leaf-size-only wiring; `filter_field_name` / `filter_limit_*` are left at
`pcl_ros` defaults.

**Optional ground removal.** **Not present.** See §4.

**2-D projection.** `pointcloud_to_laserscan::PointCloudToLaserScanNode`,
`cloud_in` ← `/points_self_filtered`, `scan` → `/scan`. Publisher
`rclcpp::SensorDataQoS()`
([`pointcloud_to_laserscan_node.cpp:78`](../src/external/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp#L78)).
Output frame is `target_frame` = `base_footprint`
([`:143-145`](../src/external/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp#L143-L145));
stamp is the input cloud's. **It transforms first, then height-filters** — the
ordering that makes the height band ground-relative (§3).
**This stage is lazy**: a background thread subscribes to `cloud_in` only while
`/scan` has ≥1 subscriber, and unsubscribes when the last one leaves
([`:108-135`](../src/external/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp#L108-L135)).
With no probe and no RViz attached the whole downstream chain sits idle and
looks dead.

**Circle extraction.** `obstacle_extractor`, `/scan` → `/raw_obstacles`
(Reliable depth 5). It records `base_frame_id_` and `stamp_` from the scan
header ([`:153-154`](../src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L153-L154))
and, with `transform_coordinates: true` + `frame_id: odom`, looks up
`base_footprint → odom` **at the scan stamp**, so `/raw_obstacles` is odom-frame
and replay-deterministic. Not lazy.

**Tracking.** `obstacle_tracker`, `/raw_obstacles` → `/tracked_obstacles`
(Reliable depth 5). No timer since P-2: every message triggers predict-all →
associate → correct → prune → publish, with `dt` from consecutive header stamps
and a fallback to `1/sensor_rate` on a clock jump. Output header carries the
**measurement** stamp ([`:772`](../src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L772)).
`loop_rate: 20.0` in the YAML is inert and kept only for upstream compatibility.

**Safety inflation/filtering.** `safety_obstacle_filter`, `/tracked_obstacles`
(Reliable depth 5) → `/obstacles_safe` (**Reliable depth 1, latest wins**).
Rules in [`gating.h`](../src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h),
applied in order: message-level age gate (`now() − header.stamp > max_age` drops
**all** circles, stamp still passes through); per-circle
`true_radius > max_circle_radius` drop; `r ← max(true_radius, min_radius)`;
speed clamp to `v_max_obstacle`; then

```
radius = r + fixed_inflation + k_sigma·σ + |v_clamped|·latency_horizon
```

with the σ term **off** (`use_covariance: false`). `now()` is sim time when
`use_sim_time:=true`. **The output header keeps the input stamp**, so downstream
staleness is measured from the measurement time, not from this node's
processing time. Segments are not forwarded (§7.3 — DPCBF eats circles only).

**DPCBF adapter → 1 kHz `Filter()`.** See §6.

### 2.3 Active topic table

| Topic | Type | Rate | Frame | Publisher QoS | Producer | Timestamp source |
|---|---|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz | `mid360_link` | sim: SensorData / **hw: Reliable d256** | sidecar / driver | sim: `MjState.sim_time`; hw: LiDAR (PTP/GPS) or host clock at packet reception |
| `/points_self_filtered` | `PointCloud2` | 10 Hz | `mid360_link` | SensorData (patch 0002) | `pcl_ros::CropBox` | input header |
| `/points_voxel` | `PointCloud2` | 10 Hz | `mid360_link` | SensorData | `pcl_ros::VoxelGrid` (**only with `voxel:=on`**) | input header |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `base_footprint` | SensorData | `pointcloud_to_laserscan` | input cloud header |
| `/raw_obstacles` | `obstacle_detector/Obstacles` | 10 Hz | `odom` | Reliable d5 | `obstacle_extractor` | scan header |
| `/tracked_obstacles` | `obstacle_detector/Obstacles` | 10 Hz | `odom` | Reliable d5 | `obstacle_tracker` | measurement stamp |
| `/obstacles_safe` | `obstacle_detector/Obstacles` | 10 Hz | `odom` | **Reliable d1** | `safety_obstacle_filter` | input stamp (unchanged) |
| `/odom` | `nav_msgs/Odometry` | 100 Hz | `odom`→`base_link` | Reliable d10 | sidecar (GT pose, **twist not populated**) / DLIO | sim: `sim_time`; hw: wall |
| `/livox/imu` | `sensor_msgs/Imu` | ~200 Hz | `mid360_link` | Reliable d256 | driver only — **no sim counterpart** | hw wall/LiDAR clock |
| `/sim/gt_obstacles` | `obstacle_detector/Obstacles` | 50 Hz | `odom` | Reliable d1 | `simulate` | `d->time` |
| `/sim/mj_state` | `sim_msgs/MjState` | 100 Hz | — | BestEffort d1 | `simulate` | `d->time` |
| `/dpcbf/status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz (**wall-clock timer**) | — | Reliable d10 | `ObstacleSource` inside `simulate` | `node->get_clock()->now()` with `use_sim_time=false` ⇒ **wall** |
| `/clock` | `rosgraph_msgs/Clock` | 250 Hz + idle keep-alive | — | ClockQoS | `simulate` | `d->time` |
| `/dpcbf_overlay/markers` | `visualization_msgs/MarkerArray` | 10 Hz | `odom` + `dpcbf_velocity_plane` | default | `dpcbf_overlay` (only with `viz.launch.py overlay:=on`) | sim time |

**QoS compatibility note.** The hardware driver publishes Reliable while the sim
sidecar publishes SensorData. That asymmetry survives because **every subscriber
in the stack is best-effort**, and Reliable-pub/BestEffort-sub is a compatible
match. It is asserted on the wire by `test_hw_source_contract.launch_test.py`.

**Reconciliation:** the architecture doc's §7.1 lists `/livox/lidar` as
"SensorData (best-effort, depth 5)" in both worlds. That is the *subscriber*
contract; the hardware *publisher* is Reliable depth 256 and is not
parameterisable — recorded as a deviation in
[`livox_driver.yaml:27-30`](../src/g1_perception/g1_perception_bringup/config/livox_driver.yaml#L27-L30)
rather than patched.

---

## 3. Height cropping before 2-D projection

This is the section with the most surprising answer, so it is spelled out
mechanically.

### 3.1 The two vertical cuts, and what each is for

There are **two** vertical filters, in **two different frames**, doing **two
different jobs**.

| | CropBox `min_z`/`max_z` | `pointcloud_to_laserscan` `min_height`/`max_height` |
|---|---|---|
| Node | `crop_box_self_filter` (`pcl_ros::CropBox`) | `pointcloud_to_laserscan` |
| Config | [`config/cropbox_self_filter.yaml`](../src/g1_perception/g1_perception_bringup/config/cropbox_self_filter.yaml) | [`config/pointcloud_to_laserscan.yaml`](../src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml) |
| Param names | `min_z`, `max_z` (and `min_x/max_x/min_y/max_y`, `negative`) | `min_height`, `max_height` |
| Frame | **`mid360_link`** — the sensor frame, because `input_frame: ''` | **`base_footprint`** — the value of `target_frame` |
| TF used? | **No.** `input_frame`/`output_frame` empty ⇒ `pcl_ros::Filter` never calls `transformPointCloud` | **Yes.** Transform happens *first* |
| Polarity | `negative: true` ⇒ **removes** everything inside the box | **keeps** points whose `z` is inside the band |
| Job | self-filter (remove the robot's own returns) | ground/overhead rejection before the 2-D collapse |

### 3.2 Answers, in order

**1. Is there a min/max height filter?** Yes — `min_height: 0.15`,
`max_height: 1.60` in
[`pointcloud_to_laserscan.yaml:5-6`](../src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml#L5-L6).

**2. Which node applies each bound?** The ground/overhead band is
`pointcloud_to_laserscan`. The robot-body cut is `pcl_ros::CropBox`. They are
independent nodes with independent parameters and independent frames.

**3. What frame are the bounds in?**
- CropBox: `mid360_link`. Because `input_frame: ''`, `pcl_ros::Filter::computePublish`
  skips the transform entirely; the box coordinates are the raw sensor
  coordinates the driver/sidecar produced.
- `pointcloud_to_laserscan`: `base_footprint`, because that is `target_frame`.

**4. Is the cloud transformed before the height limits?**
- CropBox: **no**.
- `pointcloud_to_laserscan`: **yes** —
  [`:167-176`](../src/external/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp#L167-L176)
  does `tf2_->transform(*cloud_msg, *cloud, target_frame_, tolerance)`, and only
  then does [`:191`](../src/external/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp#L191)
  test `*iter_z > max_height_ || *iter_z < min_height_`. This ordering is the
  whole reason the band is usable.

**5. Shipped numerical values.**

```yaml
# cropbox_self_filter.yaml  (frame: mid360_link, negative=true → REMOVE inside)
min_x: -0.40   max_x: 0.40
min_y: -0.40   max_y: 0.40
min_z: -0.55   max_z: 0.45

# pointcloud_to_laserscan.yaml  (frame: base_footprint, KEEP inside)
target_frame: base_footprint
min_height: 0.15
max_height: 1.60
range_min: 0.3     range_max: 5.0
angle_min: -3.14159265   angle_max: 3.14159265   angle_increment: 0.0058
use_inf: true    inf_epsilon: 1.0    scan_time: 0.1   transform_tolerance: 0.05
```

**6. What those values physically retain or reject.**

The Mid-360 sits at `base_link + (−0.00368, 0.00003, 0.472434)` (torso offset
`0.044` + `mid360_z 0.428434`) and is **mounted upside-down**, `roll = π`. With
`Rx(π)` the sensor `+z` axis points **downward** in the world, so a sensor-frame
`z = +0.45` is 0.45 m *below* the sensor and `z = −0.55` is 0.55 m *above* it.
At the 0.793 m standing pelvis height the sensor is ≈ **1.265 m** above ground,
so the CropBox removes a 0.80 × 0.80 m column spanning roughly **world z ≈ 0.82
… 1.82 m** — head shell, torso, shoulders and the wrist links in the standing
pose. It does **not** touch the floor and does not bound obstacle height.

The `base_footprint` band `[0.15, 1.60]` is the one that means "above the
ground": it rejects floor returns below 0.15 m and anything above 1.60 m
(ceilings, overhangs, door lintels). `range_min: 0.3` — a *horizontal* distance
in `base_footprint` — is what removes the robot's own legs and feet, which live
below the CropBox column and inside 0.3 m.

**7. Can you configure "min/max obstacle height above the ground"?**
**Yes, and `min_height`/`max_height` are exactly that** — because
`base_footprint` has its origin on the ground plane directly under `base_link`
(`out.z = 0.0` in
[`footprint.hpp:19`](../src/g1_perception/g1_perception_utils/include/g1_perception_utils/footprint.hpp#L19)).
The band is a true ground-relative height band. Do not use CropBox for this: its
z bounds are sensor-relative, flip sign under the `roll = π` mount, and it is a
*removal* box, not a keep band.

**8. Under pitch / roll / body-height change.**

| Filter | Behaviour |
|---|---|
| CropBox `min_z/max_z` | **body-relative** — the box rides the sensor, so it rotates with torso pitch and roll and translates with body height. That is correct for a self-filter (the robot's own geometry is fixed in that frame) and wrong for anything else. |
| `pointcloud_to_laserscan` `min_height/max_height` | **ground-relative** — `base_footprint` is z = 0 with roll = pitch = 0, so the band stays horizontal while the torso pitches and rolls. |

The remaining caveat is that `base_footprint.z ≡ 0` is *asserted*, not measured:
it is `base_link` projected to zero. In sim, `/odom` is MuJoCo ground truth, so
z = 0 really is the floor. On hardware, `base_footprint.z` is 0 by construction
while DLIO's `base_link` z can drift, so a drifting z **does not** move the band —
the band is anchored to the assumed ground plane, and any DLIO z error appears
instead as points sliding *within* the band. On a slope, "0.15 m above
base_footprint" is 0.15 m above the plane through the robot's own feet, not
above the terrain ahead.

**9. Is `target_frame=base_footprint` sufficient to make the band
ground-aligned?** Yes, given the two things that make it true:
`ProjectToFootprint` zeroes roll and pitch and sets z = 0
([`footprint.hpp:14-30`](../src/g1_perception/g1_perception_utils/include/g1_perception_utils/footprint.hpp#L14-L30)),
and `pointcloud_to_laserscan` transforms before thresholding. Both hold. The
yaw is extracted ZYX-style so it remains the heading even when the torso
pitches — that matters for the *angular* bins, not the height band.

**10. Which stage performs the effective vertical cut?**
`pointcloud_to_laserscan`, after the transform, is the **only** ground-relative
vertical cut. CropBox is a body-frame self-filter and is not a substitute for
it.

**11. Wrist / arm / torso / floor / overhead — same mechanism or different?**
**Four different mechanisms:**

| Returns | Removed by |
|---|---|
| head shell (sim only) | geom-group-2 mask in the raycast — `masked_geom_group: 2` in [`sim_mjlidar_bridge.yaml`](../src/g1_perception/g1_perception_bringup/config/sim_mjlidar_bridge.yaml), applied at [`bridge_node.py:62-68`](../src/g1_perception/sim_mjlidar_bridge/sim_mjlidar_bridge/bridge_node.py#L62-L68) |
| torso, shoulders, wrists | CropBox body-frame box (`±0.40` xy, `−0.55…+0.45` z, sensor frame) |
| legs and feet | `range_min: 0.3` in `pointcloud_to_laserscan` (horizontal distance in `base_footprint`) |
| floor | `min_height: 0.15` |
| overhead | `max_height: 1.60` |

The CropBox xy/z deviate from Appendix A (`±0.35`, `max_z 0.25`) — recorded in
the YAML header: without the widening, two wrist clusters merged into a
persistent phantom circle of true_radius ≈ 0.41 m at 0.29 m, i.e. *inside*
`p_max`.

**12. Failure modes of a mis-sized band.**

*Too narrow.* Short obstacles fall out of the band and vanish — and since the
extractor needs `min_group_points: 5` in one angular group, a partially-clipped
obstacle degrades to a *smaller* fitted circle rather than disappearing cleanly,
which is worse: a confidently-tracked under-sized obstacle. On a pitching robot a
narrow band also flickers, and the tracker's `tracking_duration: 1.0` /
association gate will churn uids.

*Too wide.* Downward: floor returns enter, and the extractor happily fits
circles to the near-field ground arc — a ring of phantom obstacles at
`range_min` that the safety filter inflates and DPCBF then avoids, i.e. the
robot freezes for no reason. Upward: ceilings, overhangs and door frames project
onto the same 2-D bins as real ground obstacles. Both directions terminate in
the same place: `/obstacles_safe` grows objects that are not in the robot's way,
and because the QP takes only `default_num_constraints: 10` obstacles ordered by
closing alignment, phantoms can **displace real obstacles out of the QP**.

### 3.3 Minimal config edits for the three example policies

All three are edits to
`ros2/src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml`,
under `pointcloud_to_laserscan: → ros__parameters:`. **They require a colcon
rebuild of `g1_perception_bringup`** — configs are installed, not read from the
source tree (§10).

**A. Keep 0.10 – 1.20 m above `base_footprint`.**

```yaml
    min_height: 0.10
    max_height: 1.20
```
Frame-safe as written: `base_footprint` is ground-aligned and the transform
precedes the threshold. Caveat, not a blocker: 0.10 m is 5 cm of margin against
floor returns instead of 15 cm. In sim the ground plane is exact and this is
fine; on hardware, DLIO z error and a pitching gait can push floor points above
0.10 m, so verify with `phase2_probe.py --duration N` and watch
`scan_occupied_bin_fraction` for a jump.

**B. Keep only leg-height obstacles, 0.05 – 0.65 m.**

```yaml
    min_height: 0.05
    max_height: 0.65
```
Frame semantics are still fine, but **this one is genuinely risky and the risk
is not the frame — it is what else lives in that band.** Two things:

- 0.05 m is below any reasonable floor-rejection margin. Anything that makes
  `base_footprint` disagree with the true ground by 5 cm — DLIO z drift, a
  1.5 cm floor slope over the sensor's 5 m range, gait bounce — puts the floor
  into `/scan`. Use 0.05 only in sim, or with Patchwork++ actually enabled
  upstream (§4), which is the case §9.7 of the architecture doc was written for.
- The 0.65 m ceiling puts the band squarely on the robot's own **thighs and
  knees**, which the CropBox column (world z ≳ 0.82 m) does **not** cover. Today
  the only thing protecting that band is `range_min: 0.3`. Before shipping B,
  confirm with `selfhit_analysis.py` on a capture that no self-return survives
  inside 0.3 m horizontally, and be prepared to raise `range_min`.

**C. Reject the floor but retain table-height objects.**

```yaml
    min_height: 0.15     # unchanged — this is already the floor rejector
    max_height: 1.20     # from 1.60; excludes ceilings/lintels, keeps tables
```
Safe as written. But understand what "retain table-height objects" buys you:
after the collapse, a table's legs and its top edge occupy the **same** 2-D bins,
and the extractor fits **one** circle to whichever arc is closest and
best-grouped. The 2-D model has no notion of "under the table". If the intent is
that the robot must not walk under an overhang, that is not a height-band
question — it needs a 2.5-D representation the pipeline does not have.

**One thing none of A/B/C can express.** The band is a filter on *points*, not
on *obstacles*. There is no "minimum obstacle height" anywhere: a 0.2 m bollard
and a 1.5 m pillar produce identical `CircleObstacle`s once they are in the
band. If you need to reject low clutter you must do it by *height extent*, which
requires a segmentation stage before the collapse — that is what Patchwork++ plus
a clustering stage would give you, and it is not built.

---

## 4. Ground removal and rough terrain

**Is ground removal done only through the projection height band?** **Yes.**
`min_height: 0.15` in `base_footprint` is the entire ground-rejection mechanism.

**Is Patchwork++ compiled but disabled?** **No — it is not present at all.**
This is the sharpest documentation/implementation divergence in the tree:

- `deps.repos:107-109` has the `url-kaist/patchwork-plusplus` entry with a
  resolved SHA (`3e6903a1`) **commented out**. `setup_external.sh` never imports
  it. `ls ros2/src/external/` does not contain it.
- [`bringup.launch.py:41-42`](../src/g1_perception/g1_perception_bringup/launch/bringup.launch.py#L41-L42)
  declares `ground_seg` with `choices=['off','patchwork']` — and **never
  references it again**. There is no `IncludeLaunchDescription`, no node, no
  remap keyed on it. `ground_seg:=patchwork` is accepted by the argument
  validator and then silently does nothing.
- `/points_no_ground` exists nowhere outside documentation.

So: **the launch argument that "enables it" is `ground_seg:=patchwork`, and it
is a no-op.** Treat §9.7 / §5.8 of the architecture doc as a Phase-7 design
sketch, not as shipped behaviour.

Consequently the remaining questions are about intent, not implementation:

- **Topics in/out:** designed as `/points_self_filtered` → `/points_no_ground`,
  with `pointcloud_to_laserscan`'s `cloud_in` remapped. Not wired.
- **Frame and gravity assumptions:** Patchwork++ assumes a sensor-frame cloud
  with a known sensor height and a gravity-aligned z. The `roll = π` Mid-360
  mount means the cloud arriving on `/points_self_filtered` has z pointing
  *down*; any real integration must either pre-rotate or configure for it. Not
  resolved anywhere in this repo.
- **Does it consume IMU orientation?** Upstream Patchwork++ does not subscribe
  to IMU; it estimates ground planes per concentric-zone bin. Nothing in this
  repo would feed it one.
- **Before or after self-filtering and voxelization?** Designed to go between
  CropBox and projection — so after self-filtering, and (with `voxel:=on`)
  the ordering with VoxelGrid is undefined because neither is wired.
- **Latency / QoS / frame impact:** unmeasured. §5.8 quotes 10–30 ms/scan on
  Orin-class CPUs from upstream's paper, which is a citation, not a measurement
  on this stack.
- **Which tests prove the ground path?** **None.** No test in the tree
  references ground segmentation.

**Is flat-ground mode valid on slopes or uneven terrain?** No, and the failure is
predictable from §3.8: `base_footprint` is the plane through the robot's own
feet. Pitch the robot up a 10° ramp and the band tilts with it — points 5 m
ahead on the ramp surface sit ~0.87 m *above* the band's origin plane and enter
the band as obstacles, while a real obstacle on a downslope drops below
`min_height` and disappears. The band is correct on a flat floor and degrades
smoothly-but-wrongly off it. That is exactly the condition Patchwork++ was
deferred to handle.

**Four operations that are not the same thing:**

| Operation | Where | Frame | What it decides |
|---|---|---|---|
| self-filtering | `pcl_ros::CropBox` | `mid360_link` (body) | "is this point part of the robot?" |
| height-band filtering | `pointcloud_to_laserscan` | `base_footprint` (ground) | "is this point in the horizontal slab I care about?" |
| ground segmentation | *not implemented* | would be sensor frame | "is this point part of the terrain surface, whatever shape it is?" |
| 2-D projection | `pointcloud_to_laserscan` | `base_footprint` | "what is the nearest surviving return per angular bin?" — the irreversible step |

---

## 5. IMU handling

### 5.1 Simulation

**Is simulated IMU published?** **No ROS topic carries it.** The sidecar
publishes `/livox/lidar`, `/odom` and TF, and nothing else
([`bridge_node.py:1-18`](../src/g1_perception/sim_mjlidar_bridge/sim_mjlidar_bridge/bridge_node.py#L1-L18)).
The architecture doc's §7.1 marks `/livox/imu`'s sim producer as
"**never implemented — this topic has no sim counterpart**", and the audit
confirms it: the only references to `/livox/imu` in the whole `g1_perception`
tree are `source_hw.launch.py`, `livox_driver.yaml`, `record.launch.py`,
`hw_source_stub.py` and `test_dlio_wiring.launch_test.py`.

There *is* IMU data in simulation, but it never enters ROS. The MuJoCo IMU
sensors are read by the SDK2 bridge at 1 kHz and packed into
`unitree_hg::LowState.imu_state` (quaternion, rpy derived from it, gyroscope,
accelerometer) at
[`unitree_sdk2_bridge.h:197-220`](../../simulate/src/unitree_sdk2_bridge.h#L197-L220),
published on the DDS topic `rt/lowstate`. Its consumer is `g1_ctrl`'s policy, not
perception.

- Which process publishes it: `simulate`, over SDK2 DDS, not ROS.
- Topic / type: `rt/lowstate`, `unitree_hg::msg::dds_::LowState_`.
- Frame: the MJCF IMU site on the pelvis/torso — not a ROS TF frame.
- Rate: 1 kHz (`RecurrentThread(..., 1000 /* µs */, run)` at
  [`unitree_sdk2_bridge.h:172-173`](../../simulate/src/unitree_sdk2_bridge.h#L172-L173)).
- Generated from: MuJoCo `d->sensordata`.
- Subscribers: `g1_ctrl` (`LowState_t`), and `fsm_button_probe`.
- **Does the perception-only sim path use IMU?** **No — not at all.** Attitude
  enters perception exclusively through TF `odom→base_link`, which the sidecar
  publishes from MuJoCo ground-truth `qpos`
  ([`bridge_node.py:125-133`](../src/g1_perception/sim_mjlidar_bridge/sim_mjlidar_bridge/bridge_node.py#L125-L133)).

### 5.2 Hardware

**Does `livox_ros_driver2` publish `/livox/imu`?** Yes — the Mid-360's built-in
6-axis IMU, at ~200 Hz, on a relative topic name (hence the node is left
unnamespaced in [`source_hw.launch.py:38`](../src/g1_perception/g1_perception_bringup/launch/source_hw.launch.py#L38)).

**Expected record format / rate / QoS / frame** (verified against the pinned
checkout, and encoded literally in `hw_source_stub.py:14-19,55-57`):
`sensor_msgs/Imu` at 200 Hz, **RELIABLE depth 256** (= `kMinEthPacketQueueSize`
32 × 8), `frame_id: mid360_link` — the last only because **patch 0005** fixes
`InitImuMsg`, which upstream hardcoded to `livox_frame` regardless of the
parameter. The companion cloud is `xfer_format 0`: 7 fields, `point_step` **26**
under `#pragma pack(1)` — `x,y,z,intensity` FLOAT32 at 0/4/8/12, `tag,line` UINT8
at 16/17, `timestamp` FLOAT64 at the unaligned offset 18.

**Does DLIO subscribe to the LiDAR IMU or the body IMU?** The **LiDAR** IMU.
[`source_hw.launch.py:65`](../src/g1_perception/g1_perception_bringup/launch/source_hw.launch.py#L65)
remaps DLIO's `imu` → `/livox/imu`. The G1 body IMU is never published to ROS
and is never seen by DLIO.

**LiDAR-IMU vs body-IMU extrinsics.** DLIO **has no TF listener** — its entire
notion of where `base_link` is comes from two parameters:

```yaml
extrinsics/baselink2lidar/t: [-0.0036800000000, 0.0000300000000, 0.4724340000000]
extrinsics/baselink2imu/t:   [ 0.0073593506587, -0.0232599998416, 0.5165441705330]
extrinsics/baselink2{lidar,imu}/R: [ 0.999999602168, 0.0, -0.000891999882,
                                     0.0,           -1.0, -0.000000003590,
                                    -0.000891999882, 0.0, -0.999999602168 ]
```
([`dlio.yaml:52-61`](../src/g1_perception/g1_perception_bringup/config/dlio.yaml#L52-L61)).
These are **derived**, not authored: `baselink2lidar = (base_link→torso_link) ∘
H-1(torso_link→mid360_link)` from the xacro, and `baselink2imu = baselink2lidar ∘
(lidar→imu)` with `lidar→imu = (0.011, 0.02329, −0.04412) m`, identity rotation —
the Mid-360 user-manual constant. `t7_hw_extrinsic_guard.py` recomputes both from
the xacro and fails if this file drifts. **Left at upstream's identity default,
`odom→base_link` would really be `odom→mid360_link`: 0.47 m off in z and rotated
by roll = π.** The 5 cm lever arm is **not verified against this robot's unit** —
a 5B checklist item.

**Time offsets, gravity, bias, covariance.**

| Quantity | Where |
|---|---|
| time offset | `odom/computeTimeOffset: true` — DLIO estimates it |
| gravity | `odom/gravity: 9.80665`; `odom/imu/approximateGravity: false` |
| bias | `imu/calibration: true`, `odom/imu/calibration/{gyro,accel}: true`, `odom/imu/calibration/time: 3.0` — **the robot must be still for 3 s at boot**; intrinsics seeded at zero (`imu/intrinsics/{accel,gyro}/bias`) |
| covariance | not configured anywhere; DLIO's geometric observer gains `odom/geo/K{p,v,q,ab,gb}` play that role |

**What is the IMU used for?**

| Use | Answer |
|---|---|
| point-cloud deskewing | **yes** — `pointcloud/deskew: true`, per-point, inside DLIO only |
| odometry | **yes** — DLIO is LiDAR-*inertial*; it will not initialise without IMU |
| ground segmentation | **no** (there is none) |
| TF stabilization | **indirectly** — DLIO's attitude flows into `odom→base_link`, which is what `base_footprint_publisher` projects. There is no separate IMU-based stabilizer. |
| DPCBF directly | **no.** The QP's `RobotState` in sim comes from `mj_objectVelocity` ground truth ([`main.cc:164-182`](../../simulate/src/main.cc#L164-L182)); on hardware that path is Phase 6 and unbuilt. |

**Is the IMU path bench-tested?** Partly. `hw_source_stub.py` synthesises a
stationary specific-force vector — gravity through the `roll = π` mount,
`GRAVITY_SENSOR = [−g·sin(0.000892), 0, −g·cos(0.000892)]` — at 200 Hz on the
driver's exact QoS. Two launch tests drive it: `test_hw_source_contract` (wire
format, QoS, TF tree shape) and `test_dlio_wiring` (subscriptions, QoS, `/odom`
rate, frame parentage). The stub's own docstring is explicit that
**"odometry computed against it is meaningless — these tests assert wiring, QoS
and TF tree shape, never odometry quality."**

Unverified without hardware: real IMU noise, bias stability, the 3 s
still-calibration actually converging on a robot that is never perfectly still,
PTP-vs-host timestamping (there is *no* timestamp-mode field in
`MID360_config.json` — sync mode is read out of each packet header, and the
host-clock path is a fallback, not a setting), the 5 cm lidar→IMU lever arm on
this unit, and DLIO's odometry accuracy and CPU cost.

### 5.3 `base_footprint`: roll/pitch-stripped, and why it matters

`base_footprint` is **constructed yaw-only**, not derived with roll and pitch.
[`ProjectToFootprint`](../src/g1_perception/g1_perception_utils/include/g1_perception_utils/footprint.hpp#L14-L30)
takes `odom→base_link`, keeps x and y, sets **z = 0**, extracts yaw ZYX-style
from the quaternion and rebuilds the rotation as pure yaw
(`qx = qy = 0`).

This is the load-bearing detail for §3 and for the whole 2-D reduction. Because
the projector transforms into this frame *before* thresholding z:

- the height band stays horizontal while the torso pitches and rolls through a
  gait cycle — the stabilization the abandoned branch implemented with custom
  gravity-alignment code is obtained for free from one frame choice;
- the `LaserScan` angular bins are heading-referenced rather than torso-referenced,
  so a pitching robot does not smear its own scan;
- and `odom→base_footprint` is what the extractor's stamped TF lookup composes
  to get odom-frame obstacles.

Two operational consequences: `base_footprint_publisher` **deduplicates by
stamp** ([`base_footprint_publisher.cpp:41-42`](../src/g1_perception/g1_perception_utils/src/base_footprint_publisher.cpp#L41-L42)),
so a `static_transform_publisher` standing in for odometry makes it emit
`base_footprint` exactly once at t = 0 and every later lookup extrapolates into
the future; and its rate is the odometry rate — 100 Hz in sim, **~10 Hz on
hardware**, because DLIO broadcasts TF from its per-scan `publishToROS()` thread,
not from the 100 Hz pose timer.

---

## 6. Rate mismatch: 10 Hz perception vs 1 kHz control

This is the core of the design and it is worth being exact about, because two
different things are going on: a *transport* solution (a wait-free double buffer)
and a *control* solution (extrapolate, then ramp the command to zero rather than
the obstacle set to empty).

### 6.1 The eight rates, from the repository

| Rate | Value | Source |
|---|---|---|
| MuJoCo physics | **500 Hz** (`timestep` 0.002 — MuJoCo default; no `<option>` element in `scene_g1.xml` or `g1.xml`) | `mj_step` in `PhysicsLoop` |
| SDK2 bridge / seam / `Filter()` | **1 kHz** | `RecurrentThread("unitree_bridge", UT_CPU_ID_NONE, 1000 /* µs */, run)` — [`unitree_sdk2_bridge.h:170-179`](../../simulate/src/unitree_sdk2_bridge.h#L170-L179); `run()` calls `joystick->update()`, which calls `axis_filter_`, which calls `Filter()` |
| RL policy (`State_RLBase`) | **50 Hz** (`step_dt: 0.02`, `deploy/robots/g1/config/policy/velocity/v1/params/deploy.yaml:2`) | its own `policy_thread` |
| DPCBF QP reference frequency | **500 Hz** (`reference_control_frequency_hz`) | `dpcbf_config.yaml` |
| Sensor / perception (every stage) | **10 Hz** | `scan_rate: 10.0` (sim) / `publish_freq: 10.0` (hw) |
| Tracker update | **10 Hz** — measurement-driven, exactly one update per `/raw_obstacles` | patch 0004 |
| Adapter subscription callback | **10 Hz** (arrival-driven, on `ObstacleSource`'s own executor thread) | `obstacle_source.cpp:298-317` |
| `/odom` + TF `odom→base_link` | **100 Hz** sim / **~10 Hz** hw | `/sim/mj_state` cadence / DLIO per-scan thread |
| `/clock` | 250 Hz | `kClockPeriod = 0.004` |
| `/dpcbf/status` | 10 Hz, **wall-clock** | `create_wall_timer(0.1)` |

Note the two easily-conflated pairs: the seam runs at **1 kHz** while physics
runs at **500 Hz**, so roughly every second `Filter()` call sees an *unchanged*
`d->time` and an unchanged robot state; and the QP's delta-v is normalised at
**500 Hz** while being applied at **1 kHz**.

### 6.2 The mechanism, question by question

**3. Does the adapter cache the latest obstacle set?** Yes — `ObstacleBuffer`, a
**double buffer with a per-slot seqlock**
([`obstacle_buffer.h:131-184`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L131-L184)).
`Publish()` writes the back slot (odd seq → writing, even → done) then flips
`front_`; `Read()` retries only if it observes a torn slot. Capacity
`kMaxObstacles = 128`; overflow is counted (`dropped_circles`), never UB.

**4/5. Extrapolation.** Yes, constant-velocity, computed **at query time, not at
arrival time**, in `Materialize()`
([`obstacle_buffer.h:89-124`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L89-L124)):

```cpp
age      = max(0, t_query_s - frame.stamp);
dt_extrap= min(age, max_age_s + fade_out_s);          // capped at 0.60 s
o.x += o.velocity_x * dt_extrap;
o.y += o.velocity_y * dt_extrap;
if (state == kStop)                                    // radius inflation
  o.radius += hypot(o.velocity_x, o.velocity_y) * min(age, hold_after_stale_s);
```

The `dt_extrap` cap is the "retain the last set" rule: past the stop boundary the
set is frozen at its 0.60 s extrapolation point rather than chasing a stale
velocity to the horizon.

**6. Which timestamp?** **Sim time on both sides, no wall clock anywhere in the
safety path.** `t_query = d->time` ([`main.cc:956`](../../simulate/src/main.cc#L956));
`frame.stamp = StampToSec(msg.header.stamp)`
([`obstacle_source.cpp:84`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L84)),
which is the `/obstacles_safe` header — which is the tracker's measurement stamp
— which is the `/scan` stamp — which is the cloud stamp. **Neither ROS reception
time nor wall time is used.** The adapter node deliberately runs
`use_sim_time = false` ([`obstacle_source.cpp:293`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L293)):
inside `simulate` it must not consume the `/clock` its own process publishes.
Wall time paces only the diagnostics timer and the query-latency histogram.

**7. Robot velocity.** MuJoCo ground truth, every tick:
`mj_objectVelocity(model, data, mjOBJ_BODY, body_id, v, /*flg_local=*/1)`, taking
`v[3]`/`v[4]` as sagittal/lateral, and `phi = atan2(xmat[3], xmat[0])`
([`main.cc:164-182`](../../simulate/src/main.cc#L164-L182)). It is **not** derived
from `/odom` and does not go through ROS. (The RViz overlay, which has no such
access, must differentiate `/odom` pose — hence the ~0.03 m/s velocity
disagreement documented in runbook §4.6. That is a *viewer* limitation, not a
control-path one.)

**8. Obstacle velocities.** From `CircleObstacle.velocity`, i.e. the tracker's
per-axis Kalman estimate, in `odom`, copied verbatim in `OnObstacles()`. In
oracle mode they are MuJoCo ground truth from `DynamicObstacleManager::Snapshot()`.

**9. Interpolation, extrapolation, or ZOH?** **Extrapolation** for position
(linear in `dt_extrap`); **zero-order hold** for radius while fresh or
degrading; **ZOH + growth** for radius in the stop regime. No interpolation
anywhere — there is nothing to interpolate *to*.

**10. Do radii change over time?** Only in `kStop`, by
`|v|·min(age, hold_after_stale_s)`, i.e. the H-8 velocity-inflation kernel with
age as the horizon and a 1.0 s cap. Note this is a *second*, adapter-side
inflation on top of `safety_obstacle_filter`'s `|v|·latency_horizon`; the two are
in different processes and both apply.

**11. An obstacle that disappears for one or more frames.** Nothing happens in
the adapter — it holds whole frames, not per-obstacle tracks. The dropout is
handled two stages earlier: `obstacle_tracker` keeps predicting a track without
correction and only prunes after `tracking_duration: 1.0` s of fading, so the
obstacle continues to appear in `/tracked_obstacles` (coasting on its KF) and
therefore in `/obstacles_safe`. Its covariance grows, which today changes nothing
because `use_covariance: false`. Only when the tracker finally drops it does the
adapter see a smaller set. This is the mechanism the S4 occlusion fixture
exercises, and the recorded containment there is the pipeline's weakest number
(13.2 % at F = 0 mm; runbook §7.1).

**12. FRESH / DEGRADE / STOP thresholds.**

```cpp
kFresh   : age <= max_age                      (0.30 s)
kDegrade : max_age < age <= max_age + fade_out (0.30 – 0.60 s)
kStop    : age > 0.60 s
kNoData  : never received a frame              (startup)
```
`CommandScale(age)` is `1.0` while fresh, a linear ramp `1 → 0` across the
fade-out window, `0` after. `kNoData` is treated as the fail-safe stop **with an
empty set** — there is no last-known set to retain, and scale 0 keeps the robot
stopped until perception is up.

**13. Inflation as data goes stale.** Position extrapolation up to 0.60 s;
radius growth only in `kStop`, capped by `hold_after_stale_s = 1.0`.

**14. Does the retained set ever become empty?** **No, by explicit design** —
"obstacles-empty-on-stale is forbidden: the set is retained indefinitely
(inflation capped at `hold_after_stale`) — the command is already 0"
([`obstacle_buffer.h:87-88`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L87-L88)).
The only empty case is `kNoData`.

**15. Where motion is stopped.** At the **call site, before `Filter()`**:

```cpp
const auto desired  = dra::AxesToDesired(lx, ly, rx, seam_limits);
const auto scaled   = dra::ScaleDesired(desired, snap.command_scale);   // <-- here
const auto filtered = safety_filter.Filter(robot, scaled, snap.obstacles);
```
([`main.cc:958-961`](../../simulate/src/main.cc#L958-L961), with `ScaleDesired` at
[`dpcbf_seam.h:46-54`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/dpcbf_seam.h#L46-L54)).
`ScaleDesired` returns the input **unmodified** when `scale >= 1.0` — a bit-exact
no-op on the fresh and oracle paths, which is what lets T1 stay byte-comparable.

**16. `/dpcbf/status`.** `diagnostic_msgs/DiagnosticArray`, 10 Hz,
`name: "dpcbf_ros_adapter"`, `hardware_id` = the mode string, `message` =
`fresh|degrade|stop|no_data`, and `level` = **OK** for oracle-or-fresh,
**WARN** for degrade, **ERROR** for stop/no_data
([`obstacle_source.cpp:104-194`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L104-L194)).
Key/value pairs: `mode`, `staleness_state`, `age_s`, `command_scale`,
`frames_received`, `dropped_circles`, `query_count`,
`query_latency_histogram` (buckets `le{1,2,5,10,20,50,100,1e9}us`),
`query_latency_p50_le_us`, `query_latency_p99_le_us`, plus the nine
`shadow_*` fields in shadow mode.

**17. Oracle vs estimated at the adapter interface.** Structurally separated, not
switched (H-9 enforced by construction): the mode is fixed at construction with
no setter; `kOracle` **never creates the subscription**; `kEstimated` never
invokes the oracle callback; `GetObstacles` returns exactly one source's set and
there is no API path that mixes them in one `Filter()` call. In `kOracle`,
`Snapshot.fresh = true`, `age_s = 0`, `command_scale = 1.0` unconditionally — no
extrapolation, no staleness.

**18. New vector every 1 ms, or reused?** A **new `std::vector` every tick.**
`GetObstacles` returns by value; in `kEstimated`, `Materialize()` fills
`scratch_snapshot.obstacles` (a member vector that keeps its capacity), then
`snap.obstacles = std::move(...)` and the scratch is re-seated with a fresh empty
vector ([`obstacle_source.cpp:357-372`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L357-L372)).
The *contents* are recomputed from the cached frame each tick — extrapolated to
that tick's `t_query` — so successive ticks between two perception frames see
*different* obstacle positions.

**19. Different threads?** Yes, three:
(a) the ROS executor thread inside `ObstacleSource`
(`SingleThreadedExecutor::spin_some(50 ms)` in its own `std::thread`) —
subscription callback + diagnostics timer;
(b) the SDK2 `RecurrentThread` at 1 kHz — the only caller of `GetObstacles` and
`Filter()`;
(c) the MuJoCo physics thread.

**20. Synchronization primitive.** For the obstacle handoff: **none in the
blocking sense** — the seqlock'd double buffer, `std::atomic` seq/front, wait-free
for the reader. `ObstacleBuffer::Read` can spin only if the writer laps the
reader's slot mid-copy, which at 10 Hz writes vs µs reads needs the reader to
stall > 100 ms inside the copy. The only `std::mutex` is `shadow_mutex`, and the
1 kHz thread **`try_lock`s** it and increments `shadow_flush_misses` on failure
rather than ever blocking ([`obstacle_source.cpp:251-257`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L251-L257)).

**21. Dynamic allocation in the 1 kHz loop?** **Yes** — and it is a deliberate,
documented trade rather than an oversight. `GetObstacles` returns a `std::vector`;
`Filter()` allocates `nearest`, `dpcbf_constraints`, `selected_obstacles`,
`qp_constraints` and Eigen vectors per call
([`dpcbf_safety_filter.cpp:502-620`](../../dpcbf/src/dpcbf_safety_filter.cpp#L502-L620)).
`dpcbf/` is frozen (D3), so this cannot be changed without breaking T1. The
measured cost is what justifies it: `query_latency_p99_le_us` puts `GetObstacles`
in the single-digit-µs buckets, and 1 ms is 1000 µs.

**22. Measured latency and CPU** (quoted from runbook §3.1/§3.2/§4.6, not
re-measured here):

| | bag replay (34 s) | live sim, 90 obstacles (45 s) |
|---|---|---|
| `cloud→scan` p50/p95 | 0.49 / 0.95 ms | 0.66 / 1.17 ms |
| `cloud→tracked` p50/p95 | 0.67 / 1.16 ms | 1.26 / 1.91 ms |
| `cloud→safe` p50/p95/max | 0.76 / 1.28 / 1.41 ms | 1.48 / 2.18 / 3.93 ms |
| `perception_container` CPU | 2.9 % of one core | 4.6 % |
| `sim_mjlidar_bridge` CPU | — | 30.3 % (24 000 rays @ 10 Hz) |

The end-to-end perception latency is ~1.5 ms against a 100 ms budget; the
dominant age term is the 100 ms sampling interval itself, which is precisely what
the extrapolation exists to cancel.

**23. Which tests verify what.**

| Property | Gate | Where |
|---|---|---|
| oracle bit-equivalence | **T1** — 38 402 recorded `Filter()` calls, byte-for-byte | `cd simulate/build_ros2 && ctest -R t1` (`t1_replay`) |
| extrapolation + staleness ladder | `test_obstacle_buffer` (gtest) | `colcon test --merge-install --packages-select dpcbf_ros_adapter` |
| mode separation, subscription behaviour, diagnostics | `test_obstacle_source` (gtest) | same |
| overlay constraint-geometry recomputation | `dpcbf_boundary_recomputation` — 38 402 ticks / 214 085 rows, deltas exactly 0 | `ctest -R dpcbf_boundary` |
| gating + inflation rules | `test_gating` (16 tests) | `--packages-select safety_obstacle_filter` |
| timestamp correctness end-to-end | **T9** — `odom→base_footprint` resolves within 50 ms of every cloud | asserted in the replay/wall tests; printed by `phase2_probe.py` |
| deterministic replay | **T8** — two replays give bit-identical `/raw_obstacles` **and** `/tracked_obstacles` | CTest `t8_replay_determinism` |
| stale-data behaviour end-to-end (**T6**) | the SIGKILL-the-container drill | `phase4_live_session.sh estimated 90 /tmp/t6 t6` — **not executed in the recorded session**; the DEGRADE-0.300 s / STOP-0.600 s figures are the Phase-4 record |

### 6.3 Timing diagram — one 100 ms perception interval

```
sim time (ms)   0        10       20  ...        90      100      110
                │                                        │
/scan, /raw,    ▼ frame k                                 ▼ frame k+1
/tracked,       stamp = t_k                               stamp = t_k + 0.100
/obstacles_safe │                                         │
                │  (~1.5 ms of pipeline latency; the      │
                │   header still says t_k, so the age     │
                │   the adapter computes already          │
                │   INCLUDES that latency)                │
                │                                         │
ObstacleBuffer  ├─ Publish(frame k) on the executor thread┤
                │                                         │
1 kHz seam      ├┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┬┤   100 ticks
ticks           ││││││││││││││││││││││││││││││││││││││││││
                ││                                     ││
 tick n (t=t_k+0.001):  age=0.001  dt_extrap=0.001  o.x += vx·0.001   scale=1.0
 tick n+50    (t_k+0.050): age=0.050 dt_extrap=0.050 o.x += vx·0.050  scale=1.0
 tick n+99    (t_k+0.099): age=0.099 dt_extrap=0.099 o.x += vx·0.099  scale=1.0
                                            ▲
              every tick recomputes from the SAME cached frame k,
              extrapolated to ITS OWN t_query — not a zero-order hold

MuJoCo physics  ├─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┼─┤   50 steps (500 Hz)
                 ▲ so ~every 2nd seam tick sees an UNCHANGED d->time and robot state

RL policy       ├───────┼───────┼───────┼───────┼─────────┤   5 steps (50 Hz)
                the policy consumes the FILTERED axes via rt/lowstate's
                wireless_remote, at its own cadence
```

**If frame k+1 never arrives:**

```
age    0 ────────── 0.30 ────────── 0.60 ─────────── 1.60 ──────►
state  │   FRESH    │    DEGRADE    │        STOP              │
scale  │    1.0     │  1.0 → 0.0    │         0.0              │
pos    │ extrapolate to age          │ frozen at dt_extrap=0.60 │
radius │            ZOH              │ +|v|·min(age,1.0), capped │
set    │                    NEVER EMPTIED                       │
status │    OK      │     WARN      │        ERROR             │
```

The key control-theoretic point: **the system degrades the command, not the
model.** Emptying the obstacle set on stale data would make the QP unconstrained
and therefore *permissive* exactly when it has least information. Instead the set
is retained and grown while the commanded velocity is independently ramped to
zero — so a perception outage converges to "stopped inside a conservative
obstacle field", which is the safe fixed point.

---

## 7. Oracle, shadow, and estimated modes

### 7.1 Semantics

| | `oracle` (default, D5) | `shadow` | `estimated` |
|---|---|---|---|
| What `Filter()` eats | `DynamicObstacleManager::Snapshot()` — MuJoCo ground truth, converted index-as-id ([`main.cc:889-901`](../../simulate/src/main.cc#L889-L901)) | **the oracle**, same as above | `/obstacles_safe`, extrapolated + staleness-ladder'd |
| Perception still runs? | yes, if you launched it — but nothing consumes `/obstacles_safe` at the seam. **No subscription is even created.** | yes, and it is consumed | yes, and it is consumed |
| Estimated obstacles logged/compared? | no | **yes** — greedy NN within 0.5 m, restricted to oracle obstacles inside `p_max`, accumulating position/velocity error, radius containment margin and frame age ([`obstacle_source.cpp:199-258`](../src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L199-L258)) | no comparison (no GT reference) |
| Filtered command actually changes? | yes — the QP runs | yes, **driven by GT** | yes, driven by perception |
| `command_scale` | 1.0 always | 1.0 always (`Snapshot` defaults; the ladder is computed for the *comparison*, not applied) | 1.0 / ramp / 0.0 |
| `/dpcbf/status` reports | `mode=oracle`, `level=OK` always, `age_s=-1` | `mode=shadow` + the nine `shadow_*` keys; `age_s` tracks the estimated stream | `mode=estimated`, full ladder |
| Selected by | `UNITREE_DPCBF_MODE=oracle\|shadow\|estimated` ([`main.cc:904-918`](../../simulate/src/main.cc#L904-L918)) | same | same |
| Changeable at runtime? | **No.** Fixed at construction, no setter — H-9 is enforced structurally. Changing mode means restarting `simulate`. | | |
| If `/obstacles_safe` is absent | irrelevant — no subscription | oracle still drives the robot; `shadow_frames` stays 0 | `kNoData` → **empty set, `command_scale = 0`, robot commanded to a stop**, `/dpcbf/status` level ERROR |
| GT topics on hardware | **no.** `/sim/gt_obstacles` is sim-only. On hardware `oracle` and `shadow` are meaningless — `ObstacleSource`'s constructor *throws* without an oracle callback, and there is no hardware oracle. | | |

Two further constraints: `shadow`/`estimated` require the ROS 2 build
(`-DUNITREE_MUJOCO_WITH_ROS2=ON`) and error out otherwise
([`main.cc:919-925`](../../simulate/src/main.cc#L919-L925)); and `p_max` for the
shadow comparison is the adapter's **own default of 3.0**
([`obstacle_source.h:66`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_source.h#L66)),
not read from `dpcbf_config.yaml` — it happens to match today's `p_max: 3.0` and
would silently diverge if that were retuned.

### 7.2 Commands

**A. Live MuJoCo + RViz on the real desktop, walking, oracle mode.**

`run_live_w4_view.sh` takes `[scenario] [mode]` and forwards `$2` verbatim to
`UNITREE_DPCBF_MODE`. Verified by reading the script: `MODE=${2:-estimated}` and
`UNITREE_DPCBF_MODE="$MODE"` in the simulator's env block.

```bash
cd ~/unitree_rl_mjlab_
ros2/src/g1_perception/g1_perception_bringup/test/run_live_w4_view.sh W4 oracle
```

That single command gives you everything asked for: a visible MuJoCo window and
a visible RViz top-down 2-D view on `DISPLAY=:1`, the Mid-360 sidecar running
(`LivoxCloud` is force-enabled in a copy of the layout), `g1_ctrl` started
**first** so it wins the `wait_for_connection()` race, the scripted walk profile,
the elastic band at `34,6`, and DPCBF consuming ground truth. It refuses to run
on `:77`/`:1001`/`:1002` and it stays up until you Ctrl+C. The OpenCV
`dpcbf_visualizer` is deliberately re-enabled (`walk_scenarios.py` turns it off
for headless batches). Logs land in `/tmp/live_w4_logs/`.

Timeline in **sim** seconds: 15.0/15.5 → FixStand, 21.0/21.5 → Velocity, 34→40
band lowers, 40.0 walking.

**B. Same, estimated mode — DPCBF consumes `/obstacles_safe`.**

```bash
cd ~/unitree_rl_mjlab_
ros2/src/g1_perception/g1_perception_bringup/test/run_live_w4_view.sh W4 estimated
```

(`estimated` is also the script's default, so a bare `run_live_w4_view.sh` does
this on W4.) Scenarios: `W1` 6 static, `W2` 6 crossing at 0.5–0.8 m/s, `W3` 20
mixed, `W4` the 90-obstacle Phase-4 field, seed 42.

**C. Shadow mode.**

```bash
cd ~/unitree_rl_mjlab_
ros2/src/g1_perception/g1_perception_bringup/test/run_live_w4_view.sh W4 shadow
```

**What is actually controlling the robot: the oracle.** `Filter()` receives
`im.config.oracle()` — MuJoCo ground truth — exactly as in oracle mode, and the
command is bit-identical to an oracle run. The perception stream is read *in
parallel*, materialised to the same `t_query`, and compared; the deltas go to
`/dpcbf/status` and nowhere else. It is the risk-free way to price the full
perception stack against truth while the robot is still driven by truth. Watch
it with:

```bash
ros2 topic echo /dpcbf/status --field status[0].values
```

**D. Perception-only bag replay.**

```bash
cd ~/unitree_rl_mjlab_/ros2
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true &
sleep 5
python3 src/g1_perception/g1_perception_bringup/test/phase4_latency_probe.py 34 /tmp/lat.txt &
sleep 2
ros2 bag play test_fixtures/s1_surveyed
```

**Why this cannot produce a closed-loop collision result or live DPCBF status.**
Three independent reasons, each sufficient:

1. **`simulate` is not running**, so `DpcbfSafetyFilter::Filter()` is never
   called, `ObstacleSource` is never constructed, and `/dpcbf/status` has no
   publisher. Any status you see is a leftover process.
2. **The loop is open.** The bag is a recording; nothing the filter decides can
   change where the robot goes or where the obstacles are. Collision rate and
   minimum clearance are properties of a trajectory the replay cannot alter.
3. Even the offline A/B harness (`phase4_ab_run.sh`), which *does* run two real
   `DpcbfSafetyFilter` arms at 1 kHz over the replayed obstacle streams, pins the
   robot at (0, 0, 0). It scores command tracking and containment, never collision
   rate — a robot that never moves has neither.

For collision rate and clearance you need the closed loop:
`walk_ab_run.sh [outdir] [scenarios...]`, which brings up `g1_ctrl` + simulator +
description + sidecar + perception once per mode in `WALK_MODES`.

Bag replay remains the right first move when something looks broken, precisely
because it removes the simulator from the question. But remember the lazy
subscription: with no probe and no RViz attached, `pointcloud_to_laserscan` never
subscribes and the whole chain sits idle.

---

## 8. Joystick and scripted control

### 8.1 Scripted mode

**Why `UNITREE_MUJOCO_SCRIPTED_COMMANDS` is mandatory here, and why the seam is
dead without a joystick object.** This is a structural fact, not a workaround.
`Filter()` is only ever reached through `JoystickAxisFilter`, and that callable
is only invoked from inside a `UnitreeJoystick::update()` implementation. The
call chain is:

```
RecurrentThread(1 kHz) → RobotBridge::run()
   → if (lowstate->joystick) lowstate->joystick->update()
        → axes = axis_filter_(lx, ly, rx)          ← the DPCBF seam
```
([`unitree_sdk2_bridge.h:177-179`](../../simulate/src/unitree_sdk2_bridge.h#L177-L179)).
`UnitreeSDK2BridgeBase`'s constructor only constructs a joystick when
`param::config.use_joystick == 1`
([`unitree_sdk2_bridge.h:30-40`](../../simulate/src/unitree_sdk2_bridge.h#L30-L40)).
This machine has no `/dev/input/js0` and every harness therefore writes
`use_joystick: 0` into its shadow config — at which point `lowstate->joystick`
is null, `update()` is never called, `axis_filter_` is never invoked, and **the
1 kHz DPCBF seam is dead code that no topic-level probe can detect**. It was dead
through Phases 0–3 for exactly this reason. `ScriptedJoystick` restores it by
being installed on the **public** `lowstate->joystick` and
`wireless_controller->joystick` members *before* `interface->start()`, so the
1 kHz thread never observes a half-wired joystick
([`main.cc:1057-1072`](../../simulate/src/main.cc#L1057-L1072)).

**Profile path and format.**
`ros2/src/g1_perception/g1_perception_bringup/config/walk_profile.txt`, passed by
absolute path in the environment variable. Format: `t lx ly rx [buttons]`,
whitespace-separated, `#` comments, **piecewise-constant hold** — the last
breakpoint with `p.t <= d->time` wins
([`main.cc:754-782`](../../simulate/src/main.cc#L754-L782)). `buttons` is a
comma-separated key list held to the next breakpoint; `-` or absent means none.
Accepted keys: `L2 R2 L1 R1 A B X Y up down left right start select back` plus
the aliases `LT→L2 RT→R2 LB→L1 RB→R1`. **An unknown key aborts at load**
(`exit(1)`) rather than silently doing nothing — a typo'd key is exactly how a
scripted FSM run becomes an unexplained failure.

**The staggered chords, and why simultaneous fails.** `g1_ctrl`'s transitions are
DSL strings in `deploy/robots/g1/config/config.yaml`:

```yaml
Passive:  transitions: { FixStand: LT + up.on_pressed }
FixStand: transitions: { Passive: LT + B.on_pressed, Velocity: RT + A.on_pressed }
```

`LT`/`RT` arrive as the **L2/R2 bits** of `wireless_remote`
([`g1_pub.h:41-46`](../src/external/unitree_dds_wrapper/cpp/include/unitree/dds_wrapper/robots/g1/g1_pub.h#L41-L46)),
and the **receiver** re-filters them through an `Axis`
([`g1_sub.h:103-104`](../src/external/unitree_dds_wrapper/cpp/include/unitree/dds_wrapper/robots/g1/g1_sub.h#L103-L104)).
`Axis::operator()` is an exponential low-pass with `smooth = 0.03` compared
against `threshold = 0.5`
([`unitree_joystick.hpp:96-113`](../src/external/unitree_dds_wrapper/cpp/include/unitree/dds_wrapper/common/unitree_joystick.hpp#L96-L113)):

```
data_ ← data_·(1 − 0.03) + input·0.03      pressed ← (data_ > 0.5)
```

so a step to 1.0 needs `1 − 0.97ⁿ > 0.5`, i.e. **n ≈ 23 ticks ≈ 23 ms**, before
`LT.pressed` goes true. Meanwhile `up.on_pressed` is true for **exactly one
tick** — the first. Pressed at the same breakpoint, the AND is never
simultaneously true and the FSM silently stays in Passive. Hold the axis half
≥ 25 ms (the profile uses 0.5 s) **before** adding the button. Setting
`smooth = 1.0` on the *sender* (which `ScriptedJoystick` does, at
[`main.cc:714-715`](../../simulate/src/main.cc#L714-L715)) does not help — it is
the receiver's filter that lags.

**Axis mapping.** At the seam
([`dpcbf_seam.h:28-42`](../src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/dpcbf_seam.h#L28-L42)):

```
desired.sagittal = AxisToCommand( ly, v_s_min, v_s_max)
desired.lateral  = AxisToCommand(-lx, v_l_min, v_l_max)
desired.yaw_rate = AxisToCommand(-rx, w_min,   w_max)
```
and back the other way through `CommandToAxes`. `AxisToCommand` maps each
**half-axis** onto the asymmetric range:
`normalized ≥ 0 ? normalized·max : −normalized·min`. The policy applies the
identical rule at
[`observations.h:131-133`](../../deploy/include/isaaclab/envs/mdp/observations/observations.h#L131-L133)
with `scale_from_joystick: true` and ranges `lin_vel_x [−1, 2.0]`,
`lin_vel_y [−1, 1]`, `ang_vel_z [−1, 1]`. So: **`ly` → forward, `−lx` → lateral,
`−rx` → yaw**; `ly = 0.20` is 0.40 m/s forward.

**Chord keys deliberately bypass the filter.** `axis_filter_` is applied to
`lx/ly/rx` only; buttons are written straight through
([`main.cc:772-781`](../../simulate/src/main.cc#L772-L781)) — DPCBF gates
locomotion commands, not mode changes.

**When walking begins.** At `t = 40.0` sim seconds in the shipped profile — the
first line with a non-zero `ly`. Everything before it is bring-up.

**Elastic band.** Enabled by `enable_elastic_band: 1` in the (shadow)
`simulate/config.yaml`. Two environment variables, both inert when unset
([`main.cc:1042-1055`](../../simulate/src/main.cc#L1042-L1055)):
`UNITREE_MUJOCO_BAND_LENGTH` (rest length; **0.572** carries the 33.34 kg robot
at its 0.793 m spawn height) and `UNITREE_MUJOCO_BAND_RELEASE=<t0>[,<ramp>]`
(sim-time lowering). Interactively you can also use keys `9` (toggle), `7`/`Up`
(shorten 0.1 m), `8`/`Down` (lengthen). **Lower it onto the running policy, never
onto FixStand's PD hold pose** — that topples the robot every time (tilt 0.02 →
0.33 rad within 3 s). `34,6` is preferred over the harness default `24,4`, which
is kept only for comparability with the recorded A/B matrix.

**Changing forward / lateral / yaw.** Edit `walk_profile.txt` columns 2–4 and
re-run. It is read at simulator startup from the path in the environment
variable, and `run_live_w4_view.sh` copies it into the shadow tree — so **no
rebuild is needed**, but you must edit the copy the run will actually read (or
edit the source and re-run the launcher, which re-copies).

### 8.2 Physical joystick mode

**Status: SOURCE-SUPPORTED, NOT MEASURED.** There is no gamepad on this machine,
no test in the tree exercises `XBoxJoystick`/`SwitchJoystick`, and the runbook
records no joystick session. Everything below is read from source; the
distinction is marked per item.

1. **Devices supported** *(source)*: two layouts —
   `joystick_type: "xbox"` → `XBoxJoystick`, `"switch"` → `SwitchJoystick`
   ([`physics_joystick.h`](../../simulate/src/physics_joystick.h)). Anything else
   is `exit(EXIT_FAILURE)`. The backend is the plain Linux `js` interface
   (`simulate/src/joystick/joystick.cc`).
2. **Device path** *(source)*: `joystick_device: "/dev/input/js0"`, and if
   `isFound()` is false the simulator prints `Error: Joystick open failed.` and
   `exit(1)`.
3. **Config that enables it** *(source)*: `use_joystick: 1` in
   `simulate/config.yaml`. **The tracked file already has `use_joystick: 1`** —
   it is every *harness* that rewrites it to 0 in its shadow tree.
4. **Can a real gamepad co-exist with the ROS 2 stack?** *(source)* Yes. They are
   orthogonal: the gamepad is read by `simulate`'s SDK2 bridge thread; the ROS 2
   module is a separate publisher set in the same process. Nothing arbitrates
   between them.
5. **Does joystick mode still run DPCBF at 1 kHz?** *(source)* **Yes, and this is
   the point** — `XBoxJoystick::update()` calls
   `axes = axis_filter_(axes[0], axes[1], axes[2])` at
   [`physics_joystick.h:56`](../../simulate/src/physics_joystick.h#L56), exactly
   where `ScriptedJoystick` does. The device path is the *original* seam; the
   scripted path was added to emulate it.
6. **Startup without `UNITREE_MUJOCO_SCRIPTED_COMMANDS`** *(source-derived
   procedure, unverified end-to-end)*:

```bash
# --- environment (runbook §2.1) --------------------------------------------
export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa

# --- shadow run-tree with the joystick ON ----------------------------------
REPO=~/unitree_rl_mjlab_; TREE=/tmp/sim_joy
rm -rf $TREE && mkdir -p $TREE/simulate/build $TREE/dpcbf/config
ln -sfn $REPO/src $TREE/src
cp $REPO/simulate/build_ros2/unitree_mujoco $TREE/simulate/build/
cp $REPO/dpcbf/config/dpcbf_config.yaml     $TREE/dpcbf/config/
sed -e 's/^use_joystick: .*/use_joystick: 1/' \
    -e 's/^joystick_device: .*/joystick_device: "\/dev\/input\/js0"/' \
    -e 's/^joystick_type: .*/joystick_type: "xbox"/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    $REPO/simulate/config.yaml > $TREE/simulate/config.yaml

# --- 1. g1_ctrl FIRST (blocks until rt/lowstate exists) --------------------
( cd $REPO/deploy/robots/g1/build && ./g1_ctrl --network lo ) &
sleep 3

# --- 2. simulator, NO scripted profile -------------------------------------
( cd $TREE/simulate/build && env \
    UNITREE_DPCBF_MODE=estimated \
    UNITREE_MUJOCO_BAND_LENGTH=0.572 \
    ./unitree_mujoco ) &        # release the band with keys 9/7/8, not a timer
sleep 14                        # model load + mirror dump + DDS up

# --- 3-5. perception -------------------------------------------------------
cd $REPO/ros2
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true &
sleep 2
ros2 launch g1_perception_bringup source_sim.launch.py &
sleep 3
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true &
sleep 6

# --- 6. RViz + relays + overlay -------------------------------------------
ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true \
    overlay:=on dpcbf_config:=$TREE/dpcbf/config/dpcbf_config.yaml &
```

   Order is load-bearing for the same reasons the scripted launcher documents:
   `g1_ctrl` before the simulator, and the sidecar only after
   `/tmp/unitree_mujoco_mirror_model.xml` exists.

7. **Button combinations** *(source, from `deploy/robots/g1/config/config.yaml`)*:
   **`L2 + Up`** → FixStand, then **`R2 + A`** → Velocity. Also `L2 + B` → Passive
   from either state, and `R1 + A` → the Mimic state. On a physical pad these are
   naturally staggered by human timing (~100 ms between the trigger and the
   button), so the 23 ms `Axis` lag that breaks scripted simultaneous chords is
   not an issue — *reasoning from source, not measured*.
8. **Axes** *(source, `XBoxJoystick::update()`)*: `axis_[0]` → `lx` (lateral,
   negated at the seam), `−axis_[1]` → `ly` (**forward**), `axis_[3]` → `rx`
   (yaw, negated at the seam), `−axis_[4]` → `ry` (unused by the seam).
   `axis_[2]`/`axis_[5]` are the LT/RT triggers, `axis_[6]`/`axis_[7]` the d-pad.
   Values are normalised by `1 << (joystick_bits − 1)`, with `joystick_bits: 16`.
   On the `switch` layout the stick indices differ (`axis_[2]` is yaw,
   `axis_[4]`/`axis_[5]` are the triggers).
9. **Does the safety filter modify joystick commands before the controller sees
   them?** **Yes — that is the entire architecture.** The device axes are filtered
   *in place* inside `update()` before `lx()/ly()/rx()` are set, so what gets
   packed into `wireless_remote` and shipped over `rt/lowstate` to `g1_ctrl` is
   already the DPCBF-filtered command. The FSM and the policy never see the raw
   stick. Chord *buttons* are exempt.
10. **Switch oracle ↔ estimated while holding a gamepad?** **No.** The mode is
    read from the environment once, at `UnitreeSdk2BridgeThread` startup, and
    `ObstacleSource`'s mode is const after construction. Changing it requires
    restarting `simulate` — which drops `rt/lowstate` and therefore also
    restarts the FSM.
11. **Is there a CLI/env override for `use_joystick`?** **No.** `param::helper`
    exposes only `--domain_id/-i`, `--network/-n`, `--robot/-r`, `--scene/-s`
    ([`param.h:57-64`](../../simulate/src/param.h#L57-L64)). `use_joystick`,
    `joystick_type`, `joystick_device` and `joystick_bits` are **YAML-only** —
    so the shadow-tree edit above is the only way, and **the tracked
    `simulate/config.yaml` must not be edited**.
12. **Confirming detection and that the FSM predicates fire.**
    - The device: `ls -l /dev/input/js*`, then
      `simulate/build_ros2/jstest /dev/input/js0` (built from
      `src/joystick/jstest.cc`) to see raw axis/button events.
    - Simulator-side: if the device is missing you get
      `Error: Joystick open failed.` and an immediate `exit(1)` — a silent
      failure is not possible here.
    - **The predicates**: `simulate/build_ros2/fsm_button_probe`, run against a
      live `rt/lowstate`. It executes the FSM's **own** `LowState::update()`
      decode and its **own** compiled transition DSL at 1 kHz and exits non-zero
      if a configured transition never fires. This is the only tool that can
      answer the question, because in the failure it was built for the keys
      **arrived perfectly** — 601 pressed ticks each, axis peak 0.999999 — and
      all seven transition predicates fired **zero** times. *Arrival is not
      satisfaction.* (The probe itself was not exercised in the recorded
      session, because the run it gates transitioned on the first attempt.)
    - The seam: `UNITREE_DPCBF_FILTER_LOG=/tmp/capture.bin` on the simulator,
      then `phase4_capture_stats.py stats /tmp/capture.bin`. A non-empty capture
      proves `Filter()` is running; an empty one proves the joystick object was
      never wired.

---

## 9. Frames and coordinate conventions

### 9.1 The tree

```
odom ─────────────────────────────► base_link ──► torso_link ──► mid360_link
 │      sim: sim_mjlidar_bridge          (static, robot_state_publisher, latched)
 │           100 Hz, MuJoCo GT qpos
 │      hw:  dlio_odom_node
 │           ~10 Hz (per-scan publishToROS thread)
 │
 └───────────────────────────────► base_footprint
        base_footprint_publisher, 100 Hz timer, dedup'd by stamp
        ⇒ effective rate = odometry rate (100 Hz sim / ~10 Hz hw)

dpcbf_velocity_plane      ← dpcbf_overlay only; parked beside the robot;
                            VISUALIZATION ONLY, axes are m/s

hw only, from DLIO:  base_link ──► dlio_lidar_link
                     base_link ──► dlio_imu_link
```

| Transform | Publisher | Static/dynamic | Rate |
|---|---|---|---|
| `odom→base_link` | `sim_mjlidar_bridge` / `dlio_odom_node` | dynamic | 100 Hz / ~10 Hz |
| `odom→base_footprint` | `base_footprint_publisher` | dynamic | tracks the row above |
| `base_link→torso_link` | `robot_state_publisher` (xacro) | **static**, latched | once |
| `torso_link→mid360_link` | `robot_state_publisher` (xacro) | **static**, latched | once |
| `base_link→dlio_{lidar,imu}_link` | DLIO | dynamic | hw only |
| `odom→dpcbf_velocity_plane` | `dpcbf_overlay` | dynamic | 10 Hz |

`dlio_lidar_link` / `dlio_imu_link` are named that way on purpose
([`dlio.yaml:20-24`](../src/g1_perception/g1_perception_bringup/config/dlio.yaml#L20-L24)):
DLIO publishes `base_link → <frames/lidar>` itself, and if that were named
`mid360_link` it would **collide** with `robot_state_publisher`'s
`torso_link → mid360_link` and the frame would have two parents.

### 9.2 Does `base_footprint` remove roll and pitch?

Yes, and z. See §5.3.

### 9.3 The Mid-360 extrinsic

**The single source of truth is
[`g1_description/urdf/g1_mid360.xacro`](../src/g1_perception/g1_description/urdf/g1_mid360.xacro)**
(decision §8.3). Shipped values:

```xml
<!-- base_link -> torso_link : MJCF chain offsets at zero waist joints -->
<origin xyz="-0.0039635 0.0 0.044" rpy="0 0 0"/>

<!-- torso_link -> mid360_link : H-1 -->
<origin xyz="0.0002835 0.00003 0.428434"
        rpy="3.14159265 0.000892 0.0"/>
```

Composed: `base_link → mid360_link = (−0.00368, 0.00003, 0.472434)` — which is
exactly `extrinsics/baselink2lidar/t` in `dlio.yaml`, because that file is
*derived* from this one and `t7_hw_extrinsic_guard.py` fails if it drifts.

**The upside-down mount** is `roll = π`. URDF rpy is `R = Rz(y)·Ry(p)·Rx(r)`, so
the sensor's `+z` points downward in the world and its `+y` is flipped — which is
what makes the FOV (−7°…+52° about the sensor's own axis) sweep the ground ahead
of the robot (H-2), and what makes the CropBox z-bound signs counter-intuitive
(§3.6).

The same pose in MJCF is
`<site name="mid360_link" pos="0.0002835 0.00003 0.428434" euler="3.14159265 -0.000892 0"/>`
([`scene_g1.xml:157`](../../src/assets/robots/unitree_g1/xmls/scene_g1.xml#L157)).
**The pitch sign differs** — MJCF `euler` is *intrinsic* xyz, URDF `rpy` is
*extrinsic* xyz, and the two do not commute at `roll = π`. Gate **T7**
(`g1_description/test/t7_extrinsic_guard.py`) therefore compares the two as
**rotation matrices**, not as raw numbers.

**Why the extrinsic must not also go in the Livox driver.**
`MID360_config.json`'s `extrinsic_parameter` is identity **by decree**: the
driver emits points in the sensor frame and TF does the rest. Setting it here as
well would apply H-1 **twice** — a ~0.43 m z error plus a double roll — and
`hw_config_check.py` fails the preflight if it is not identity.

### 9.4 Where each quantity lives

| Quantity | Frame | Units |
|---|---|---|
| `/livox/lidar`, `/points_self_filtered` points | `mid360_link` (z **down**) | m |
| CropBox bounds | `mid360_link` | m |
| `LaserScan` ranges and angles | `base_footprint` — angles are `atan2(y, x)` after the transform, i.e. **heading-referenced**, and `range = hypot(x, y)` is a **horizontal** distance | m, rad |
| `min_height`/`max_height` | `base_footprint` z, ground-relative | m |
| `/raw_obstacles`, `/tracked_obstacles`, `/obstacles_safe` centres and velocities | **`odom`** (extractor's `frame_id: odom` + stamped TF) | m, m/s |
| `dpcbf::ObstacleState.x/.y/.velocity_*` | `odom` ≡ DPCBF "world" (§8.1) | m, m/s |
| `dpcbf::RobotState.x/.y/.phi` | MuJoCo world = `odom` in sim | m, rad |
| `RobotState.sagittal_velocity/.lateral_velocity` | **body** frame (`mj_objectVelocity` with `flg_local=1`) | m/s |
| DPCBF barrier `(x̃, ỹ)` | obstacle **line-of-sight velocity** frame | **m/s** |

### 9.5 Why the parabola is not workspace geometry

The DPCBF barrier is

```
h = x̃ + A·( λ·ỹ² + k_μ·d_safe ),   A = √(s²−1)/r_safe,   λ = k_λ·d_safe/v_safe
```

([`dpcbf_safety_filter.cpp:400-404`](../../dpcbf/src/dpcbf_safety_filter.cpp#L400-L404)),
so `h = 0` is `x̃ = vertex_x − curvature·ỹ²`. But `(x̃, ỹ)` is the **relative
velocity** `v_obs − v_robot` rotated into the obstacle's line of sight
([`:383-386`](../../dpcbf/src/dpcbf_safety_filter.cpp#L383-L386)) — **both axes
are m/s**. Drawing that curve on an `odom` metre grid is a category error, which
is why `dpcbf_overlay` draws it in its own TF frame, `dpcbf_velocity_plane`,
parked beside the robot, with every card labelled "axes are m/s, NOT metres".

The OpenCV `dpcbf_visualizer` *does* draw a world-frame curve. It is the same
velocity-space curve multiplied by `velocity_arrow_seconds = 1.0` and anchored at
the **robot** — metres = (m/s)×(1 s), a one-second-lookahead diagram, not an
envelope around the obstacle. Its own config bounds it in m/s
(`world_parabola_lateral_limit`) on a pane scaled in m. Do not read it as
workspace geometry.

There *is* a real world-frame circle in the picture: the **eCBF** barrier,
`h = ‖p‖² − (r_rob + r_obs)²`
([`:459-461`](../../dpcbf/src/dpcbf_safety_filter.cpp#L459-L461)). Because the
shipped config runs `ecbf_enabled: true` **and** `slack_enabled: true`, both
families shape the command and the overlay draws both — showing only the parabola
would understate what is constraining the robot.

---

## 10. Configuration map

`R` = rebuild needed. **Configs, launch files and the RViz layout are
*installed*, not read from the source tree** — editing one and re-launching
without `colcon build --merge-install --packages-select g1_perception_bringup`
silently runs the old file. This has cost the project at least one live run: the
layout and `viz.launch.py` were both edited, neither was rebuilt, and the run
came up with the old four relays and no overlay — silently, because a launch
argument that selects nothing looks exactly like one that is off.

| Behaviour | File | Parameter | To take effect |
|---|---|---|---|
| LiDAR pattern (sim) | hard-coded | `LivoxGenerator('mid360')` → `mid360.npy`, ~24 000 rays | edit `bridge_node.py`, colcon rebuild, restart sidecar |
| LiDAR rate, range (sim) | `config/sim_mjlidar_bridge.yaml` | `scan_rate: 10.0`, `cutoff_dist: 40.0`, `min_range: 0.1` | **colcon rebuild** + sidecar restart |
| Head-shell mask (sim) | `config/sim_mjlidar_bridge.yaml` | `masked_geom_group: 2` | colcon rebuild + sidecar restart |
| LiDAR rate, format, frame (hw) | `config/livox_driver.yaml` | `publish_freq: 10.0`, `xfer_format: 0`, `frame_id: mid360_link` | colcon rebuild + driver restart |
| LiDAR network (hw) | `config/MID360_config.json` | `host_net_info.*`, `lidar_configs[0].ip`, `extrinsic_parameter` (**must stay identity**) | colcon rebuild + driver restart |
| **CropBox limits** | `config/cropbox_self_filter.yaml` | `min_x/max_x/min_y/max_y/min_z/max_z`, `negative`, `input_frame`, `output_frame` | **colcon rebuild** + container restart |
| VoxelGrid leaf | `launch/perception.launch.py:45` (**inline, not YAML**) | `leaf_size: 0.05` | colcon rebuild + container restart; and `voxel:=on` to insert it at all |
| **`min_height`/`max_height`** | `config/pointcloud_to_laserscan.yaml` | `min_height`, `max_height` | **colcon rebuild** + container restart |
| Scan angular range/increment, ranges | same | `angle_min/angle_max/angle_increment/range_min/range_max/use_inf/inf_epsilon/scan_time/transform_tolerance` | colcon rebuild + container restart |
| Projection target frame | same | `target_frame: base_footprint` | colcon rebuild + container restart |
| Extractor radius limits & grouping | `config/obstacle_detector.yaml` | `max_circle_radius: 0.60`, `radius_enlargement: 0.17`, `min_group_points: 5`, `max_group_distance: 0.10`, `distance_proportion`, `max_split_distance`, `max_merge_*` | colcon rebuild + container restart |
| Extractor output frame | same | `frame_id: odom`, `transform_coordinates: true` | colcon rebuild + container restart |
| Tracker process/measurement variance | same | `process_variance: 0.0001`, `process_rate_variance: 0.03`, **`measurement_variance: 1.0`** | colcon rebuild + container restart |
| Track confirmation/deletion | same | `tracking_duration: 1.0`, `sensor_rate: 10.0` (⇒ counter size 10), `min_correspondence_cost: 0.3`, `radius_residual_weight: 0.3`, `std_correspondence_dev: 0.15` | colcon rebuild + container restart |
| **Fixed inflation** | `config/safety_obstacle_filter.yaml` | `fixed_inflation: 0.051` | colcon rebuild + container restart |
| **`k_sigma`** | same | `use_covariance: false`, `k_sigma: 2.748`, `sigma_max: 0.50` | colcon rebuild + container restart |
| Safety gates | same | `max_age: 0.30`, `min_radius: 0.20`, `max_circle_radius: 0.60`, `latency_horizon: 0.12`, `v_max_obstacle: 1.5` | colcon rebuild + container restart |
| **`p_max`** | `dpcbf/config/dpcbf_config.yaml` → `robot.p_max: 3.0` | — | edit the **shadow copy**; **restart `simulate`**. No rebuild (YAML read at load). |
| **Staleness thresholds** | ⚠ **C++ defaults**, `obstacle_buffer.h:41-44` | `max_age_s 0.30`, `fade_out_s 0.30`, `hold_after_stale_s 1.0` | **plain CMake rebuild of `simulate`** (see the note below) |
| **DPCBF mode** | env `UNITREE_DPCBF_MODE` | `oracle`\|`shadow`\|`estimated` | restart `simulate`. No rebuild. |
| **Obstacle priority** | `dpcbf_config.yaml` → `qp_parameters.obstacle_priority: 1` | 0 = nearest first, 1 = closing-alignment first | shadow-copy edit + restart `simulate` |
| QP size / gains | same | `default_num_constraints: 10`, `alpha: 2.0`, `k_a_s/k_a_l`, `reference_control_frequency_hz: 500.0` | shadow-copy edit + restart `simulate` |
| **eCBF enabled** | same | `ecbf_enabled: true`, `alpha_ecbf: 100.0` | shadow-copy edit + restart `simulate` |
| **Slack enabled** | same | `slack_enabled: true`, `slack_weight: 1e6` (and `odcbf_enabled: false`) | shadow-copy edit + restart `simulate` |
| Robot geometry / limits | same | `r_rob: 0.30`, `s: 1.05`, `k_mu: 1`, `k_lambda: 0.5`, `eps_v/eps_d: 0.05`, `v_s_*/v_l_*/w_*`, `a_s_*/a_l_*` | shadow-copy edit + restart `simulate` |
| Obstacle field | same | `dynamic_obstacles.{count,radius_range,speed_range,arena,random_seed}` | shadow-copy edit + restart `simulate`; or `walk_scenarios.py W1..W4` |
| **OpenCV visualizer** | same | `visualization.enabled: true`, `window_*`, `refresh_hz: 30.0` | shadow-copy edit + restart `simulate` |
| **RViz overlay enabled** | launch arg | `viz.launch.py overlay:=on\|off` (default **on**), `dpcbf_config:=`, `overlay_log:=`, `rviz_config:=` | **colcon rebuild if you edit the launch file**; otherwise just relaunch |
| RViz layout | `rviz/perception.rviz` | display enable/disable, camera | **colcon rebuild** — or pass `rviz_config:=<copy>` and skip it |
| **Ground segmentation** | `bringup.launch.py` `ground_seg:=off\|patchwork` | — | **no effect — the argument is a no-op (§4)** |
| **`use_joystick`** | `simulate/config.yaml` | `use_joystick`, `joystick_type`, `joystick_device`, `joystick_bits` | edit the **shadow copy**; restart `simulate`. **No CLI or env override exists.** |
| **Elastic band** | `simulate/config.yaml` `enable_elastic_band` + env `UNITREE_MUJOCO_BAND_LENGTH`, `UNITREE_MUJOCO_BAND_RELEASE` | | shadow-copy edit / env; restart `simulate` |
| **Walking profile** | `config/walk_profile.txt` via env `UNITREE_MUJOCO_SCRIPTED_COMMANDS` | | restart `simulate`; harnesses copy it into the shadow tree, so **rebuild only if you want the launcher to pick up a new source version** |
| Simulator scene / domain / interface | `simulate/config.yaml` or CLI `-r/-s/-i/-n` | | restart `simulate` |
| DLIO everything | `config/dlio.yaml` | `use_sim_time: false`, `extrinsics/*`, `frames/*`, `odom/*`, `pointcloud/deskew` | colcon rebuild + DLIO restart |

**Two things in that table deserve emphasis.**

1. **`config/dpcbf_ros_adapter.yaml` is dead.** Nothing reads it. Grepped across
   the entire repo: the only match for the filename is a CMake target name.
   `simulate/src/main.cc:941-944` constructs `ObstacleSource::Config` with
   `mode` and `oracle` only, so `staleness`, `topic`, `diagnostics_topic` and
   `p_max` all come from the **C++ defaults**. Those defaults happen to equal
   the YAML values (0.30 / 0.30 / 1.0), so the file is *accurate documentation*
   and *not* a control surface. Changing the staleness ladder today means editing
   `obstacle_buffer.h` and rebuilding both the colcon workspace and `simulate`.
2. **The RViz overlay defaults to `on`** in `viz.launch.py:36-37`, contradicting
   the runbook's phrasing that it lives "behind `overlay:=on|off`". You get it
   unless you ask for `overlay:=off`. Its `dpcbf_config` parameter is **required
   by design** — the node exits at startup without it, because deriving the
   geometry from ROS defaults would let the viewer drift from the filter
   silently.

---

## 11. Measured, inferred, and unverified

**Read this as a statement about the repository's evidence, not about this
report's session.** Nothing was executed here; the "verified" rows below are
what [`operator_runbook.md`](operator_runbook.md) and `../evidence/` record for
the 2026-08-02 session, and I have checked that the code they describe is the
code that is checked in.

### 11.1 Verified live on the dev machine

- The full bag-replay chain to `/obstacles_safe`: 290/290 frames at 10.0 Hz,
  drop fraction 0.0, `cloud→safe` p50 0.76 ms / p95 1.28 ms, container CPU 2.9 %.
- The live sim stack with the 90-obstacle field: 450/450 frames, `cloud→safe`
  p50 1.48 ms / p95 2.18 ms, container 4.6 %, sidecar 30.3 %.
- **T9** TF availability: 289/0 and 449/0 misses.
- **T1** oracle equivalence: 3.79 s, Passed, 38 402 calls.
- **`dpcbf_boundary_recomputation`**: 3.07 s, Passed — 38 402 ticks, 214 085
  selected-obstacle rows, every delta exactly 0 (bit-exact, not "within
  tolerance").
- **T2** wall occlusion: 17 608 expected hits, 0 through-wall, max range error
  0.00 mm. **T3** pattern envelope: elevation −7.2123…+52.1640°, 99.782 % strict.
- **T10** DDS coexistence: PASS, exactly one `libddsc.so.0.10.2` in the map.
- The whole suite: **97 tests, 0 failures, 2 expected skips** (4 min 54 s).
- **Closed-loop walking**, W1, oracle: 35.22 s window, no fall, 15.139 m path,
  0.43 m/s mean, `tilt_max_rad` 0.044, **`margin_violation_events` 0**,
  `clearance_min_m` 0.0188, `tracked_to_gt_p50_mm` 74.2.
- The offline A/B + containment sweep reproducing every recorded §17.3 ratio
  exactly (worst 0.8265 on `s3_swarm`, gate ≥ 0.95 — i.e. **failing**).
- The circle-fit bias sweep, including the finding that the **G2 sensing limit is
  range-dependent**: r = 0.55 m is seen at 2 m and dropped at 3 m and 4 m, so
  props must be **r ≤ 0.52 m** to be visible across the working range.
- RViz headless at 31 fps under software GL on a private Xvfb, with a real
  screenshot.
- The `dpcbf_overlay` layer, its `boundary_check join` (1186/1186 joined, 0
  dropped; selected set identical on 63.3 % of ticks, Jaccard 0.061), and its
  CPU (1.44 % W1 / 2.02 % W4).

### 11.2 Verified by tests or replay only

- **T4** static accuracy, **T5** dynamic tracking, **T8** replay determinism,
  **T7/T7-hw** extrinsic guards, the hardware cloud contract, and DLIO wiring —
  all green inside the suite, none observed live.
- The staleness ladder and extrapolation: covered by `test_obstacle_buffer`
  gtests; the **T6 end-to-end drill was not run** in the recorded session, so the
  DEGRADE-at-0.300 s / STOP-at-0.600 s / never-emptied-set figures are the
  Phase-4 record, not a fresh measurement.
- The hardware source path: exercised **only** through `hw_source_stub.py`, which
  asserts wire format, QoS and TF-tree shape and explicitly not odometry quality.
- `hw_config_check.py` exits 2 on this machine — the *correct* answer here, which
  is why the CTest maps it to SKIP.
- The one-line `bringup.launch.py`, `foxglove_bridge`, `fsm_button_probe`
  standalone, and `hw_source_stub.py` by hand: **not executed**; the composition
  is read, not measured.
- The from-scratch build (~40 min) and the `simulate`/`deploy` CMake builds:
  transcribed, not re-executed.

### 11.3 Not verified on hardware — the open list

| Area | State |
|---|---|
| **Mid-360 network config (Q-1)** | `MID360_config.json` still carries upstream placeholder IPs (`192.168.1.5`, `192.168.1.12`). `hw_config_check.py` fails on it deliberately. **Blocking for 5B block 1.** |
| **`measurement_variance`** | Shipped at the inherited **1.0 m²**, which asserts a one-metre 1σ LiDAR measurement. Real sim scatter is 1.775e-06 m² (1σ = 1.3 mm) — but that is a noiseless analytic raycast, so neither endpoint is shippable. `calibrate_k_sigma.py` **refuses** to emit a number while pooled σ p50 > 0.25 m and prints why. **R must come from hardware before anyone calibrates `k_sigma`.** |
| **σ / `k_sigma` path** | Wired (patch 0007 + `gating.h`) but `use_covariance: false`. `k_sigma: 2.748` is the abandoned branch's value, a placeholder. |
| **Hardware IMU timing and bias** | 3 s still-calibration, PTP-vs-host timestamping (no config field — it is read from packet headers), gyro/accel bias stability: all unmeasured. |
| **Lidar→IMU lever arm** | `(0.011, 0.02329, −0.04412)` is the Mid-360 *manual* constant, **not verified against this unit**. |
| **DLIO performance** | Never run against real data. Its `odom/preprocessing/cropBoxFilter/size: 1.0` is a ±1 m cube around the LiDAR applied *before* the extrinsic, so on the G1 it also removes the ground within 1 m — untuned. |
| **TF `odom→base_link` at ~10 Hz on hardware** | vs 100 Hz in sim. The extractor's stamped TF lookup and `base_footprint_publisher`'s dedup both inherit that rate. Consequences for T9 on hardware: unmeasured. |
| **Patchwork++ / rough terrain** | **Not imported, not wired, not tested.** `ground_seg:=patchwork` is a no-op. Flat-ground validity does not survive slopes (§4). |
| **Physical joystick** | **Never exercised.** No gamepad on this machine, no test. §8.2 is source-supported behaviour only. |
| **T6 stale-state visual appearance** | The colour shift to purple and the `*** stop: SET RETAINED ***` banner are implemented and driven by `/dpcbf/status`'s level, but the drill was not run — the DEGRADE/STOP *appearance* is written, not seen. |
| **aarch64 target build** | 10 of 18 packages build under qemu-user emulation. An upstream `ament_cmake` `_lib` cache-shadow bug stops the rest; `tools/diagnose_ament_export_libraries.py` recognises it but **no workaround has been shown correct**. |
| **Hardware latency and CPU budget** | Every §17.4 number is a dev-machine, sim-source measurement. The Orin/onboard-PC budget is unmeasured. |
| **S3/S4 containment** | The known real limit: 70.9 % (S3) and 13.2 % (S4) containment at F = 0 mm, and the offline A/B's worst performance ratio 0.8265 against a ≥ 0.95 gate. The 379.6 mm "calibrated inflation" the sweep prints is **not a shipping value** — it is the 99.9th percentile of a distribution containing two populations, a calibratable steady-state bias (which `fixed_inflation = 0.051` covers) and occlusion-coast/merged-arc transients (which no fixed term can). |

---

## 12. Reconciliation summary — doc vs implementation

Six places where the architecture document and the code disagree, ranked by how
much they would mislead someone operating the stack:

1. **`ground_seg:=patchwork` does nothing.** The argument exists and validates;
   there is no node, no import, no `/points_no_ground`. §4.
2. **`config/dpcbf_ros_adapter.yaml` is not loaded by anything.** The staleness
   ladder is compiled-in. §10.
3. **`/livox/imu` has no simulation counterpart** — correctly noted in §7.1 of
   the doc, but easy to miss, and it means the sim path uses **no IMU at all**. §5.
4. **`xfer_format` 2 does not work in ROS 2 and the driver cannot dual-publish** —
   the doc carries the correction inline (§5.6, 2026-08-01); the YAML repeats it.
   Q-4's FAST-LIO bake-off is a separate capture session, not a co-run.
5. **`/livox/lidar` QoS is asymmetric** — SensorData in sim, Reliable depth 256
   on hardware, unparameterisable. Recorded as a deviation, not patched.
6. **`viz.launch.py overlay` defaults to `on`**, and `perception.rviz` opens
   top-down 2-D with `LivoxCloud` and `RawObstacles` off — layout state that
   changed after most of the doc was written.

And one thing that is *not* a divergence but is the most common operational trap:
**launch files, YAML and the RViz layout are installed artefacts.** Editing them
without `colcon build --merge-install --packages-select g1_perception_bringup`
runs the previous version, silently.
