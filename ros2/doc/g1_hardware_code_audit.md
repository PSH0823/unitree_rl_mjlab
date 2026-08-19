# Hardware-readiness source audit

*What the repository actually contains on the hardware path, as of commit
`801efc6` on `dpcbf_perception_ros2`, and what of it has been verified with
what.*

Every claim below is labelled:

| Label | Meaning |
|---|---|
| **[src]** | verified by reading the pinned source in this workspace |
| **[test]** | verified by a test in the suite (bench, stub or fixture data) |
| **[hw]** | measured on real hardware — **nothing in this document carries this label yet** |
| **[not measured]** | knowable, but nobody has measured it |
| **[blocked]** | cannot be determined without hardware or an operator answer |

The rule this document exists to enforce: **the hardware path is unverified
until real data are observed.** It builds, launch files exist, stub tests
pass, the simulation path works and the topics appear in `ros2 topic list` —
none of that is evidence about a G1.

Companion documents: [`g1_first_day_field_runbook.md`](g1_first_day_field_runbook.md)
(the field session), [`operator_runbook.md`](operator_runbook.md) (development
operations), and [`phase5a_seam_audit.md`](phase5a_seam_audit.md) (the sim/hw
contract diff, still accurate).

---

## 1. The hardware source path, edge by edge

```
Mid-360 ──UDP──▶ livox_ros_driver2 ──▶ /livox/lidar ─┬─▶ DLIO ──▶ /odom + TF odom→base_link
                                     └─▶ /livox/imu ─┘        │
                                                              ▼
robot_state_publisher ──▶ /tf_static           base_footprint_publisher ──▶ TF odom→base_footprint
                                                              │
/livox/lidar ──▶ CropBox ──▶ /points_self_filtered ──▶ pointcloud_to_laserscan ──▶ /scan
                                                              │
        /scan ──▶ obstacle_extractor ──▶ /raw_obstacles ──▶ obstacle_tracker ──▶ /tracked_obstacles
                                                              │
                             /tracked_obstacles ──▶ safety_obstacle_filter ──▶ /obstacles_safe
                                                              │
                                                        (consumed by NOBODY)
```

### 1.1 `Mid-360 → livox_ros_driver2`

| | |
|---|---|
| package / exe | `livox_ros_driver2` / `livox_ros_driver2_node` **[src]** |
| pin | tag 1.2.6 (`13eb05e4`), patches 0005 + 0008 **[src]** |
| launch | `source_hw.launch.py`, node name `livox_lidar_publisher`, unnamespaced **[src]** |
| config | `config/livox_driver.yaml` + `config/MID360_config.json` (path injected by the launch so the two cannot drift) **[src]** |
| transport | raw UDP, ports 56100–56501, host addresses from `host_net_info` **[src]** |
| **network config** | **PLACEHOLDER IPs** (`192.168.1.5` / `192.168.1.12`) — **[blocked]** on Q-1 |
| bind failure behaviour | logs `bind failed` / `Init lds lidar fail!`, does **not** exit, creates **no** topic, ignores SIGINT/SIGTERM **[test]** (dev machine, no device) |
| verified with real hardware | **no** |

### 1.2 `livox_ros_driver2 → /livox/lidar`

| | |
|---|---|
| type | `sensor_msgs/PointCloud2` **[src]** |
| layout | 7 fields, `point_step` **26**, `#pragma pack(1)`: x,y,z,intensity FLOAT32 @0/4/8/12; tag,line UINT8 @16/17; timestamp FLOAT64 @18 **[src]** |
| `xfer_format` | **0 is the only usable value in ROS 2** — 1 is CustomMsg *instead of* PointCloud2, 2 is ROS1-only (`#if 0`'d) **[src]** |
| rate | 10 Hz (`publish_freq: 10.0`) **[not measured]** on hardware |
| `frame_id` | `mid360_link`, from `livox_driver.yaml`. Driver default is `frame_default`; its example launches say `livox_frame` **[src]** |
| publisher QoS | **RELIABLE, depth 256**, not parameterised — a §7.1 deviation, recorded not patched; Reliable-pub ↔ BestEffort-sub is the compatible direction **[src]** |
| timestamp source | LiDAR clock if PTP/GPS-synced, else **host clock at packet reception** (`pub_handler.cpp::GetEthPacketTimestamp`). No config field selects this **[src]**; which is live is **[blocked]** on measurement |
| topic creation | **lazy** — created on first data, so "no topic" ≠ "no sensor" **[src]** |
| end-to-end through this layout | 278 driver-format clouds → 277 `/obstacles_safe` **[test]** (`test_hw_source_contract`, synthetic geometry) |
| point density / rosette distribution | Q-3, **[not measured]** |

### 1.3 `livox_ros_driver2 → /livox/imu`

| | |
|---|---|
| type | `sensor_msgs/Imu`, RELIABLE depth 256 **[src]** |
| rate | nominally 200 Hz — device fact, **[not measured]** |
| `frame_id` | `mid360_link` via patch 0005; upstream hardcoded `livox_frame` for the IMU only **[src]** |
| sim counterpart | **none.** The sim sidecar never published this topic; DLIO does not start without it. The largest "free in sim" gap **[src]** |
| stub | `hw_source_stub.py` synthesises a stationary IMU so DLIO can be wired-tested **[test]** |
| shares a clock with the cloud? | **[blocked]** — measured by `hw_source_probe.py` on the robot |

### 1.4 `/livox/{lidar,imu} → DLIO → /odom`

| | |
|---|---|
| package / exe | `direct_lidar_inertial_odometry` / `dlio_odom_node` **[src]** |
| pin | `feature/ros2` @ `c8acc371`, patch 0006 **[src]** |
| launch | `source_hw.launch.py`, `lio:=dlio` **[src]** |
| remaps | `pointcloud→/livox/lidar`, `imu→/livox/imu`, `odom→/odom`, rest under `/dlio/` **[src]** |
| cloud sub QoS | SensorData keep_last(1) via patch 0006 (upstream subscribed Reliable, which never matches this stack's best-effort publishers) **[src]** |
| `/odom` pub QoS | Reliable depth 10 via patch 0006 (upstream 1) **[src]** |
| rate | 100 Hz `publishPose()` wall timer — **[test]** measured 100.0 Hz on the bench |
| stamp | `imu_stamp`, i.e. **0 until the first IMU message arrives** — a rate computed from header stamps over startup is nonsense **[src]** |
| config | `config/dlio.yaml`; `use_sim_time: false` (upstream ships **true**) **[src]** |
| **TF listener** | **DLIO has none.** `extrinsics/baselink2{lidar,imu}` are the *only* definition of where `base_link` is. Left identity, `odom→base_link` is really `odom→mid360_link`: 0.472 m and roll = π out **[src]** |
| extrinsics provenance | derived from `g1_mid360.xacro`; `t7_hw_extrinsic_guard.py` re-derives on every build **[test]** |
| lidar→IMU lever arm | Livox **manual constant**, not measured on this unit **[not measured]** |
| odometry quality, drift, IMU bias stability | **[not measured]** — the stub's IMU is synthetic, so no test has ever exercised real odometry |
| preprocessing crop | `odom/preprocessing/cropBoxFilter/size: 1.0` — a ±1 m cube around the LiDAR applied before the extrinsic, so it also removes the ground within 1 m. Odometry-only **[src]**, unverified indoors **[not measured]** |

### 1.5 `DLIO → TF odom→base_link`

| | |
|---|---|
| rate | **scan rate ~10 Hz, not `/odom`'s 100 Hz** — the broadcasts live in the per-scan `publishToROS()` thread, not the 100 Hz `publishPose()` timer **[src]**, asserted by `test_dlio_wiring` **[test]** |
| measured cost | cloud→`/scan` p50 9.24 / p95 12.81 ms; cloud→`/obstacles_safe` p50 9.61 / p95 13.19 ms (vs 0.48/1.28 in sim) — the tf2 MessageFilter waiting for the bracketing TF **[test]**, dev machine |
| conditional fix | patch **P-5** (broadcast from `publishPose()` too) — held pending the on-target latency number **[not measured]** |
| startup transient | ~31 clouds dropped with *"timestamp … earlier than all the data in the transform cache"* before DLIO's first TF. Expected, not a fault **[test]** |
| extra broadcasts | `base_link→dlio_lidar_link`, `base_link→dlio_imu_link`, unconditional; names must stay disjoint from §8.2 or `mid360_link` gets two parents **[src]**, guarded **[test]** |

### 1.6 `robot_state_publisher` (static robot TF)

| | |
|---|---|
| launch | `description.launch.py` **[src]** |
| source | `g1_description/urdf/g1_mid360.xacro` — the single source of truth for extrinsics (§8.3) **[src]** |
| chain | `base_link → torso_link` (fixed, nominal waist pose) `→ mid360_link` (H-1, roll = π, upside-down mount) **[src]** |
| `base_link→torso_link` | a **fixed approximation**: the true transform moves with three waist joints. In sim the waist genuinely did not move; on hardware it will **[src]**, error **[not measured]** |
| H-1 vs the physical mount | **[blocked]** — stage 4 measures it |

### 1.7 `base_footprint_publisher`

| | |
|---|---|
| package / exe | `g1_perception_utils` / `base_footprint_publisher` **[src]** |
| input | TF `odom→base_link` at `TimePointZero` (latest), 100 Hz wall timer, **deduplicated by stamp** **[src]** |
| output | TF `odom→base_footprint` |
| effective rate on hardware | **inherits DLIO's ~10 Hz**, not the 100 Hz in §7.1 (which described the sim sidecar) **[src]** |
| trap | fed a *static* `odom→base_link`, it emits `base_footprint` exactly once at time 0 and every later lookup extrapolates into the future — the whole chain then produces nothing, silently **[test]** |

### 1.8 CropBox self-filter

| | |
|---|---|
| package / plugin | `pcl_ros` 2.6.1 / `pcl_ros::CropBox`, composed **[src]** |
| launch | `perception.launch.py`, remaps `input→/livox/lidar`, `output→/points_self_filtered` **[src]** |
| config | `config/cropbox_self_filter.yaml` |
| frame | the **cloud frame** (`mid360_link`); `input_frame`/`output_frame` empty so **no TF is involved** **[src]** |
| box | xy ±0.40, z −0.55…0.45, `negative: true` **[src]** |
| provenance of the box | **sim-interim** — fitted to wrist returns in a *simulated* grounded pose. Not hardware **[not measured]** |
| output QoS | SensorData via patch 0002 **[src]** |

### 1.9 `pointcloud_to_laserscan`

| | |
|---|---|
| pin | humble, `59bf996f`, composed **[src]** |
| remaps | `cloud_in→/points_self_filtered`, `scan→/scan` **[src]** |
| sub QoS | SensorData keep_last(input_queue_size); pub QoS SensorData **[src]** |
| target frame | `base_footprint` — a tf2 MessageFilter **at the cloud stamp** **[src]** |
| height band | `min_height 0.15`, `max_height 1.60`, applied **after** transforming to `base_footprint` **[src]** |
| `range_min` | 0.3 m, **horizontal distance in `base_footprint`** **[src]** |
| flat-floor validity | the height band is the **only** floor rejection in the system **[src]**; behaviour on a real floor **[not measured]** |
| `transform_tolerance` | 0.05 s **[src]** |

### 1.10 `obstacle_extractor` / `obstacle_tracker`

| | |
|---|---|
| package | `obstacle_detector` (our fork), patches 0003/0004/0007/0009, composed **[src]** |
| extractor sub | `/scan`, **SensorDataQoS** (upstream's Reliable never matched a best-effort laser publisher) **[src]** |
| extractor pub | `/raw_obstacles`, `rclcpp::QoS(5)` **[src]** |
| TF | lookup **at the scan stamp**, 0.1 s tolerance, output transformed to `odom` **[src]** |
| tracker | measurement-driven (patch 0004): predict+correct on arrival, dt from header stamps, no wall timer **[src]** |
| tracker pub | `/tracked_obstacles`, `rclcpp::QoS(5)`, with per-track covariance (patch 0007) **[src]** |
| accuracy | ±2 cm walls, centre/radius bias characterised — **all in simulation** **[test]** |
| hardware centre/radius/velocity accuracy | **[not measured]** |
| `measurement_variance: 1.0` | **known wrong** (asserts 1 m 1σ); every track born at σ = 1.0 m **[src]** |

### 1.11 `safety_obstacle_filter`

| | |
|---|---|
| package / plugin | `safety_obstacle_filter::SafetyObstacleFilterNode`, composed **[src]** |
| sub / pub | `/tracked_obstacles` Reliable depth 5 → `/obstacles_safe` Reliable depth 1 **[src]** |
| `fixed_inflation` | 0.051 m, calibrated **in simulation** against the sim circle-fit bias **[test]** |
| `use_covariance` | **false**; `k_sigma: 2.748` is an abandoned branch's placeholder **[src]** |
| every drop path | counted with a throttled warning **[src]** |
| hardware calibration | **[not measured]** — needs surveyed fixtures |

### 1.12 `/obstacles_safe` → nothing

`dpcbf_ros_adapter` is a **static library**, not a node (`add_library(...
STATIC)`) **[src]**. Nothing in any hardware launch loads it. `/obstacles_safe`
is published and consumed by no process. That is the deliberate endpoint of
this phase.

---

## 2. Target-architecture build readiness

**The target architecture is [blocked]** on Q-1 — item 1.5 of the preflight
checklist. Everything below is conditional on the answer.

### 2.1 What is known about each package

| Package | Build | Pin | Patches | Native/vendored dep | Arch-specific code | System dep | sudo | Known target blocker | Built on the target? |
|---|---|---|---|---|---|---|---|---|---|
| `cyclonedds` 0.10.2 | CMake/colcon | SHA | — | — | none known | OpenSSL | apt for deps | `find_package(OpenSSL)` fails inside the ROS config chain; fixed by explicit `OPENSSL_*` args | **no** |
| `cyclonedds-cxx` 0.10.2 | CMake/colcon | SHA | — | cyclonedds | none known | — | — | — | **no** |
| `rmw_cyclonedds` humble | ament | SHA | — | cyclonedds | none known | — | — | — | **no** |
| `unitree_sdk2` | CMake | SHA | — | **vendored prebuilt libs** | **yes** — `lib/{x86_64,aarch64}/` and `thirdparty/lib/{x86_64,aarch64}/`; both arches ship **[src]** | — | — | the vendored `.so`s exist for both targets, so this is *not* expected to block — but it has never been linked on aarch64 hardware | **no** |
| `unitree_dds_wrapper` | header copy | SHA | 0001 | — | none | — | — | — | **no** |
| `Livox-SDK2` v1.3.1 | plain CMake, COLCON_IGNOREd | tag | — | — | none known | — | — | driven into the prefix by `livox_sdk2_vendor`; upstream installs to `/usr/local` | **no** |
| `livox_ros_driver2` 1.2.6 | ament | tag | 0005, 0008 | Livox-SDK2 | none known | — | — | needs `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`; upstream `build.sh` would `rm -rf` this workspace | **no** |
| DLIO `feature/ros2` | ament | SHA | 0006 | PCL, Eigen | none known | `libpcl-dev`, `libeigen3-dev` | apt | — | **no** |
| `pcl_ros` 2.6.1 | ament | tag | 0002 | PCL, VTK | none known | `libpcl-dev`, `libvtk9-dev` | apt | VTK imported targets exist for `RelWithDebInfo` only; PCL includes Eigen unqualified | **no** |
| `pointcloud_to_laserscan` | ament | SHA | — | — | none | — | — | intermittent configure failure under the parallel executor + `--merge-install`; hence `--executor sequential` | **no** |
| `obstacle_detector_2` (fork) | ament | SHA | 0003,0004,0007,0009 | Armadillo | none known | `libarmadillo-dev` | apt | — | **no** |
| `g1_*`, `safety_obstacle_filter`, `dpcbf_ros_adapter`, `sim_msgs` | ament | in-repo | — | `dpcbf/` headers, yaml-cpp | none | — | — | — | **no** |

### 2.2 The emulated aarch64 result is not a validation

An aarch64 rootfs under qemu-user (`tools/arm64_emulate.sh`, no root, no
binfmt) built **10 of 18 packages**. The remaining 8 were stopped by an
upstream `ament_cmake` bug in which a package reports it cannot find **its own**
library; it is package-set dependent, characterised in
`tools/diagnose_ament_export_libraries.py`, and **unsolved** — that file is
diagnostic only and offers no workaround shown to be correct.

**Do not describe this as a successful aarch64 build.** It is a partial build
under emulation on a machine that is not the robot, and it is the single
largest schedule risk in this phase.

### 2.3 What to do about it

`tools/build_target.sh --check` runs the environment preflight; without
`--check` it builds, encoding all five known environment faults with the error
each prevents, and ends with a triage list keyed by error message. Installing
the apt dependencies needs root **on the target**. Do this before the robot is
powered.

---

## 3. Dead or misleading configuration

Everything here **looks active and is not**. Each is a way for a session to
believe something false.

| # | Item | Reality | Status after this phase |
|---|---|---|---|
| 1 | `ground_seg:=patchwork` in `bringup.launch.py` | a **no-op** — nothing ever read the argument **[src]** | **now raises** with an explanation; the hardware-only launch has no such argument |
| 2 | Patchwork++ | **not in `deps.repos`, not imported, not built, not launched** — the only reference in the tree is the rejection message **[src]** | documented as absent |
| 3 | `/points_no_ground` | **does not exist**; no node publishes or subscribes it **[src]** | documented as absent |
| 4 | `config/dpcbf_ros_adapter.yaml` | **loaded by nothing.** No launch file, node or test references it **[src]** | documented; left in place as the Appendix-A record |
| 5 | Sim `/livox/imu` | **never existed** — the sim sidecar did not publish it, so no sim test exercised the IMU path **[src]** | documented |
| 6 | `/livox/lidar` publisher QoS | **Reliable depth 256**, not the SensorData in §7.1 **[src]** | recorded deviation; compatible direction, subscribers all best-effort |
| 7 | `use_covariance: false` | the σ-inflation path is live code but **disabled** **[src]** | must stay off until hardware calibration |
| 8 | `measurement_variance: 1.0` | **wrong value**: asserts a 1 m 1σ measurement **[src]** | must be derived from hardware before `k_sigma` |
| 9 | `k_sigma: 2.748` | an **abandoned branch's** number **[src]** | placeholder |
| 10 | `fixed_inflation: 0.051` | calibrated, but **in simulation** **[test]** | initial value only |
| 11 | `MID360_config.json` IPs | **upstream placeholders** **[src]** | preflight exits 2 |
| 12 | `dlio.yaml` `cropBoxFilter/size: 1.0` | upstream default; removes the ground within 1 m for odometry **[src]** | **[not measured]** indoors |
| 13 | CropBox bounds | **sim-interim**, from a simulated grounded pose **[src]** | retune from stage 7/8 data |
| 14 | §7.1's "100 Hz" TF rows | describe the **sim sidecar**; hardware is ~10 Hz **[src]** | corrected in the 5A audit |
| 15 | `record.launch.py` topic list | includes sim-only `/clock`, `/sim/gt_obstacles` **[src]** | `record_hw.launch.py` added with the hardware list + metadata |
| 16 | `mode:=oracle\|shadow\|estimated` | Phase-4 adapter vocabulary, meaningless without DPCBF in the loop **[src]** | absent from the hardware-only launch |

---

## 4. Existing tests vs missing hardware tests

| Property | Existing test | Stub only | Simulation only | Real hardware required |
|---|---|---|---|---|
| Livox point format (7 fields, step 26) | `test_hw_source_contract` | ✔ | | to confirm the real driver matches |
| LiDAR publisher/subscriber QoS match | `test_hw_source_contract` wire-check | ✔ | | ✔ |
| IMU QoS | — | ✔ (synthetic) | | ✔ |
| IMU rate | — | ✔ | | ✔ |
| IMU bias stability | — | | | ✔ |
| TF tree shape | `test_dlio_wiring` | ✔ | | ✔ |
| Extrinsic consistency (xacro ↔ dlio.yaml) | `t7_hw_extrinsic_guard` (CTest) | | | — *config-level only* |
| Extrinsic vs the **physical** mount | — | | | ✔ |
| DLIO subscription wiring / TF split | `test_dlio_wiring` | ✔ | | |
| DLIO odometry quality / drift | — | | | ✔ |
| Timestamp synchronisation (host vs device) | — | | | ✔ |
| Stamp monotonicity | `hw_offline_gates` (core logic) | | | ✔ (on real stamps) |
| Self-hit filtering | — | | ✔ (sim wrists) | ✔ |
| Obstacle centre accuracy | `test_detection_static` | | ✔ | ✔ |
| Radius accuracy | `circle_fit_sweep` | | ✔ | ✔ |
| Velocity accuracy | `test_tracking_dynamic_*` | | ✔ | ✔ |
| CPU | `proc_cpu.py` | | ✔ dev machine | ✔ on target |
| Latency (cloud→safe) | `phase4_latency_probe` | ✔ dev bench | ✔ | ✔ on target |
| Packet loss | — | | | ✔ |
| TF lookup availability at the cloud stamp | `hw_tf_probe.py` (new) | | | ✔ |
| Real walking motion | `walk_ab_run.sh` | | ✔ | ✔ |
| Placeholder IP rejection | `hw_offline_gates` **(new)** | | | — |
| Source vs installed drift | `hw_offline_gates` **(new)** | | | — |
| Perception launch has no command publisher | `hw_offline_gates` **(new)** | | | — |
| Recording metadata completeness | `hw_offline_gates` **(new)** | | | — |
| `ground_seg:=patchwork` rejection | `hw_offline_gates` **(new)** | | | — |
| Diagnostics no-data rule | `hw_offline_gates` **(new)** | | | — |
| Launch shutdown clean | — | | | ✔ — see below |

**Clean shutdown is not testable on the bench for the driver.** The pinned
`livox_ros_driver2` ignores SIGINT and SIGTERM when it fails to bind, and its
behaviour on a *successful* bind is unobserved. Stage 3 of the experiment
records what SIGINT actually does; until then, the documented teardown is
`pkill -9 -f livo[x]`. Presenting a bench shutdown test as evidence here would
be exactly the kind of claim this audit exists to prevent.

---

## 5. Summary: what a first session must establish

Nothing on the hardware path carries a **[hw]** label. In dependency order,
the first session must produce the first ones:

1. the target architecture and a **successful on-target build** (§2);
2. this robot's **network configuration**, with the preflight exiting 0 (§1.1);
3. real `/livox/lidar` + `/livox/imu`: rate, layout, frame, **stamp
   monotonicity and clock domain** (§1.2, §1.3);
4. the **physical extrinsic** against H-1 (§1.6);
5. **DLIO stationary behaviour**: initialisation, drift, TF rate (§1.4, §1.5);
6. **stamped TF availability** at real cloud stamps (§1.7, §1.9);
7. real **self-hit geometry** with CropBox bypassed (§1.8);
8. real **floor behaviour** through the height band (§1.9);
9. detector/tracker error against **surveyed** fixtures (§1.10);
10. **CPU and latency on the machine that will actually run it** (§2, §3.4 of
    the preflight).
