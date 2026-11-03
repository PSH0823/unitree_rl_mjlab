# Phase 5A — sim/hardware seam audit

*What the simulated source satisfied for free that hardware does not.*

Scope: every §7.1 row, walked against the **pinned checkouts** of
`livox_ros_driver2` (tag 1.2.6 @ `13eb05e4`) and DLIO (`feature/ros2` @
`c8acc371`), not against their READMEs or against §5.6/§5.7. Findings marked
**[fixed]** are already handled in this workspace (patch, config or launch);
**[recorded]** means accepted and documented; **[5B]** means it needs the
robot to settle.

`source_hw.launch.py` and `config/{livox_driver.yaml,MID360_config.json,dlio.yaml}`
are where all of this lands. `perception.launch.py` is unchanged and still has
no sim/hw conditional (D4). The one sim/hw branch outside `source_*` is the
`use_sim_time` default in `bringup.launch.py`, which now follows `source`.

---

## §7.1 row by row

### `/livox/lidar` — PointCloud2, 10 Hz, `mid360_link`, SensorData

| | sim (`sim_mjlidar_bridge`) | hardware (`livox_ros_driver2`) |
|---|---|---|
| fields | `x,y,z` FLOAT32 | `x,y,z,intensity` FLOAT32, `tag,line` UINT8, `timestamp` FLOAT64 |
| `point_step` | 12 | **26** (`#pragma pack(1)`; the FLOAT64 sits at offset 18, unaligned) |
| publisher QoS | SensorData (best-effort, depth 5) | **RELIABLE, depth 256**, not parameterised |
| `frame_id` | param, default `mid360_link` | param, ROS2 default **`frame_default`** (its own example launch files say `livox_frame`) |
| stamp | sim time, from `/sim/mj_state` | LiDAR time if PTP/GPS-synced, else **host clock at packet reception** |
| topic existence | always | **created lazily on first data** |

- **[fixed]** `frame_id: mid360_link` is set in `config/livox_driver.yaml`. Left
  at the default, every TF lookup in the stack fails on a frame nobody
  publishes.
- **[fixed/verified]** The 26-byte packed layout goes through the whole chain
  unmodified — `test_hw_source_contract.launch_test.py` feeds the exact
  driver record and gets 277 `/obstacles_safe` out of 278 clouds. §7.2's "only
  x,y,z, tolerate extra fields" holds in practice, not just on paper.
- **[recorded]** The Reliable publisher is a §7.1 deviation with no knob. It is
  the *compatible* direction (Reliable pub ↔ best-effort sub matches), and the
  QoS wire-check in the same test asserts every `/livox/lidar` subscriber is
  best-effort so the single contract keeps working in both worlds. Not patched:
  changing a driver's publisher QoS is a bigger upstream argument than it is
  worth while the match is legal.
- **[recorded]** `xfer_format`: **0 is the only usable value in ROS2.**
  1 is `CustomMsg` (FAST-LIO's format, *instead of* PointCloud2), and 2
  (`kPclPxyziMsg`) is ROS1-only — its `CreatePublisher` branch is `#if 0`'d out
  and `PublishPclMsg()` prints "not supported in ROS2" and returns. §5.6/§7.2's
  "0 or 2 for perception" is wrong for this build. See the §5.6 corrections
  below.
- **[5B]** Point density. Sim measured 10 700–20 200 valid points/frame
  (Phase 1). The real Mid360's ~200 kpts/s at 10 Hz is nominally the same
  order, but the rosette *distribution* is Q-3's question — the Phase-2
  bin-occupancy table is the comparison baseline.

### `/livox/imu` — Imu, 200 Hz, `mid360_link`, SensorData

- **[recorded]** The sim sidecar **never published this topic at all** (§7.1
  marks it "sidecar (optional)"). Nothing in Phases 1–4 needed it — and DLIO
  does not start without it. That is the single largest "free in sim" gap:
  there is no sim counterpart to exercise, which is why `hw_source_stub.py`
  synthesises a stationary IMU for the bench tests.
- **[fixed]** `InitImuMsg` hardcoded `frame_id = "livox_frame"`, ignoring the
  `frame_id` parameter that the cloud path honours — `/livox/imu` came out in a
  frame that exists nowhere in the TF tree. Patch 0005; upstreamable.
- **[5B]** Actual IMU rate and whether its stamps share the cloud's time base
  are device facts. Measure in block 1.

### `/odom` — Odometry, ≥50 Hz, Reliable depth 10

| | sim | hardware (DLIO) |
|---|---|---|
| source | ground truth (MuJoCo pelvis) | GICP + IMU propagation |
| rate | 99.2 Hz | 100 Hz (`publishPose` wall timer) — **measured 100.0 Hz** |
| depth | 10 | upstream **1** |
| stamp | sim time | `imu_stamp`, i.e. **0 until the first IMU message arrives** |
| topic name | `/odom` | `odom` relative — upstream's launch remaps it to `dlio/odom_node/odom` |

- **[fixed]** Patch 0006 raises the publisher to depth 10 (§7.1).
- **[fixed]** `source_hw.launch.py` deliberately does *not* copy upstream's
  remap, so the §7.1 name is what appears.
- **[recorded]** Anything computing a rate from `/odom` header stamps over a
  window that includes startup gets nonsense, because the first stamps are 0.
  The wiring test measures arrival rate instead.

### TF `odom→base_link` — "with `/odom`"

This row hides the biggest behavioural difference in the phase.

- **[recorded, measured] DLIO publishes this TF at SCAN rate (~10 Hz), not at
  `/odom`'s 100 Hz.** `publishPose()` (100 Hz wall timer, `odom.cc:328-369`)
  emits only `/odom` and `/pose`; all three TF broadcasts live in
  `publishToROS()` (`odom.cc:371-444`), which is spawned as a thread once per
  scan (`odom.cc:852`). The sim sidecar published TF at 100 Hz in the same
  callback as `/odom`, always at or before the cloud stamp.
  Measured consequence on this machine, full chain, DLIO in the loop:

  | | sim (Phase 3/4) | hardware path (this rig) |
  |---|---|---|
  | cloud → `/scan` | p50 0.48 / p95 0.95 ms | **p50 9.24 / p95 12.81 ms** |
  | cloud → `/obstacles_safe` | p50 1.28 / p95 1.76 ms | **p50 9.61 / p95 13.19 ms** |

  The tf2 `MessageFilter` in `pointcloud_to_laserscan` waits for the TF sample
  that brackets the cloud stamp; that wait is the ~9 ms. Still far inside the
  §17.2 ≤60 ms budget with no frame loss in steady state (249 clouds → 249
  scans → 249 `/obstacles_safe` over 25 s), **so the obvious fix is not
  taken**. It is recorded as conditional patch **P-5** — broadcast
  `odom→base_link` from `publishPose()` as well, using the IMU-propagated state
  it already publishes — to fire only if T9-hardware or the Orin latency
  benchmark shows the wait growing. `test_dlio_wiring.launch_test.py` asserts
  the split so it cannot change silently.
- **[recorded]** Startup transient: 31 clouds were dropped by the message
  filter with *"the timestamp on the message is earlier than all the data in
  the transform cache"* before DLIO's first TF. Harmless, but it is what the
  first seconds of every hardware bag will look like — do not read it as a
  fault on robot day.
- **[fixed] DLIO has no TF listener at all.** `extrinsics/baselink2lidar` and
  `.../baselink2imu` in its config are the *only* thing that defines where
  `base_link` sits relative to the sensor. Left at upstream's identity,
  `odom→base_link` would really be `odom→mid360_link`: off by 0.472 m in z and
  rotated by roll = π. In sim, `base_link` ≡ MuJoCo pelvis came for free.
  `config/dlio.yaml` derives both from `g1_mid360.xacro` (§8.3), and
  `t7_hw_extrinsic_guard.py` recomputes them from the xacro on every
  `colcon test` so the copy cannot drift.
- **[fixed]** DLIO also broadcasts `base_link→frames/lidar` and
  `base_link→frames/imu` unconditionally. Naming either of them `mid360_link`
  would give that frame **two parents** alongside `robot_state_publisher`'s
  `torso_link→mid360_link`. They are named `dlio_lidar_link` / `dlio_imu_link`;
  the guard rejects any name from the §8.2 tree, and the wiring test checks the
  resulting tree shape.

### TF static (robot) — `robot_state_publisher`

Identical in both worlds; no change. §8.3's fixed `base_link→torso_link`
approximation (R-8) is unchanged and remains a hardware-only error source: in
sim the waist genuinely did not move relative to the published TF, on hardware
it will.

### TF `odom→base_footprint` — 100 Hz

- **[recorded]** `base_footprint_publisher` republishes only when the
  `odom→base_link` stamp changes, so on hardware it **inherits DLIO's ~10 Hz**.
  The "100 Hz" in §7.1 describes the sim sidecar. Nothing downstream needs more
  — `pointcloud_to_laserscan` is stamp-driven — but the row is misleading.
- **[recorded, test-rig lesson]** Feeding it a *static* `odom→base_link` (e.g.
  `static_transform_publisher` as a stand-in) makes it emit `base_footprint`
  exactly once, at time 0, after which every lookup extrapolates into the
  future and the whole chain silently produces nothing. Real odometry — sidecar
  or DLIO — is dynamic. `hw_source_stub.py` grew an `odom_hz` mode for this.

### `/points_self_filtered`, `/scan`, `/raw_obstacles`, `/tracked_obstacles`, `/obstacles_safe`

Unchanged nodes, unchanged params, verified end-to-end against the hardware
cloud layout. Two carried items are hardware-sensitive:

- **[5B]** The CropBox box is still the Phase-3 **sim-interim** (xy ±0.40,
  `max_z` 0.45), derived from wrist returns in a *grounded sim pose*. Block 4
  of the 5B checklist captures real self-hit data with CropBox bypassed; the
  retune happens offline afterwards.
- **[5B]** The extractor's chord/√3 radius heuristic showed +16…+23 mm bias on
  r=0.15 cylinders and, per Phase 4, a bias that **grows with radius**
  (+33 mm radius, 83 mm centre at r=0.30). Block 5 re-measures on real props,
  which is why the ≥0.30 m prop is mandatory.

### `/sim/mj_state`, `/sim/gt_obstacles`, `/clock` — sim-only

- **[fixed]** No `/clock` exists on hardware. `bringup.launch.py` now derives
  the `use_sim_time` default from `source` (`true` for sim, `false` for hw).
  This is the only sim/hw conditional outside the `source_*` files, and it is
  in the switchboard, not in the shared stack.
- **[fixed]** DLIO's upstream `cfg/params.yaml` ships `use_sim_time: true`.
  Our `config/dlio.yaml` sets it false and both the guard and the wiring test
  assert it — a node left true on hardware never advances its clock and its
  100 Hz publish timer never fires.
- **[recorded]** There is **no ground truth on hardware**. Every Phase 1–4
  accuracy number was computed against `/sim/gt_obstacles`. 5B substitutes
  surveyed static layouts (Q-6), which is why the survey plan is part of the
  checklist rather than an afterthought.

### `/dpcbf/status` — adapter

Out of scope: no DPCBF in the loop on hardware this phase (Phase 6).

---

## Corrections to the architecture document

Verified against the pinned checkouts; these are §5.6/§5.7/§7.1/§14.3
statements that do not survive contact with the source.

1. **§5.6 / §7.2 — `xfer_format` 2 does not exist in ROS2.** Only 0 produces a
   PointCloud2. "0 or 2 for perception" → **0**.
2. **§5.6 — the driver cannot dual-publish.** `transfer_format_` selects a
   single publisher for a single topic; there is no simultaneous
   PointCloud2 + CustomMsg mode. "The driver can dual-publish CustomMsg for
   FAST-LIO without affecting the perception contract" is false. **The Q-4
   bake-off is therefore a separate capture session with the driver
   reconfigured, not a co-run** — and while it runs, the perception stack has
   no cloud. Plan session time accordingly.
3. **§7.1 — `/livox/imu` frame.** Upstream hardcodes `livox_frame`; patch 0005
   makes it follow `frame_id`.
4. **§7.1 — TF rates.** `odom→base_link` and `odom→base_footprint` are
   scan-rate (~10 Hz) on hardware, not 100 Hz. The table's rates are the sim
   sidecar's.
5. **§14.3 — timestamp mode is not a driver configuration.** There is no
   PTP field in `MID360_config.json`. The driver reads the sync mode out of
   each packet header (`time_type`) and, when the LiDAR reports
   `kTimestampTypeNoSync`, stamps with the **host clock at packet reception**
   (`pub_handler.cpp::GetEthPacketTimestamp`). PTP is enabled on the *LiDAR*,
   out of band. "PTP if the G1 network supports it, else driver host-time mode"
   is right in outcome, wrong in mechanism — and which one is live is only
   observable at runtime (block 1 measures it).
6. **§12.1 — `livox_ros_driver2` is componentisable.** It registers
   `livox_ros::DriverNode` via `rclcpp_components`. §14.2 keeps it standalone;
   worth knowing if the Orin CPU budget gets tight.
7. **§13 — upstream's `build.sh` cannot be used.** It `rm -rf`s
   `../../build`, `../../devel` and `../../install`, i.e. this workspace.
   Patch 0005 supplies `package.xml` and a `colcon.pkg` with
   `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble` instead (without `DISTRO_ROS` the
   humble typesupport branch is skipped and the link fails).

## Operational facts worth knowing before robot day

- **The driver does not exit when it cannot bind.** With unreachable host IPs
  it logs `bind failed` / `Init lds lidar fail!` and then *keeps running*: the
  node appears in `ros2 node list`, no `/livox/*` topic is ever created (the
  publisher is lazy), and it ignores **SIGINT and SIGTERM** — `ros2 launch`
  escalates to SIGKILL after 15 s. Diagnostic rule for block 1: *no topic at
  all* = SDK/bind failure; *topic present but 0 Hz* = something else. Kill it
  with `pkill -9`.
- `test/hw_config_check.py` catches the bind case before launch: it verifies
  every `host_net_info` IP is actually assigned to a local interface, that the
  ports are free, that the LiDAR shares a /24 with the host, that
  `xfer_format` is 0 and `frame_id` is `mid360_link`, and that
  `extrinsic_parameter` is identity (H-1 belongs in the xacro only — setting it
  in the JSON as well applies the extrinsic twice). It exits 2 while the
  Q-1 placeholder IPs are still in place, which is the expected dev-machine
  state.
- `pkill -f component_container` **kills the shell that runs it**, because the
  shell's own command line contains the pattern. Use a bracketed class:
  `pkill -9 -f 'component_containe[r]'`. (Phase 4 recorded a related pkill trap
  for `unitree_mujoc[o]`; this is the same family and cost one debugging cycle
  here.)
