# DPCBF Perception — ROS2 workspace

Colcon workspace for the perception subsystem. Architecture, contracts and
phase gates: `../DPCBF_Perception_Subsystem_ROS2_Architecture.md` (the single
source of truth).

## Environment hygiene (read before building — R-11)

This machine's shell auto-activates a **conda env with Python 3.12**, which
shadows ROS2 Humble's system Python 3.10 and silently breaks rclpy, colcon
and every ament_python package. Every build/run command below must see system
python first:

```bash
export PATH=/usr/bin:$PATH          # puts /usr/bin/python3 (3.10) first
hash -r
source /opt/ros/humble/setup.bash
source install/setup.bash            # after the first build
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # D2 — mandatory
export ROS_DOMAIN_ID=0                          # matches simulate/config.yaml & SDK2
```

Launch files pin the sidecar to system python; nothing in this workspace may
ever run under the conda interpreter.

## No-sudo reality of this machine (2026-08-01)

`sudo` needs a password and apt is unusable non-interactively, so packages the
architecture assumed as apt binaries are **built from pinned source** in this
workspace instead: `cyclonedds` 0.10.2, `cyclonedds-cxx` 0.10.2,
`rmw_cyclonedds_cpp` (humble), `pointcloud_to_laserscan` (humble), and
`pcl_ros` 2.6.1 (Phase 2 — the Humble BINARY pcl_ros 2.4.5 ships **no**
filter components at all, so the CropBox stage is impossible without this
pin; patch 0002 makes its output publisher SensorData per §7.1). The
`obstacle_detector_2` fork carries two recorded patches (Phase 3):
0003 (P-1: composable-node components, SensorData `/scan` subscription —
upstream's Reliable one never matches a best-effort laser publisher —
publishers at §7.1 depth 5, TF lookup at the scan stamp, and an upstream
grouping bug fix: `begin()++` double-counted the first scan point of every
first group, corrupting its circle fit) and 0004 (P-2: measurement-driven
tracker — predict+correct on arrival with dt from header stamps, no wall
timer, measurement-stamped output, two-point track initiation with matching
init covariance, radius-residual weight 0.3). The
Unitree side (`unitree_sdk2`, C++ `unitree_dds_wrapper` headers) was **never
installed on this machine** (the doc's `/opt/unitree_robotics` does not
exist); both are pinned in `deps.repos` and built here too. Side effect: the
whole stack shares ONE CycloneDDS — the R-3 mitigation by construction.

Still missing until someone runs apt with sudo:
`ros-humble-rosbag2-storage-mcap` (bags record as sqlite3 until then),
`ros-humble-foxglove-bridge`.

## Build

```bash
cd ros2
vcs import src < deps.repos          # pinned SHAs only — never floating
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`--merge-install` is required: unitree_sdk2 and the ROS packages must share
one prefix so simulate/deploy find both through a single `CMAKE_PREFIX_PATH`
entry, and so exactly one `libddsc.so.0` exists at runtime (R-3).

`colcon test && colcon test-result --verbose` runs the unit tests + T7.

## simulate with the ROS2 module (Phase 1)

```bash
cd ../simulate && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=ON \
      -DCMAKE_PREFIX_PATH=$PWD/../../ros2/install
make -j$(nproc)
```

With the option OFF (default) simulate builds and behaves exactly as at
baseline `f111cfa`. With ON it publishes `/clock`, `/sim/mj_state`,
`/sim/gt_obstacles` and dumps the compiled mirror model to
`/tmp/unitree_mujoco_mirror_model.xml` (override: env
`UNITREE_MUJOCO_MIRROR_XML`) — the sidecar loads THAT file, never the raw
scene, because the obstacle mocap bodies are added to the spec at runtime.

## Run (sim)

```bash
# terminal 1 — simulator (needs a scene config; see repo README)
./simulate/build/unitree_mujoco

# terminal 2 — perception bringup
ros2 launch g1_perception_bringup bringup.launch.py source:=sim viz:=rviz
```

## Runtime notes (hard-won, do not rediscover)

- **Init order is load-bearing (T10/R-3):** in any process that links both
  stacks, rclcpp must initialize BEFORE `ChannelFactory::Init`, and the
  factory must join with an EMPTY interface (`Init(domain, "")`). Both
  rmw_cyclonedds and SDK2's ddscxx hard-fail when asked to create a domain
  that already exists. The interface is pinned for both stacks via one
  `CYCLONEDDS_URI`; the simulate ROS2 module derives it from config.yaml
  (including `MaxAutoParticipantIndex=120` — cyclone's default ~10 exhausts
  fast) when the env var is unset.
- **Broken NVIDIA driver:** run simulate (and rviz2) with
  `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa`.
- **No joystick on this machine** and `use_joystick: 1` has no CLI override:
  for testing, run simulate from a shadow directory with a
  `use_joystick: 0` copy of config.yaml (never edit the tracked file).
- The sidecar mirrors `/tmp/unitree_mujoco_mirror_model.xml` (override:
  `UNITREE_MUJOCO_MIRROR_XML`), dumped by simulate at model load — never the
  raw scene, which lacks the runtime-added obstacle mocap bodies.
- **Live-path processes must export the lo-pinned `CYCLONEDDS_URI`** (Phase 2):
  simulate derives it internally from config.yaml, so its topics live on `lo`
  — any launch/CLI/probe started WITHOUT the same URI binds the default NIC
  and sees the topic names via discovery but **no data**. Use:
  `CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'`
  (Bag-replay-only sessions work without it because every endpoint then
  shares the default interface.)
- **SIGTERM on `ros2 launch` orphans composable containers** (Phase 2): the
  stale `/perception_container` twin then races the next launch's LoadNode
  RPC and the new container stays empty. Stop launches with SIGINT and/or
  `pkill -f rclcpp_components/component_container` before relaunching in
  scripts.

## Gate tests (§16.2)

| Gate | Where |
|---|---|
| T10 DDS coexistence | `install/t10_dds_coexistence/lib/t10_dds_coexistence/t10_smoke` |
| T7 extrinsic guard | `colcon test --packages-select g1_description` (CTest `t7_extrinsic_guard`) |
| T2 wall occlusion | `src/g1_perception/sim_mjlidar_bridge/test_gates/t2_wall_occlusion.py` |
| T3 pattern envelope | `src/g1_perception/sim_mjlidar_bridge/test_gates/t3_pattern_envelope.py` |
| Phase-2 measured wall (±2 cm) + T9 | CTest `test_wall_accuracy.launch_test.py` (self-contained; fixture: `test_fixtures/wall_scene/`) |
| Phase-2 replay integration + T9 | CTest `test_projection_replay.launch_test.py` (needs the gitignored fixture bag) |
| T4 static accuracy + extractor integration | CTest `test_detection_static.launch_test.py` (fixture: `test_fixtures/s1_surveyed`) |
| T5 dynamic tracking (0.5 / 0.8 m/s) | CTest `test_tracking_dynamic_{05,08}.launch_test.py` (fixtures: `test_fixtures/s2_cross_*`) |
| T8 replay determinism (HARD, P-2 landed) | CTest `t8_replay_determinism` (script `test/test_t8_replay_determinism.py`; fixture: `s1_surveyed`) |
