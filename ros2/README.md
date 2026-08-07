# DPCBF Perception — ROS2 workspace

Colcon workspace for the perception subsystem.

- **실험 당일 — 컴퓨터 3대 구성과 운용 순서:** **[`doc/README.md`](doc/README.md)**
  (Blackbox / onboard / 노트북이 각각 무엇을 하고, 어디에 무엇을 치고,
  무엇이 정상이고, 안 되면 무엇부터 보는지. 당일에는 이 파일 하나로 시작).

- **처음부터 전부 세팅 — 컴퓨터 2대(Mid-360 직결), clone부터 실시간
  시각화까지:** **[`doc/g1_two_computer_setup.md`](doc/g1_two_computer_setup.md)**
  (Foxy 온보드 PC의 apt/빌드, LiDAR IP 탐색, `MID360_config.json` 작성,
  Humble 노트북 빌드, CycloneDDS 연결, 터미널별 복붙 시트. Foxy에서 실제로
  다르게 동작하는 CLI 차이도 실측해 정리).

- **Architecture, contracts, phase gates, the progress log:**
  `../DPCBF_Perception_Subsystem_ROS2_Architecture.md` (the single source of truth).
- **How to run any of this** — bring the stack up, see it in RViz, walk the
  robot, reproduce a §21 number, work out why nothing is arriving:
  **[`doc/operator_runbook.md`](doc/operator_runbook.md)**.
- **The robot session:** [`doc/phase5b_checklists.md`](doc/phase5b_checklists.md)
  (block-structured capture plan) and, for a first-time G1 operator, the
  Phase-5C trio:
  [`doc/g1_hardware_preflight.md`](doc/g1_hardware_preflight.md) — what to
  find out before the robot is powered;
  [`doc/g1_hardware_code_audit.md`](doc/g1_hardware_code_audit.md) — what the
  hardware path contains and what of it is actually verified;
  [`doc/g1_first_perception_experiment.md`](doc/g1_first_perception_experiment.md)
  — the staged, perception-only session (no actuation).

This file is the workspace's **provenance**: what it is made of, why each
external is pinned or patched, and how to build it. Procedures live in the
runbook and are not repeated here — including the environment variables, the
runtime traps, and the gate-by-gate commands, all of which used to be in this
file and moved wholesale on 2026-08-02 so that there is exactly one copy of
each.

## Build

```bash
cd ros2
./setup_external.sh                  # vcs import (pinned SHAs) + patches 0001–0011
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test  --merge-install --packages-select \
    dpcbf_ros_adapter safety_obstacle_filter g1_perception_utils \
    g1_description g1_perception_bringup sim_mjlidar_bridge
```

`--merge-install` is required **on both commands**: `unitree_sdk2` and the ROS
packages must share one prefix so simulate/deploy find both through a single
`CMAKE_PREFIX_PATH` entry and exactly one `libddsc.so.0` exists at runtime
(R-3) — and colcon refuses to operate on a merged layout without the flag, so
plain `colcon test` errors out and the follow-up `colcon test-result` then
cheerfully prints the *previous* run's summary. See the runbook §5.1 for that
trap and for why the test query must be scoped to these six packages.

The simulator and the deploy FSM are plain CMake, not colcon; their build
commands are in the runbook §2.4.

### Foxy (the G1's onboard computer)

The robot's onboard PC runs **ROS 2 Foxy on Ubuntu 20.04**, while this
workspace is developed on Humble/22.04. Foxy cannot be installed natively on
jammy, so the Foxy build is proven in a focal container and the sources are
kept buildable on **both** distros — never ported one way.

```bash
./tools/foxy_docker.sh build-image   # focal + Foxy + this workspace's apt deps
./tools/foxy_docker.sh build         # the ten packages this repo owns
./tools/foxy_docker.sh build all     # the whole workspace, src/external included
./tools/foxy_docker.sh test
./tools/foxy_docker.sh shell         # or poke around by hand
```

The repo is bind-mounted at its host path and the Foxy artefacts go to
`build_foxy/ install_foxy/ log_foxy/`, so the Humble `build/ install/` next to
them is untouched and both can coexist.

Foxy is EOL: `packages.ros.org` dropped it, but the frozen archive at
`snapshots.ros.org/foxy/final` still serves every deb this workspace needs and
the stock `ros:foxy-ros-base` image already points there. Do not "fix" those
apt sources.

What the distro difference actually costs, all of it plumbing rather than
behaviour (patches 0011–0015 plus five in-repo edits): `rosidl_get_typesupport_target`
is Galactic+; `tf2_geometry_msgs` / `tf2_sensor_msgs` are `.h` on Foxy, and
`tf2_geometry_msgs` there has no unstamped-`Pose` `doTransform`;
`declare_parameter(name, ParameterType)` and `resolve_topic_name()` are Humble+;
subscription callbacks taking `const MessageT&` are Galactic+ (Foxy binds only
the shared_ptr forms, and cannot deduce through `const ConstSharedPtr&` either);
`PointCloud::Ptr` is `boost::shared_ptr` on focal's PCL 1.10; libstdc++ 9 does
not pull in `<cstring>` transitively; Foxy vendors googletest 1.8, where a
`SetUpTestSuite()` hook is silently never called; and `ast.unparse` is 3.9+.

### CycloneDDS on Foxy, and R-3

`rmw_cyclonedds_cpp` is the one package **not** built from source on Foxy — the
image installs the `ros-foxy-rmw-cyclonedds-cpp` 0.7.11 deb instead. No source
pair exists: the humble branch needs rmw headers Foxy does not ship
(`rmw/features.h`, `rmw/get_network_flow_endpoints.h`), and the foxy branch is
written against CycloneDDS **0.7**'s `ddsi_sertopic`, which the 0.10.2 this
workspace pins for `unitree_sdk2` replaced with `ddsi_sertype`.

R-3 (one `libddsc` per process) still holds, and it was worth checking rather
than assuming. At runtime the 0.7-built deb rmw loads `unitree_sdk2`'s
**0.10.2** `libddsc.so.0` — 0.10.2 kept the old ABI — and a CycloneDDS pub/sub
round trip delivers normally under it. The T10 coexistence gate passes on Foxy
(491 messages on each stack, one `libddsc` mapped). The break is
**compile-time only**: the deb drags `ros-foxy-cyclonedds` 0.7 headers into
`/opt/ros/foxy/include`, and `ddscxx`'s topic templates need the 0.10 ones, so
`t10_dds_coexistence` puts `unitree_sdk2`'s own copy first with
`target_include_directories(... BEFORE ...)`. Anything else that links both
stacks — the DPCBF hardware seam when it lands — needs the same treatment.

**Scope.** Verified on Foxy: all 19 colcon packages build, the C++ unit tests
pass, the 283 `hw_offline_gates` checks pass (including all five hardware
launch files constructing under Foxy's launch), and T10 passes at runtime.

**Foxy↔Humble DDS interop is now measured too** (2026-08-07): the focal
container on `--network host` against the host's Humble workspace — i.e. the
deb rmw loading the workspace's CycloneDDS 0.10.2 on one side and the
source-built rmw with the same 0.10.2 on the other — carries all four topics
Computer 3 subscribes to, **both directions, zero loss**: `/odom` 1500/1500 at
99.99 Hz, `/obstacles_safe` at 10.00, `/dpcbf/plot` at 30.00, `std_msgs/String`
at 2.00, under multicast AND static-peer discovery. Payloads are checked
field by field, including `CircleObstacle.covariance` (patch 0007's added
`float64[3]`) and `DpcbfPlotSample`'s nested `PlotObstacle[]`, and the real
`dpcbf_plot_client.data_hub` renders it live. That proves the distro-crossing
layer; the physical link between two machines (Wi-Fi multicast, firewall,
bandwidth) is still a field variable. Details and the reproduction recipe:
[`doc/g1_two_computer_setup.md`](doc/g1_two_computer_setup.md) 부록 1.

Not verified: anything requiring the real sensor.

## What is built from source here, and why

`sudo` needed a password when this workspace was assembled, so packages the
architecture assumed as apt binaries are **built from pinned source** instead:
`cyclonedds` 0.10.2, `cyclonedds-cxx` 0.10.2, `rmw_cyclonedds_cpp` (humble),
`pointcloud_to_laserscan` (humble), and `pcl_ros` 2.6.1 — the Humble **binary**
`pcl_ros` 2.4.5 ships *no* filter components at all, so the CropBox stage is
impossible without that pin. The Unitree side (`unitree_sdk2`, the C++
`unitree_dds_wrapper` headers) was never installed on this machine either
(`/opt/unitree_robotics` does not exist) and is pinned and built here too. Side
effect: the whole stack shares ONE CycloneDDS — the R-3 mitigation by
construction.

Phase 5A added the hardware stack to the same treatment: `Livox-SDK2` (v1.3.1)
is plain CMake with no `package.xml`, so it is COLCON_IGNOREd and driven into
the merged prefix by the `livox_sdk2_vendor` package (upstream installs to
`/usr/local`); `livox_ros_driver2` (1.2.6) and DLIO (`feature/ros2`) are pinned
and patched.

`ros-humble-rosbag2-storage-mcap` and `ros-humble-foxglove-bridge` **are now
installed** (the long-standing operator ask, closed in interim block 2).
`record.launch.py` still defaults to sqlite3; switching it is an ordinary
follow-up.

## Recorded patches

Applied by `setup_external.sh`, each `git apply --check`ed first so the script
is idempotent. PR-ready, minimal versions of the upstreamable ones are split out
under `patches/pr/` (`PR_READY.md`); nothing has been pushed.

| # | Target | What and why |
|---|---|---|
| 0001 | `unitree_dds_wrapper` | restores the simulator-facing joystick API the pinned SHA lacks (the original build machine ran a modified copy that was never pushed) |
| 0002 | `pcl_ros` | filter output publisher → SensorData QoS (§7.1). **Obsolete against the upstream tip**, which added `QosOverridingOptions`; delete it and set the override when the pin moves past `e264aff1` |
| 0003 | `obstacle_detector_2` | P-1: componentization, `/scan` subscription SensorData (upstream's Reliable never matches a best-effort laser publisher), publishers at §7.1 depth 5, TF lookup at the scan stamp, and an upstream grouping bug (`begin()++` double-counted the first point of every first group, corrupting its circle fit) |
| 0004 | `obstacle_detector_2` | P-2: measurement-driven tracker — predict+correct on arrival with dt from header stamps, no wall timer, measurement-stamped output, two-point initiation with matching init covariance, radius-residual weight 0.3 |
| 0005 | `livox_ros_driver2` | `package.xml` + `colcon.pkg` (upstream's `build.sh` would `rm -rf` this workspace's build/install trees) and the `/livox/imu` `frame_id` fix |
| 0006 | DLIO | cloud subscription SensorData QoS, `/odom` depth 10 |
| 0007 | `obstacle_detector_2` | P-3: `CircleObstacle.covariance` — D6's one sanctioned message change. **Invalidates every fixture bag carrying `Obstacles`** |
| 0008 | `livox_ros_driver2` | declares its build-order dependency on `livox_sdk2_vendor`; only a sequential build exposes the omission |
| 0009 | `obstacle_detector_2` | P-4: the `max_circle_radius` gate moved onto the fit (it used to test `fit + radius_enlargement`, so making the safety inflation larger made the sensor blinder), plus drop counters and throttled warnings |
| 0010 | `obstacle_detector_2` | P-5: retires the tracker's inert `loop_rate`; the name stays declared so a config that still sets it gets an explicit warning instead of silence |
| 0011 | `obstacle_detector_2` | F-1: builds on Foxy as well as Humble — `rosidl_target_interfaces` vs `rosidl_get_typesupport_target`, the `tf2_geometry_msgs` `.h`/`.hpp` split and its missing unstamped-`Pose` `doTransform`, `declare_parameter`-by-type, and the `const MessageT&` subscription callbacks Foxy's rclcpp cannot bind. **No computation changes** |
| 0012 | `pointcloud_to_laserscan` | F-1: `tf2_sensor_msgs` `.h`/`.hpp` |
| 0013 | `pcl_ros` | F-1: `tf2_geometry_msgs` `.h`/`.hpp`; `pcd_to_pointcloud`'s two rclcpp-16 APIs (`declare_parameter<T>(name)`, `resolve_topic_name`); `project_inliers`' declare-by-type |
| 0014 | `livox_ros_driver2` | F-1: chooses the typesupport idiom with `if(COMMAND ...)` instead of `DISTRO_ROS STREQUAL "humble"`, which `colcon.pkg` pins to one value while the tree builds on two distros |
| 0015 | DLIO | F-1: typesupport idiom, the `const ConstSharedPtr&` callback, and **PCL 1.10** (focal) using `boost::shared_ptr` for `PointCloud::Ptr` where upstream assumed `std::shared_ptr` — 21 sites moved to `pcl::make_shared` |

## Building somewhere that is not this machine (`tools/`)

The build command above is the *dev machine's*. It works here and nowhere else
without help — five separate environment faults appeared the first time the
workspace was built for another architecture, none of them in this code.

| tool | for |
|---|---|
| `tools/build_target.sh` | the on-target build (Orin, Pi, CI, container). Encodes all five fixes with the error each prevents, and ends with a triage list keyed by error message. `--check` does the preflight only |
| `tools/diagnose_ament_export_libraries.py` | recognises an upstream `ament_cmake` bug that makes a package claim it cannot find **its own** library. Package-set dependent, so it hits fresh machines and never showed up here. **Diagnostic only** — no workaround has been shown correct |
| `tools/arm64_emulate.sh` | an aarch64 rootfs under qemu-user **without root** — no cross toolchain, no `qemu-user-static` package, no binfmt registration, no sudo |

Short version of the five, so a failure elsewhere is recognised rather than
explored: VTK imported targets that exist only for `RelWithDebInfo`;
`find_package(OpenSSL)` failing inside the ROS config chain while a bare
`find_library(crypto)` succeeds in the same shell; PCL headers including Eigen
unqualified; `livox_ros_driver2`'s undeclared dependency (patch 0008); and the
ament `_lib` cache-shadow bug, which is characterised but **unsolved** — 10 of
18 packages build on emulated aarch64 and that bug is what stops the rest.

## Layout

```
ros2/
├── deps.repos            pinned SHAs — never floating
├── setup_external.sh     import + patch
├── patches/              recorded patches; patches/pr/ = upstream-ready splits
├── tools/                on-target build + diagnostics (see above)
├── doc/                  operator_runbook.md, phase5b_checklists.md, phase5a_seam_audit.md
├── evidence/             per-phase measured artefacts referenced from §21
├── test_fixtures/        bag fixtures (gitignored) + their regeneration recipes
└── src/g1_perception/    the packages; src/external/ is the vcs import target
```
