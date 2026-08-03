# DPCBF Perception — ROS2 workspace

Colcon workspace for the perception subsystem.

- **Architecture, contracts, phase gates, the progress log:**
  `../DPCBF_Perception_Subsystem_ROS2_Architecture.md` (the single source of truth).
- **How to run any of this** — bring the stack up, see it in RViz, walk the
  robot, reproduce a §21 number, work out why nothing is arriving:
  **[`doc/operator_runbook.md`](doc/operator_runbook.md)**.
- **The robot session:** [`doc/phase5b_checklists.md`](doc/phase5b_checklists.md).

This file is the workspace's **provenance**: what it is made of, why each
external is pinned or patched, and how to build it. Procedures live in the
runbook and are not repeated here — including the environment variables, the
runtime traps, and the gate-by-gate commands, all of which used to be in this
file and moved wholesale on 2026-08-02 so that there is exactly one copy of
each.

## Build

```bash
cd ros2
./setup_external.sh                  # vcs import (pinned SHAs) + patches 0001–0009
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
