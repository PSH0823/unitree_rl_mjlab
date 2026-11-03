# PR-ready upstream artifacts — NOT PUSHED

Pushing to public forks and opening PRs under this project's name is an
outward-facing action and has not been authorised. This directory makes the
authorisation a *push* rather than a writing task: each artifact below is a
minimal, single-purpose patch, verified to apply to the upstream tip named
next to it, with the measurement that justifies it and a reproduction.

`../000N-*.patch` (the series `setup_external.sh` applies) stays the source of
truth for this workspace and is deliberately *not* what goes upstream — those
patches bundle project-specific decisions with genuinely general fixes. The
files here are the general parts, split out.

Verified 2026-08-02 by `git ls-remote` + `git apply --check` against a fresh
clone of each tip.

| # | Repo / branch | Upstream tip | Applies | Kind |
|---|---|---|---|---|
| 01 | harmony-eu/obstacle_detector_2 `humble-devel` | `6ff7ac48` (= our pin) | yes | bug fix |
| 02 | harmony-eu/obstacle_detector_2 `humble-devel` | `6ff7ac48` | yes | bug fix |
| 03 | Livox-SDK/livox_ros_driver2 `master` | `13eb05e4` (= our pin) | yes | build bug |
| 04 | vectr-ucla/direct_lidar_inertial_odometry `feature/ros2` | `c8acc371` (= our pin) | yes | interop |
| 05 | Livox-SDK/livox_ros_driver2 `master` | `13eb05e4` | yes | bug fix |
| 06 | harmony-eu/obstacle_detector_2 `humble-devel` | `6ff7ac48` | yes | bug fix |

---

## 01 — `obstacle_extractor`: the first scan point is counted twice

**Commit message**

> Fix first-point double-count in groupPoints()
>
> `for (PointIterator point = input_points_.begin()++; ...)` post-increments a
> temporary, so the loop starts at `begin()` and the point already seeded into
> `point_set` is processed a second time. The first group's `num_points` is
> therefore one too large. `fitSegment()` trusts that count, so it reads one
> point past the group; when the extra point belongs to a different object the
> whole line fit — and the circle built from it — is corrupted.
>
> Also guards the empty/one-point scan, which the original loop only survived
> by accident.

**Reproduction.** Publish a `LaserScan` whose first return is a near-range
point belonging to a different object from the rest of the first group. The
first `/raw_obstacles` circle is displaced towards that point; every later
group is correct.

**Measurement.** Found by porting the extractor's scan-window logic into a
standalone probe and comparing group boundaries against the analytic
expectation; the first group was consistently one point wide of it. Corrected
group boundaries reproduce the analytic circumcircle prediction
(`r_fit ≈ 1.15·r` for `d ≫ r`) to within the fit residual.

---

## 02 — `obstacle_tracker`: two-point track-initiation covariance

**Commit message**

> Initialise P from R instead of the identity
>
> A track seeded from a single measurement knows its position to R and its
> velocity only to the difference of two measurements, 2R/T². Upstream leaves
> `P = I`, which matches neither the seed uncertainty nor the configured
> `measurement_variance`. The filter therefore starts over-confident in the
> seeded (differenced, hence noisy) velocity and inconsistent with its own R,
> and discounts position innovations while that velocity converges.
>
> Standard two-point initiation (Bar-Shalom, *Estimation with Applications to
> Tracking and Navigation*, §3.3.3).

**Reproduction.** Track a single object crossing at constant velocity. With
`P = I` the reported centre lags the object for roughly the first second of
the track's life — precisely the window a safety consumer cares about, since
that is when an object first becomes relevant.

**Measurement.** On a 0.5 m/s and a 0.8 m/s single-crosser fixture, the
velocity-RMSE gate that fails with `P = I` passes with this change, and the
newly-born-track position error settles within one measurement instead of
over ~10.

**Note for the maintainer.** This is dimensionally correct only if
`measurement_variance` is a real m² value. The shipped default of 1.0 asserts
a 1 m 1σ measurement; see the related report on parameter documentation.

---

## 03 — `livox_ros_driver2`: declare the build-order dependency on the SDK

**Commit message**

> Declare the Livox-SDK2 build dependency in package.xml
>
> CMakeLists.txt resolves the SDK with `find_library`/`find_path` against the
> install prefix, but nothing tells colcon that the SDK package must be built
> first. A parallel build happens to schedule them correctly often enough to
> hide this; a sequential build (or an unlucky scheduler) fails at
> `find_library(LIVOX_LIDAR_SDK_LIBRARY)`.

**Reproduction.** `colcon build --executor sequential` in a workspace
containing both the SDK vendor package and the driver, from clean.

**Measurement.** Reproduced deterministically on an emulated aarch64 build,
where the sequential scheduler exposed it immediately; the same workspace had
built for an entire development phase on an x86-64 machine with a parallel
build.

---

## 04 — DLIO: subscribe to the cloud with SensorData QoS

**Commit message**

> Use SensorData QoS for the point-cloud subscription
>
> The subscription is Reliable, so it never matches a best-effort cloud
> publisher — which is what the Livox driver's own examples, most simulators
> and every bag replayed with sensor QoS provide. The failure mode is silent:
> the topic shows as connected and no data arrives.

**Reproduction.** Publish `/livox/lidar` with `rclcpp::SensorDataQoS()` and
start DLIO; `ros2 topic info -v` shows the endpoints, and the node never
produces odometry.

**Measurement.** Bench replay: 278 clouds published best-effort, 0 received
before the change, 274 after.

---

## 05 — `livox_ros_driver2`: `/livox/imu` ignores the `frame_id` parameter

**Commit message**

> Honour the frame_id parameter in InitImuMsg
>
> The point-cloud path uses the configured `frame_id`; the IMU path hardcodes
> "livox_frame". Any integration that names its sensor frame anything else
> gets an IMU stream in a frame that does not exist in its TF tree.

**Reproduction.** Set `frame_id: mid360_link`, run the driver, and compare
`/livox/lidar` and `/livox/imu` headers.

---

## 06 — `obstacle_extractor`: gate circles on the fitted radius, and say what is dropped

**Commit message**

> Test max_circle_radius against the fit, not fit + radius_enlargement
>
> `detectCircles()` adds the safety enlargement before testing the plausibility
> bound, so `max_circle_radius` really means "max radius minus
> radius_enlargement" and making the inflation more conservative silently
> shrinks the largest obstacle the detector can report. Also counts and
> throttle-logs both drop paths: an obstacle that disappears here is invisible
> to everything downstream, and nothing downstream can compensate for it.

**Reproduction.** With `max_circle_radius: 0.60`, `radius_enlargement: 0.17`,
a 0.40 m cylinder at 2–3 m produces no detection at all, and *regains* one
when a neighbour occludes part of it (the shorter chord fits a smaller
circle), so it flickers as the sensor moves.

**Measurement.** 210 analytic-ray configurations over radius × range ×
visible-arc: the rule `keep ⟺ fit + 0.17 < 0.60` reproduces all 17
non-detections and all 43 detections with no exceptions. The fit
over-estimates a fully visible cylinder by `+0.084·r`, so the shipped
configuration cut at ≈0.397 m of true radius.

---

## Withdrawn: `pcl_ros` filter output QoS

Carried since Phase 2 as patch 0002 (publish `output` with SensorData QoS
instead of Reliable). **Do not open this PR.** The `ros2` branch tip
(`e264aff1`) has since added `QosOverridingOptions` to `pub_output_`, so the
reliability is settable from parameters and no source change is needed:

```yaml
qos_overrides./output.publisher.reliability: best_effort
```

Our pin (`9d078eb`, 2.6.1) predates that. Action is on our side, not
upstream's: when the pin moves past `e264aff1`, delete patch 0002 and set the
override in `crop_box_self_filter`'s parameters instead.

## Not upstreamable

* **0007 (P-3, `CircleObstacle.covariance`)** changes a public message. It
  stays in the fork. If it were ever offered upstream it would have to be an
  additive field with a compatibility discussion, not a drive-by.
* **0003 / 0004 / 0005's remainder** are project decisions (componentisation,
  our QoS contract, stamped TF lookup, a `package.xml` for a repo that ships
  `package_ROS2.xml`), not general fixes. They stay in the series.

## Separate target: an ament_cmake issue, not a PR

`<pkg> exports the library '<pkg>' which couldn't be found` while
`find_library` reports finding it on the line above. Mechanism:
`set(_lib "NOTFOUND")` in the generated `*-extras.cmake` creates a NORMAL
variable that shadows the CACHE variable `find_library` writes; `_lib` is one
cache slot shared by every such file in a configure, so whether the shadow
survives depends on CMP0126 in the enclosing scope, hence on the include
stack, hence on which packages happen to be installed. Reproduces on CMake
3.22.1 in an arm64 Ubuntu 22.04 ROS Humble image and not on x86-64 with the
same sources, and the package it blames moves as the installed set changes.
Diagnostic: `../../tools/diagnose_ament_export_libraries.py`.

This is an **issue report** against `ros2/ament_cmake`, and it also needs
authorisation before it goes out. Do not attach a patch: a fix that changes
link lines on a robot without being understood is worse than the failure, and
that was the reason the local workaround was withdrawn.
