#!/bin/bash
# Import pinned external sources and apply recorded patches (§13 pinning policy).
# Run from ros2/: ./setup_external.sh
set -euo pipefail
cd "$(dirname "$0")"
vcs import --shallow src < deps.repos

# --- distro-coupled checkout: rmw_cyclonedds --------------------------------
# rmw_cyclonedds is the one external whose sources track the ROS distro's
# internal C++ API rather than a public one. Its humble branch locks
# `common->node_update_mutex` directly, and Jazzy made
# rmw_dds_common::Context::node_update_mutex PRIVATE — on Jazzy the humble
# branch fails with eight "is private within this context" errors in
# rmw_node.cpp and takes the rest of the workspace down with it.
#
# The deb (ros-jazzy-rmw-cyclonedds-cpp) is NOT the fix: it links the distro's
# own CycloneDDS 0.10.5 while unitree_sdk2 links the 0.10.2 built here, which
# puts two CycloneDDS copies in one process — exactly the symbol conflict gate
# T10 exists to prove does not happen. So the jazzy branch is built from
# source against the same pinned 0.10.2, and it compiles clean.
#
# SHAs resolved 2026-08-14 (git ls-remote); same pinning policy as deps.repos.
case "${ROS_DISTRO:-humble}" in
  jazzy)  RMW_CYCLONEDDS_SHA=c14957709256c344d1dfc07a5c5cc60ea480d9a2 ;;  # branch jazzy
  kilted) RMW_CYCLONEDDS_SHA=b67d268da024e7b49b61882aa67bee07d5a3c523 ;;  # branch kilted
  *)      RMW_CYCLONEDDS_SHA=fa8831b9f331d669c3ea408fd00606a223c6bbd9 ;;  # branch humble (deps.repos)
esac
if [ "$(git -C src/external/rmw_cyclonedds rev-parse HEAD)" != "$RMW_CYCLONEDDS_SHA" ]; then
  echo "rmw_cyclonedds -> ${ROS_DISTRO:-humble} branch ($RMW_CYCLONEDDS_SHA)"
  git -C src/external/rmw_cyclonedds fetch --depth 1 origin "$RMW_CYCLONEDDS_SHA"
  git -C src/external/rmw_cyclonedds checkout --detach "$RMW_CYCLONEDDS_SHA"
fi

touch src/external/MuJoCo-LiDAR/COLCON_IGNORE \
      src/external/unitree_dds_wrapper/COLCON_IGNORE \
      src/external/cyclonedds-cxx/COLCON_IGNORE
# Livox-SDK2 is plain CMake with no package.xml: colcon cannot build it, and
# the livox_sdk2_vendor package drives it into the merged prefix instead
# (upstream's instructions install to /usr/local, which needs sudo).
touch src/external/Livox-SDK2/COLCON_IGNORE
# perception_pcl 2.6.1: only pcl_ros is built (for the CropBox component the
# Humble binary lacks); pcl_conversions stays the Humble binary.
touch src/external/perception_pcl/perception_pcl/COLCON_IGNORE \
      src/external/perception_pcl/pcl_conversions/COLCON_IGNORE
# §7.1 QoS contract: filter output publisher must be SensorData (upstream is
# Reliable). Upstream-able; PR pending with the org-fork follow-up.
git -C src/external/perception_pcl apply --check ../../../patches/0002-pcl-ros-filter-output-sensor-data-qos.patch 2>/dev/null \
  && git -C src/external/perception_pcl apply ../../../patches/0002-pcl-ros-filter-output-sensor-data-qos.patch \
  || echo "pcl_ros QoS patch already applied"
# The C++ dds_wrapper at the pinned SHA lacks the simulator-facing joystick
# API this repo's bridge was written against (the original build machine ran
# a modified copy). Restore it with the recorded patch; goes away when the
# project org fork lands (open follow-up).
git -C src/external/unitree_dds_wrapper apply --check ../../../patches/0001-unitree-dds-wrapper-restore-sim-joystick-api.patch 2>/dev/null \
  && git -C src/external/unitree_dds_wrapper apply ../../../patches/0001-unitree-dds-wrapper-restore-sim-joystick-api.patch \
  || echo "dds_wrapper patch already applied"
# P-1 (§9.5): componentization (upstream ships plain executables only), scan
# subscription QoS SensorData (upstream Reliable never matches a best-effort
# laser publisher), obstacle publishers Reliable depth 5 (§7.1), and TF lookup
# at the scan stamp instead of latest (odom-frame consistency + replay
# determinism). Upstream-able; PR with the org-fork follow-up.
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0003-obstacle-detector-p1-components-qos-stamped-tf.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0003-obstacle-detector-p1-components-qos-stamped-tf.patch \
  || echo "obstacle_detector P-1 patch already applied"
# P-2 (§9.5, fired by the Phase-3 decision rule — T8 nondeterministic and T5
# velocity RMSE over gate with the shipped timer model): measurement-driven
# tracker — predict + correct on raw_obstacles arrival with dt from header
# stamps, no wall timer, output stamped with the measurement time. Also the
# Appendix-A P-2 scope items: radius-residual weight 0.3 in the association
# cost; covariances interpreted per-step at sensor_rate, scaled with dt.
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0004-obstacle-detector-p2-measurement-driven-tracker.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0004-obstacle-detector-p2-measurement-driven-tracker.patch \
  || echo "obstacle_detector P-2 patch already applied"
# Phase 5A, livox_ros_driver2. Upstream ships package_ROS1.xml/package_ROS2.xml
# and expects build.sh to copy the right one into place — but build.sh also
# `rm -rf`s ../../build ../../devel ../../install, which would delete this
# workspace. Patch 0005 therefore adds package.xml (the ROS2 one verbatim) and
# a colcon.pkg carrying the -DROS_EDITION=ROS2 -DDISTRO_ROS=humble that
# CMakeLists.txt needs (without DISTRO_ROS the humble typesupport-target branch
# is skipped and the link fails), plus one upstream bug fix: InitImuMsg
# hardcoded frame_id "livox_frame" instead of honouring the frame_id parameter.
git -C src/external/livox_ros_driver2 apply --check ../../../patches/0005-livox-driver-ros2-humble-package-and-imu-frame-id.patch 2>/dev/null \
  && git -C src/external/livox_ros_driver2 apply ../../../patches/0005-livox-driver-ros2-humble-package-and-imu-frame-id.patch \
  || echo "livox driver patch already applied"
# Phase 5A, DLIO. Its cloud subscription is Reliable, which cannot match the
# best-effort cloud publishers this architecture uses everywhere (§7.1) — DLIO
# would show a connected topic and receive nothing, the exact failure class the
# obstacle_extractor hit in Phase 3. Patch 0006 makes it SensorData (depth 1)
# and /odom Reliable depth 10.
git -C src/external/direct_lidar_inertial_odometry apply --check ../../../patches/0006-dlio-cloud-subscription-sensor-data-qos.patch 2>/dev/null \
  && git -C src/external/direct_lidar_inertial_odometry apply ../../../patches/0006-dlio-cloud-subscription-sensor-data-qos.patch \
  || echo "DLIO QoS patch already applied"
# P-3 (§9.6 sigma term / D6's one sanctioned message change, fired in the
# interim block after 5A). The tracker keeps a full posterior estimate-error
# covariance per track but publishes none of it, so `safety_obstacle_filter`
# had no way to widen a circle the tracker itself is unsure about. Patch 0007
# adds `float64[3] covariance` = [var_x, var_y, var_r] to CircleObstacle and
# fills it from the per-axis KFs' P(0,0) in publishObstacles(). Only meaningful
# post-P-2, which made P start from R instead of upstream's identity.
# NOT upstreamable as-is (it changes a public message); carried in the fork.
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0007-obstacle-detector-p3-per-track-covariance.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0007-obstacle-detector-p3-per-track-covariance.patch \
  || echo "obstacle_detector P-3 patch already applied"
# Interim block: declare the driver's build-order dependency on
# livox_sdk2_vendor. colcon orders packages from package.xml alone, and nothing
# said the driver needs the vendor package's liblivox_lidar_sdk_shared.so in the
# prefix first. The dev machine's PARALLEL build happened to schedule them in a
# working order and hid this through all of Phase 5A; a sequential build on
# another machine fails at CMakeLists.txt:249 find_library instead.
git -C src/external/livox_ros_driver2 apply --check ../../../patches/0008-livox-driver-declare-sdk-vendor-dependency.patch 2>/dev/null \
  && git -C src/external/livox_ros_driver2 apply ../../../patches/0008-livox-driver-declare-sdk-vendor-dependency.patch \
  || echo "livox driver vendor-dependency patch already applied"
# P-4 (§9.6, workstream C gap G2). `detectCircles()` adds radius_enlargement to
# the fitted radius BEFORE testing max_circle_radius, so the shipped 0.60/0.17
# really cut at a 0.43 m fit — ~0.397 m of true radius once the fit's +8.4%*r
# bias is included — with no diagnostic, and intermittently, because occlusion
# shortens the chord and brings the same obstacle back under the cut. Patch
# 0009 gates on the fit (making max_circle_radius mean what §9.6 says) and
# counts + throttle-logs BOTH silent drop paths. Upstream-able.
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0009-obstacle-detector-p4-fit-radius-gate-and-drop-observability.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0009-obstacle-detector-p4-fit-radius-gate-and-drop-observability.patch \
  || echo "obstacle_detector P-4 patch already applied"
# P-5 (config honesty): P-2 removed the tracker's wall timer but kept parsing
# `loop_rate` into a member nothing read, leaving the yaml a knob that looks
# tunable and does nothing. Patch 0010 removes the dead member/read and turns
# a config that still sets the (still-declared) name into an explicit
# RCLCPP_WARN instead of a silent ignore.
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0010-obstacle-detector-p5-retire-inert-tracker-loop-rate.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0010-obstacle-detector-p5-retire-inert-tracker-loop-rate.patch \
  || echo "obstacle_detector P-5 patch already applied"
# F-1 (Foxy portability): the G1's onboard computer runs Foxy (Ubuntu 20.04)
# while this workspace is developed on Humble. Patch 0011 makes the fork build
# on BOTH — rosidl_get_typesupport_target vs rosidl_target_interfaces, the
# tf2_geometry_msgs .hpp/.h spelling plus the missing unstamped-Pose
# doTransform, declare_parameter-by-type, and the const-reference subscription
# callbacks Foxy's rclcpp cannot bind. No computation changes; see the patch
# header. Verify with ros2/tools/foxy_docker.sh (focal container).
git -C src/external/obstacle_detector_2 apply --check ../../../patches/0011-obstacle-detector-f1-foxy-portability.patch 2>/dev/null \
  && git -C src/external/obstacle_detector_2 apply ../../../patches/0011-obstacle-detector-f1-foxy-portability.patch \
  || echo "obstacle_detector F-1 patch already applied"
# F-1 continued across the rest of src/external. Same rule everywhere: pick the
# spelling each distro accepts, never change what the code computes.
#   0012 pointcloud_to_laserscan — tf2_sensor_msgs .h/.hpp
#   0013 pcl_ros                 — tf2_geometry_msgs .h/.hpp, two rclcpp 16 APIs
#   0014 livox_ros_driver2       — typesupport idiom by capability, not DISTRO_ROS
#   0015 DLIO                    — typesupport idiom, callback form, and PCL 1.10's
#                                  boost::shared_ptr PointCloud::Ptr (focal)
# rmw_cyclonedds has NO patch and is not built on Foxy: its foxy branch needs
# CycloneDDS 0.7's ddsi_sertopic API and its humble branch needs rmw headers
# Foxy lacks, so Foxy uses the ros-foxy-rmw-cyclonedds-cpp deb instead.
git -C src/external/pointcloud_to_laserscan apply --check ../../../patches/0012-pointcloud-to-laserscan-f1-foxy-tf2-header.patch 2>/dev/null \
  && git -C src/external/pointcloud_to_laserscan apply ../../../patches/0012-pointcloud-to-laserscan-f1-foxy-tf2-header.patch \
  || echo "pointcloud_to_laserscan F-1 patch already applied"
git -C src/external/perception_pcl apply --check ../../../patches/0013-pcl-ros-f1-foxy-portability.patch 2>/dev/null \
  && git -C src/external/perception_pcl apply ../../../patches/0013-pcl-ros-f1-foxy-portability.patch \
  || echo "pcl_ros F-1 patch already applied"
git -C src/external/livox_ros_driver2 apply --check ../../../patches/0014-livox-driver-f1-foxy-typesupport-idiom.patch 2>/dev/null \
  && git -C src/external/livox_ros_driver2 apply ../../../patches/0014-livox-driver-f1-foxy-typesupport-idiom.patch \
  || echo "livox driver F-1 patch already applied"
git -C src/external/direct_lidar_inertial_odometry apply --check ../../../patches/0015-dlio-f1-foxy-and-pcl-1-10-portability.patch 2>/dev/null \
  && git -C src/external/direct_lidar_inertial_odometry apply ../../../patches/0015-dlio-f1-foxy-and-pcl-1-10-portability.patch \
  || echo "DLIO F-1 patch already applied"
# J-1 (Ubuntu 24.04 / GCC 13). Livox-SDK2 v1.3.1 relies on <cstdint> arriving
# through some other header; GCC 13 stopped providing it transitively, so on
# noble the SDK dies with "'uint8_t' in namespace 'std' does not name a type"
# and takes livox_ros_driver2 and the whole hardware branch with it. Patch 0016
# adds the include to the six translation units that need it. Upstream fixed
# this too, but only inside the master commit that also adds Avia2 support and
# rewrites the 3rdparty tree — far more than this pin should carry. Harmless on
# 22.04/20.04: the include is simply already satisfied there.
git -C src/external/Livox-SDK2 apply --check ../../../patches/0016-livox-sdk2-cstdint-for-gcc-13.patch 2>/dev/null \
  && git -C src/external/Livox-SDK2 apply ../../../patches/0016-livox-sdk2-cstdint-for-gcc-13.patch \
  || echo "Livox-SDK2 J-1 patch already applied"
# J-2 (merged-prefix install race; not distro-specific, but it is what a clean
# rebuild hits first). unitree_sdk2 installs its BUNDLED CycloneDDS into the
# same prefix the source-built 0.10.2 goes to, and the two use opposite symlink
# layouts for libddsc.so/.so.0 — interleaved they leave a symlink LOOP that
# fails every later link with "Too many levels of symbolic links". Patch 0017
# stops unitree_sdk2 installing libddsc.* (both copies are 0.10.2 and the
# source build defines every symbol the SDK and libddscxx need); libddscxx.* is
# still installed from there, cyclonedds-cxx being COLCON_IGNOREd.
git -C src/external/unitree_sdk2 apply --check ../../../patches/0017-unitree-sdk2-do-not-install-bundled-libddsc.patch 2>/dev/null \
  && git -C src/external/unitree_sdk2 apply ../../../patches/0017-unitree-sdk2-do-not-install-bundled-libddsc.patch \
  || echo "unitree_sdk2 J-2 patch already applied"
echo "External sources ready."
