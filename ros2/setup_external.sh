#!/bin/bash
# Import pinned external sources and apply recorded patches (§13 pinning policy).
# Run from ros2/: ./setup_external.sh
set -euo pipefail
cd "$(dirname "$0")"
vcs import --shallow src < deps.repos
touch src/external/MuJoCo-LiDAR/COLCON_IGNORE \
      src/external/unitree_dds_wrapper/COLCON_IGNORE \
      src/external/cyclonedds-cxx/COLCON_IGNORE
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
echo "External sources ready."
