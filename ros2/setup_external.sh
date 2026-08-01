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
echo "External sources ready."
