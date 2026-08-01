#!/bin/bash
# Import pinned external sources and apply recorded patches (§13 pinning policy).
# Run from ros2/: ./setup_external.sh
set -euo pipefail
cd "$(dirname "$0")"
vcs import --shallow src < deps.repos
touch src/external/MuJoCo-LiDAR/COLCON_IGNORE \
      src/external/unitree_dds_wrapper/COLCON_IGNORE \
      src/external/cyclonedds-cxx/COLCON_IGNORE
# The C++ dds_wrapper at the pinned SHA lacks the simulator-facing joystick
# API this repo's bridge was written against (the original build machine ran
# a modified copy). Restore it with the recorded patch; goes away when the
# project org fork lands (open follow-up).
git -C src/external/unitree_dds_wrapper apply --check ../../../patches/0001-unitree-dds-wrapper-restore-sim-joystick-api.patch 2>/dev/null \
  && git -C src/external/unitree_dds_wrapper apply ../../../patches/0001-unitree-dds-wrapper-restore-sim-joystick-api.patch \
  || echo "dds_wrapper patch already applied"
echo "External sources ready."
