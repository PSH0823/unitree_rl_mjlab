#!/usr/bin/env bash
# Source this file in every terminal participating in the single-machine
# MuJoCo Navigation session.  A common CycloneDDS URI is required: selecting
# the same RMW implementation alone does not pin every process to loopback.

_unitree_nav_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset FASTRTPS_DEFAULT_PROFILES_FILE
export CYCLONEDDS_URI="file://${_unitree_nav_root}/ros2/src/g1_perception/g1_perception_bringup/config/cyclonedds/localhost.xml"

echo "Simulation ROS environment:"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"

unset _unitree_nav_root
