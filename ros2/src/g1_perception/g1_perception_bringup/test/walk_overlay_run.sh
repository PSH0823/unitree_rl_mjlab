#!/bin/bash
# One-command walking visual evidence: brings the walking stack up exactly as
# walk_ab_run.sh does and renders the offscreen cloud + tracked overlay.
#   walk_overlay_run.sh <out.png> [scenario] [secs]
set +u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
OUT=${1:-/tmp/walk_overlay.png}; SCEN=${2:-W3}; SECS=${3:-110}
TREE=/tmp/sim_walk_overlay
export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"
/usr/bin/python3 "$HERE/walk_scenarios.py" "$SCEN" "$REPO/dpcbf/config/dpcbf_config.yaml" \
    "$TREE/dpcbf/config/dpcbf_config.yaml"
cleanup() {
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null || true
  pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null || true
  pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null || true; pkill -f 'g1_ctr[l]' 2>/dev/null || true
  pkill -f 'robot_state_publishe[r]' 2>/dev/null || true
  pkill -f 'base_footprint_publishe[r]' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null || true
  pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null || true; pkill -KILL -f 'g1_ctr[l]' 2>/dev/null || true
}
cleanup
( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo > /tmp/ov_g1.log 2>&1 ) &
sleep 3
( cd "$TREE/simulate/build" && env LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
    DISPLAY=${DISPLAY:-:1} UNITREE_DPCBF_MODE=estimated \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS="$HERE/../config/walk_profile.txt" \
    UNITREE_MUJOCO_BAND_LENGTH=0.572 UNITREE_MUJOCO_BAND_RELEASE=24,4 \
    ./unitree_mujoco > /tmp/ov_sim.log 2>&1 ) &
sleep 6
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true > /tmp/ov_desc.log 2>&1 &
ros2 launch g1_perception_bringup source_sim.launch.py > /tmp/ov_src.log 2>&1 &
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true > /tmp/ov_perc.log 2>&1 &
/usr/bin/python3 "$HERE/walk_overlay.py" "$SECS" "$OUT" --settle 50 --panels 4
cleanup
