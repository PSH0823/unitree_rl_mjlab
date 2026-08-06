#!/bin/bash
# Full-stack plot rehearsal (visualization runbook "방법 C"): the walking sim
# session EXACTLY as walk_ab_run.sh builds it — g1_ctrl policy + simulator
# (DPCBF in the 1 kHz seam, scripted bring-up/walk profile, elastic-band
# lowering) + description + source_sim + perception — brought up ONCE and left
# RUNNING so the operator can attach the plotting client from a second
# terminal, i.e. the closest on-bench analogue of a Computer 2/3 session.
#
#   walk_plot_session.sh [scenario] [mode]      # default: W2 estimated
#
#   terminal 2:
#     source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
#     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
#     export CYCLONEDDS_URI=file://<ws>/install/share/g1_perception_bringup/config/cyclonedds/localhost.xml
#     ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
#   (the script prints these lines with paths filled in)
#
# Environment knobs (all optional):
#   WALK_TREE=<dir>            shadow tree (default /tmp/sim_walk_plot)
#   WALK_BAND_RELEASE=t0,ramp  default 34,6 (runbook §6.1.4 preferred)
#   WALK_HEADLESS=1            force Xvfb even if DISPLAY is set
#   UNITREE_DPCBF_PLOT_RATE    plot bridge rate override (yaml default 30)
#
# Timeline (sim seconds, from config/walk_profile.txt + band release 34,6):
#   15.5 FixStand   21.5 RLBase(policy)   34-40 band lowered   40+ walking
# Everything is torn down on Ctrl-C.
set +u   # ROS setup.bash trips over unbound AMENT_* variables
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)          # ros2/
REPO=$(cd "$WS/.." && pwd)
SCEN=${1:-W2}
MODE=${2:-estimated}
TREE=${WALK_TREE:-/tmp/sim_walk_plot}
LOGS=${WALK_PLOT_LOGS:-/tmp/walk_plot_logs}
mkdir -p "$LOGS"

export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="file://$WS/src/g1_perception/g1_perception_bringup/config/cyclonedds/localhost.xml"

[ -x "$REPO/simulate/build_ros2/unitree_mujoco" ] || {
  echo "FAIL: simulate/build_ros2/unitree_mujoco not built (-DUNITREE_MUJOCO_WITH_ROS2=ON)" >&2; exit 1; }
[ -x "$REPO/deploy/robots/g1/build/g1_ctrl" ] || {
  echo "FAIL: deploy/robots/g1/build/g1_ctrl not built" >&2; exit 1; }

# ---- run tree (walk_ab_run.sh pattern, incl. the ros2/ link main.cc needs) --
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
ln -sfn "$REPO/ros2" "$TREE/ros2"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"
/usr/bin/python3 "$HERE/walk_scenarios.py" "$SCEN" \
    "$REPO/dpcbf/config/dpcbf_config.yaml" "$TREE/dpcbf/config/dpcbf_config.yaml" || exit 1

cleanup() {
  echo; echo "=== tearing down ==="
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null || true
  pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null || true
  pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null || true
  pkill -f 'g1_ctr[l]' 2>/dev/null || true
  pkill -f 'robot_state_publishe[r]' 2>/dev/null || true
  pkill -f 'base_footprint_publishe[r]' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null || true
  pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null || true
  pkill -KILL -f 'g1_ctr[l]' 2>/dev/null || true
}
trap 'cleanup; exit 0' INT TERM
cleanup >/dev/null 2>&1

# g1_ctrl FIRST (it blocks in wait_for_connection() until rt/lowstate appears;
# started after the simulator it races the profile chords — walk_ab_run.sh).
( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo \
    > "$LOGS/g1_ctrl.log" 2>&1 ) &
sleep 3

SIM_WRAP=()
if [ "${WALK_HEADLESS:-0}" = "1" ] || [ -z "${DISPLAY:-}" ]; then
  SIM_WRAP=(xvfb-run -a)
  echo "simulator: headless (Xvfb)"
else
  echo "simulator: DISPLAY=$DISPLAY"
fi
( cd "$TREE/simulate/build" && env LIBGL_ALWAYS_SOFTWARE=1 \
    __GLX_VENDOR_LIBRARY_NAME=mesa \
    UNITREE_DPCBF_MODE=$MODE \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS="$WS/src/g1_perception/g1_perception_bringup/config/walk_profile.txt" \
    UNITREE_MUJOCO_BAND_LENGTH=${WALK_BAND_LENGTH:-0.572} \
    UNITREE_MUJOCO_BAND_RELEASE=${WALK_BAND_RELEASE:-34,6} \
    "${SIM_WRAP[@]}" ./unitree_mujoco > "$LOGS/sim.log" 2>&1 ) &
sleep 6
grep -E "plot bridge|obstacle source" "$LOGS/sim.log" || true

ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true \
    > "$LOGS/desc.log" 2>&1 &
ros2 launch g1_perception_bringup source_sim.launch.py > "$LOGS/src.log" 2>&1 &
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true \
    > "$LOGS/perc.log" 2>&1 &

cat <<EOF

=== walking rehearsal up: scenario=$SCEN mode=$MODE (logs: $LOGS) ===
Timeline (sim s): 15.5 FixStand | 21.5 policy | 34-40 band down | 40+ walking

Attach the plotting client from ANOTHER terminal:

  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
  export CYCLONEDDS_URI=file://$WS/src/g1_perception/g1_perception_bringup/config/cyclonedds/localhost.xml
  ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py

Quick probes:  ros2 topic hz /dpcbf/plot   ros2 topic hz /odom   ros2 topic hz /obstacles_safe
Ctrl-C here tears the whole stack down.
EOF

wait
