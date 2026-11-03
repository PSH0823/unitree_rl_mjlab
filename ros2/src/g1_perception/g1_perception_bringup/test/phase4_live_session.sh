#!/bin/bash
# Live full-stack session driver + T6 staleness drill (Phase 4).
#
#   phase4_live_session.sh <mode:oracle|shadow|estimated> <secs> <outdir> [t6]
#
# Runs: simulate (shadow run-tree, ROS2 module, scripted commands, Filter-I/O
# capture) + description + sidecar + perception container; dumps
# /dpcbf/status; with "t6" it SIGKILLs the container mid-run and restarts it
# 5 s later (the §16.2-T6 drill — timeline extracted from the capture with
# phase4_capture_stats.py t6). MANUAL part of T6: watch /dpcbf/status level
# go OK→WARN→ERROR→OK and verify the robot command freezes in RViz.
#
# Machine-specific prereqs (see ros2/README.md runtime notes): shadow
# run-tree at $SHADOW with the build_ros2 binary + use_joystick:0 +
# enable_elastic_band:1 config; mesa GL; lo-pinned CYCLONEDDS_URI below.
MODE=$1; SECS=$2; OUT=$3; DRILL=${4:-}
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
SHADOW=${SHADOW:-/tmp/sim_shadow_phase4}
PROFILE=$WS/test_fixtures/t1_baseline/t1_command_profile.txt
export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
mkdir -p $OUT

cleanup() {
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null; sleep 1
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null
  pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null
  pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null; sleep 1
  pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null
  pkill -f 'robot_state_publishe[r]' 2>/dev/null
  pkill -f 'base_footprint_publishe[r]' 2>/dev/null
}
cleanup; sleep 2

cd $SHADOW/simulate/build
env LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa DISPLAY=:1 \
    UNITREE_DPCBF_MODE=$MODE \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS=$PROFILE \
    UNITREE_DPCBF_FILTER_LOG=$OUT/capture_$MODE.bin \
    ./unitree_mujoco > $OUT/sim_$MODE.log 2>&1 &
SIM=$!
sleep 14   # model load + mirror dump + DDS up

ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true > $OUT/desc.log 2>&1 &
DESC=$!
ros2 launch g1_perception_bringup source_sim.launch.py > $OUT/src.log 2>&1 &
SRC=$!
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true > $OUT/perc.log 2>&1 &
PERC=$!
sleep 10

/usr/bin/python3 $WS/src/g1_perception/g1_perception_bringup/test/phase4_status_dump.py $SECS $OUT/status_$MODE.jsonl &
PROBE=$!
/usr/bin/python3 $HERE/phase4_latency_probe.py $SECS $OUT/latency_$MODE.txt > /dev/null 2>&1 &
timeout 12 ros2 topic hz /lowstate --window 2000 > $OUT/lowstate_hz_$MODE.txt 2>&1 &
timeout 12 ros2 topic hz /obstacles_safe --window 100 > $OUT/safe_hz_$MODE.txt 2>&1 &

if [ "$DRILL" = "t6" ]; then
  DRILL_AT=$((SECS/2))
  sleep $DRILL_AT
  echo "T6: killing perception container at wall +${DRILL_AT}s" | tee $OUT/t6_timeline.txt
  date +%s.%N >> $OUT/t6_timeline.txt
  pkill -KILL -f 'rclcpp_components/component_containe[r]'
  kill -INT $PERC 2>/dev/null
  sleep 5
  echo "T6: restarting perception container" | tee -a $OUT/t6_timeline.txt
  date +%s.%N >> $OUT/t6_timeline.txt
  ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true > $OUT/perc2.log 2>&1 &
  PERC=$!
fi

wait $PROBE
cleanup
echo "session $MODE done -> $OUT"
ls -la $OUT | tail -8
