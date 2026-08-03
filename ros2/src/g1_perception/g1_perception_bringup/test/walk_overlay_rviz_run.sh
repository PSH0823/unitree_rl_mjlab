#!/bin/bash
# Live walking run with the §4.6 DPCBF overlay up, screenshotted (runbook §4.6).
#
#   walk_overlay_rviz_run.sh [outdir] [scenario] [mode]
#     defaults: /tmp/walk_overlay_rviz  W1  estimated
#
# Same bring-up contract as walk_ab_run.sh (which see, and §6.1): g1_ctrl
# FIRST, staggered chords from the scripted profile, elastic band lowered onto
# an actively balancing policy. What this adds:
#
#   * viz.launch.py with overlay:=on, pointed at the RUN TREE's config — the
#     scenario writer rewrites the obstacle field there, and an overlay reading
#     the repo copy would be recomputing against parameters the simulator did
#     not load;
#   * UNITREE_DPCBF_FILTER_LOG on the simulator, so the run leaves a 1 kHz
#     capture the boundary gate can be joined against;
#   * overlay_log, the 10 Hz JSONL the join consumes;
#   * an RViz screenshot off the private Xvfb (§4.2), which is the deliverable;
#   * per-process CPU over the measurement window, against the §17.4 budget.
#
# A bag replay cannot substitute: it has the filter out of the loop, so the
# constraint layer has nothing to draw and /dpcbf/status never appears.
set +u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
OUT=${1:-/tmp/walk_overlay_rviz}
SCEN=${2:-W1}
MODE=${3:-estimated}
TREE=${WALK_TREE:-/tmp/sim_walk_overlay}
SECS=${WALK_SECS:-110}
SETTLE=${WALK_SETTLE:-40}
SHOT_AT=${SHOT_AT:-78}          # wall seconds after perception start
XDISP=${XDISP:-:77}
mkdir -p "$OUT"

export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

cleanup() {
  pkill -f 'rviz[2]' 2>/dev/null || true
  pkill -f 'dpcbf_overla[y]' 2>/dev/null || true
  pkill -f 'obstacles_marker_rela[y]' 2>/dev/null || true
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
  sleep 1
}
cleanup

# ---- run tree (dpcbf/ stays byte-for-byte untouched; D3) -------------------
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"
PROFILE="$OUT/walk_profile.txt"
cp "$HERE/../config/walk_profile.txt" "$PROFILE"
/usr/bin/python3 "$HERE/walk_scenarios.py" "$SCEN" \
    "$REPO/dpcbf/config/dpcbf_config.yaml" "$TREE/dpcbf/config/dpcbf_config.yaml" \
    > "$OUT/scenario.txt" || exit 1
cat "$OUT/scenario.txt"

# ---- private display ------------------------------------------------------
if ! xdpyinfo -display "$XDISP" >/dev/null 2>&1; then
  Xvfb "$XDISP" -screen 0 1920x1080x24 -nolisten tcp > "$OUT/xvfb.log" 2>&1 &
  sleep 3
fi

# ---- bring-up -------------------------------------------------------------
( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo \
    > "$OUT/g1_ctrl.log" 2>&1 ) &
sleep 3

( cd "$TREE/simulate/build" && env LIBGL_ALWAYS_SOFTWARE=1 \
    __GLX_VENDOR_LIBRARY_NAME=mesa DISPLAY="$XDISP" \
    UNITREE_DPCBF_MODE=$MODE \
    UNITREE_DPCBF_FILTER_LOG="$OUT/capture.bin" \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS="$PROFILE" \
    UNITREE_MUJOCO_BAND_LENGTH=${WALK_BAND_LENGTH:-0.572} \
    UNITREE_MUJOCO_BAND_RELEASE=${WALK_BAND_RELEASE:-34,6} \
    ./unitree_mujoco > "$OUT/sim.log" 2>&1 ) &
sleep 6

ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true \
    > "$OUT/desc.log" 2>&1 &
ros2 launch g1_perception_bringup source_sim.launch.py > "$OUT/src.log" 2>&1 &
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true \
    > "$OUT/perc.log" 2>&1 &
sleep 8

env DISPLAY="$XDISP" LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
    ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true \
    overlay:=on dpcbf_config:="$TREE/dpcbf/config/dpcbf_config.yaml" \
    overlay_log:="$OUT/overlay.jsonl" > "$OUT/viz.log" 2>&1 &

# ---- screenshot + CPU, while the probe scores the walk ---------------------
(
  sleep "$SHOT_AT"
  for tag in a b; do
    /usr/bin/python3 -c "
from PIL import ImageGrab
ImageGrab.grab(xdisplay='$XDISP').save('$OUT/rviz_overlay_$tag.png')
print('shot $tag')" >> "$OUT/shots.log" 2>&1
    sleep 6
  done
) &

(
  sleep $((SHOT_AT - 45))
  /usr/bin/python3 "$HERE/proc_cpu.py" 45 \
      dpcbf_overlay component_container sim_mjlidar_bridge rviz2 \
      > "$OUT/cpu.txt" 2>&1
) &

/usr/bin/python3 "$HERE/walk_ab_probe.py" "$SECS" "$OUT/run" \
    --settle "$SETTLE" > "$OUT/metrics.txt" 2>&1
grep -E "Change state|FSM: Start" "$OUT/g1_ctrl.log" >> "$OUT/metrics.txt"
# The screenshot and CPU subshells are scheduled inside the probe's window
# (SHOT_AT and SHOT_AT-45 are both < SECS), so they have already finished;
# a short grace covers the second shot's tail rather than job-control %n,
# which is not reliable in a non-interactive shell.
sleep 8
cleanup

echo "=== metrics ==="; cat "$OUT/metrics.txt"
echo "=== cpu ===";     cat "$OUT/cpu.txt" 2>/dev/null
echo "=== overlay log ==="; wc -l "$OUT/overlay.jsonl" 2>/dev/null
echo "=== capture ===";     ls -la "$OUT/capture.bin" 2>/dev/null
ls -la "$OUT"/rviz_overlay_*.png 2>/dev/null
