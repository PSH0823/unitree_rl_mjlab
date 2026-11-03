#!/bin/bash
# INTERACTIVE live view of the 3D pipeline: same MuJoCo DPCBF experiment
# environment as run_live_w4_view.sh (same g1_ctrl → simulator → walk-profile
# startup, same W1-W4 obstacle fields), but perception is the 3-D chain
# (cloud → cloud_object_detector → autoware_multi_object_tracker) and RViz
# opens perception_3d.rviz: raw cloud + GT circles (green) + tracked convex
# prisms with velocity arrows. HUMBLE ONLY (the 3D stack).
#
#   run_live_3d_view.sh [scenario] [mode]        default: W2 oracle
#
# Defaults differ from the 2D script deliberately:
#   * W2 (6 obstacles crossing at 0.5-0.8 m/s), not W4 — watching how ONE
#     "person" is carved out of the cloud is the point; 90 obstacles is a
#     density stress test, not a viewing session. Pass W4 to get that anyway.
#   * mode=oracle, not estimated — nothing feeds /obstacles_safe in the 3D
#     pipeline yet (the TrackedObjects→Obstacles bridge is future work), so
#     estimated mode would starve the filter into its stale-hold ladder. In
#     oracle mode DPCBF walks the robot off GT while you watch perception.
#
# Everything else (shadow tree, display guards, startup order, cleanup) is
# the 2D script's, and the comments there are authoritative.
set +u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)          # ros2/
REPO=$(cd "$WS/.." && pwd)
SCEN=${1:-W2}
MODE=${2:-oracle}
TREE=${LIVE_TREE:-/tmp/sim_live_3d}
LOGS=${LIVE_LOGS:-/tmp/live_3d_logs}
: "${DISPLAY:=:1}"
export DISPLAY

case "$DISPLAY" in
  :77|:77.*|:1001|:1001.*|:1002|:1002.*)
    echo "REFUSING: DISPLAY=$DISPLAY is a virtual/Xvfb display." >&2
    echo "This launcher is for the real desktop. Use DISPLAY=:1." >&2
    exit 2 ;;
esac
if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  echo "REFUSING: cannot open DISPLAY=$DISPLAY" >&2; exit 2
fi

# ---- environment block (runbook §2.1) -------------------------------------
export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa

mkdir -p "$LOGS"

# Bracketed patterns: an unbracketed pkill matches this script's own command
# line and kills the invoking shell (§3.4).
cleanup() {
  echo; echo "[live3d] stopping..."
  pkill -f 'rviz[2]' 2>/dev/null
  pkill -f 'obstacles_marker_rela[y]' 2>/dev/null
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null
  pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null
  pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null
  pkill -f 'g1_ctr[l]' 2>/dev/null
  pkill -f 'robot_state_publishe[r]' 2>/dev/null
  pkill -f 'base_footprint_publishe[r]' 2>/dev/null
  sleep 2
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null
  pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null
  pkill -KILL -f 'g1_ctr[l]' 2>/dev/null
  echo "[live3d] stopped."
  exit 0
}
trap cleanup INT TERM
cleanup_quiet() { trap - INT TERM; cleanup >/dev/null 2>&1; }
# Clean slate: a stale container wins the next LoadNode race silently (§3.4).
pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null
pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null
pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null
pkill -f 'g1_ctr[l]' 2>/dev/null
sleep 2
pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null
pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null

# ---- shadow run-tree (§3.2); dpcbf/ stays byte-for-byte untouched (D3) ----
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
ln -sfn "$REPO/ros2" "$TREE/ros2"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"

PROFILE="$TREE/walk_profile.txt"
cp "$REPO/ros2/src/g1_perception/g1_perception_bringup/config/walk_profile.txt" "$PROFILE"

/usr/bin/python3 "$HERE/walk_scenarios.py" "$SCEN" \
    "$REPO/dpcbf/config/dpcbf_config.yaml" "$TREE/dpcbf/config/dpcbf_config.yaml" || exit 1

# The OpenCV dpcbf_visualizer window shows the QP's 2D inputs; in oracle mode
# with the 3D pipeline it is GT-only noise next to RViz, so it stays off
# (walk_scenarios.py already disabled it in the shadow config).

echo "[live3d] DISPLAY=$DISPLAY  scenario=$SCEN  mode=$MODE  tree=$TREE  logs=$LOGS"

# ---- 1. g1_ctrl FIRST -----------------------------------------------------
( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo \
    > "$LOGS/g1_ctrl.log" 2>&1 ) &
echo "[live3d] 1/7 g1_ctrl (waiting for rt/lowstate)"
sleep 3

# ---- 2. MuJoCo simulator (visible window on $DISPLAY) --------------------
( cd "$TREE/simulate/build" && env \
    UNITREE_DPCBF_MODE="$MODE" \
    UNITREE_DPCBF_FILTER_LOG="$LOGS/capture.bin" \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS="$PROFILE" \
    UNITREE_MUJOCO_BAND_LENGTH="${UNITREE_MUJOCO_BAND_LENGTH:-0.572}" \
    UNITREE_MUJOCO_BAND_RELEASE="${UNITREE_MUJOCO_BAND_RELEASE:-34,6}" \
    ./unitree_mujoco > "$LOGS/sim.log" 2>&1 ) &
echo "[live3d] 2/7 unitree_mujoco"

# ---- 3. wait for the compiled mirror dump + DDS --------------------------
MIRROR=${UNITREE_MUJOCO_MIRROR_XML:-/tmp/unitree_mujoco_mirror_model.xml}
rm -f "$MIRROR" 2>/dev/null
echo -n "[live3d] 3/7 waiting for mirror dump"
for i in $(seq 1 40); do
  [ -s "$MIRROR" ] && break
  echo -n "."; sleep 1
done
if [ ! -s "$MIRROR" ]; then
  echo " FAILED"; echo "[live3d] no $MIRROR — see $LOGS/sim.log" >&2
  cleanup_quiet; exit 1
fi
echo " ok ($MIRROR)"
sleep 4                       # DDS discovery settle

# ---- 4-6. description -> sidecar -> 3D perception ------------------------
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true \
    > "$LOGS/desc.log" 2>&1 &
echo "[live3d] 4/7 description"
sleep 2
ros2 launch g1_perception_bringup source_sim.launch.py > "$LOGS/src.log" 2>&1 &
echo "[live3d] 5/7 sim Mid-360 sidecar"
sleep 3
ros2 launch g1_perception_bringup perception_3d.launch.py use_sim_time:=true \
    > "$LOGS/perc.log" 2>&1 &
echo "[live3d] 6/7 3D perception container (detector + autoware tracker)"
sleep 6

# ---- 7. RViz + GT relay ---------------------------------------------------
ros2 launch g1_perception_bringup viz_3d.launch.py use_sim_time:=true \
    > "$LOGS/viz.log" 2>&1 &
echo "[live3d] 7/7 RViz (perception_3d.rviz) + GT marker relay"

cat <<EOF

[live3d] Up. Two windows on $DISPLAY: the MuJoCo simulator and RViz.

       RViz layers: LivoxCloud (raw), GT circles (green), Tracked3D —
       convex footprint prisms (green wireframe), velocity arrows (orange),
       track ids. View opens in 3-D Orbit; TopDown2D is a saved view.

       Timeline, in SIM seconds (walk_profile.txt):
         15.0/15.5  L2 then L2+up   -> FixStand
         21.0/21.5  R2 then R2+A    -> Velocity (policy running)
         34 -> 40   elastic band lowers over 6 s
         40.0       forward 0.20 + yaw 0.35 — the robot starts WALKING

       Logs: $LOGS/{g1_ctrl,sim,desc,src,perc,viz}.log
       Ctrl+C here stops everything.

EOF
wait
