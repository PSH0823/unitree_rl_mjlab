#!/bin/bash
# INTERACTIVE live view: MuJoCo + RViz on the real desktop, and they STAY UP.
#
#   run_live_w4_view.sh [scenario] [mode]        default: W4 estimated
#
# This is NOT walk_overlay_rviz_run.sh. That one is an evidence harness: it
# starts its own Xvfb on :77, forces DISPLAY at every child, and tears the whole
# stack down as soon as walk_ab_probe.py's timer expires. Both behaviours are
# wrong for watching the robot. Here:
#
#   * no Xvfb, ever — everything inherits DISPLAY (default :1, the real Xorg);
#   * nothing is torn down until YOU press Ctrl+C;
#   * the OpenCV dpcbf_visualizer is RE-ENABLED. walk_scenarios.py deliberately
#     flips `visualization.enabled` to false when it writes the shadow config
#     ("no place in a headless batch run"), which would silently suppress the
#     filter's own velocity-boundary window — the exact thing you want to see;
#   * RViz opens a copy of the committed layout with LivoxCloud turned ON, so
#     the raw Mid-360 cloud is visible without editing the tracked file.
#
# Startup order is load-bearing and is the one the working walking harness uses
# (runbook §6.1): g1_ctrl FIRST (it blocks in wait_for_connection() until
# rt/lowstate exists, so starting it first puts State_Passive live at sim t~0,
# ahead of the profile's chords), then the simulator, then a wait for the
# compiled mirror-model dump, then description -> sidecar -> perception -> RViz.
set +u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)          # ros2/
REPO=$(cd "$WS/.." && pwd)
SCEN=${1:-W4}
MODE=${2:-estimated}
TREE=${LIVE_TREE:-/tmp/sim_live_w4}
LOGS=${LIVE_LOGS:-/tmp/live_w4_logs}
: "${DISPLAY:=:1}"
export DISPLAY

# Guard: :77 / :1001 / :1002 are Xvfb or virtual displays on this machine.
# Launching there is how you end up with "it ran fine but I saw nothing".
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
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa

mkdir -p "$LOGS"

# Bracketed patterns throughout: an unbracketed `pkill -f component_container`
# matches this script's own command line and kills the invoking shell (§3.4).
cleanup() {
  echo; echo "[live] stopping..."
  pkill -f 'rviz[2]' 2>/dev/null
  pkill -f 'dpcbf_overla[y]' 2>/dev/null
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
  echo "[live] stopped."
  exit 0
}
trap cleanup INT TERM
cleanup_quiet() { trap - INT TERM; cleanup >/dev/null 2>&1; }
# Start from a clean slate: a stale container wins the next LoadNode race and
# the new one comes up empty, with no error (§3.4).
pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null
pkill -f 'sim_mjlidar_bridg[e]' 2>/dev/null
pkill -INT -f 'unitree_mujoc[o]' 2>/dev/null
pkill -f 'g1_ctr[l]' 2>/dev/null
sleep 2
pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null
pkill -KILL -f 'unitree_mujoc[o]' 2>/dev/null

# ---- shadow run-tree (§3.2); dpcbf/ stays byte-for-byte untouched (D3) ----
# The simulator resolves config.yaml relative to its own executable and
# dpcbf/config/dpcbf_config.yaml one level above that, so it cannot be
# reconfigured in place without editing tracked files.
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"

PROFILE="$TREE/walk_profile.txt"
cp "$REPO/ros2/src/g1_perception/g1_perception_bringup/config/walk_profile.txt" "$PROFILE"

/usr/bin/python3 "$HERE/walk_scenarios.py" "$SCEN" \
    "$REPO/dpcbf/config/dpcbf_config.yaml" "$TREE/dpcbf/config/dpcbf_config.yaml" || exit 1

# Undo walk_scenarios.py's headless choice: put the filter's own OpenCV
# top-down + velocity-boundary window back on. This is the ONLY view of the
# QP's actual inputs (runbook §4.4).
/usr/bin/python3 - "$TREE/dpcbf/config/dpcbf_config.yaml" <<'PY'
import sys
p = sys.argv[1]
head, sep, tail = open(p).read().partition('visualization:')
if sep:
    tail = tail.replace('enabled: false', 'enabled: true', 1)
    open(p, 'w').write(head + sep + tail)
    print('[live] dpcbf_visualizer re-enabled in the shadow config')
PY

# ---- RViz layout with the raw cloud on -----------------------------------
RVIZ_SRC="$WS/install/share/g1_perception_bringup/rviz/perception.rviz"
RVIZ_LIVE="$TREE/perception_live.rviz"
/usr/bin/python3 - "$RVIZ_SRC" "$RVIZ_LIVE" <<'PY'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src).read()
# Flip only the Enabled: line that belongs to the LivoxCloud display.
text = re.sub(r'(- Class: rviz_default_plugins/PointCloud2\n\s+Enabled: )false(\n\s+Name: LivoxCloud)',
              r'\1true\2', text)
open(dst, 'w').write(text)
print('[live] RViz layout: LivoxCloud enabled ->', dst)
PY

echo "[live] DISPLAY=$DISPLAY  scenario=$SCEN  mode=$MODE  tree=$TREE  logs=$LOGS"

# ---- 1. g1_ctrl FIRST -----------------------------------------------------
( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo \
    > "$LOGS/g1_ctrl.log" 2>&1 ) &
echo "[live] 1/7 g1_ctrl (waiting for rt/lowstate)"
sleep 3

# ---- 2. MuJoCo simulator (visible window on $DISPLAY) --------------------
( cd "$TREE/simulate/build" && env \
    UNITREE_DPCBF_MODE="$MODE" \
    UNITREE_DPCBF_FILTER_LOG="$LOGS/capture.bin" \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS="$PROFILE" \
    UNITREE_MUJOCO_BAND_LENGTH="${UNITREE_MUJOCO_BAND_LENGTH:-0.572}" \
    UNITREE_MUJOCO_BAND_RELEASE="${UNITREE_MUJOCO_BAND_RELEASE:-34,6}" \
    ./unitree_mujoco > "$LOGS/sim.log" 2>&1 ) &
echo "[live] 2/7 unitree_mujoco + OpenCV dpcbf_visualizer"

# ---- 3. wait for the compiled mirror dump + DDS --------------------------
# The sidecar loads /tmp/unitree_mujoco_mirror_model.xml, never the raw scene:
# the obstacle mocap bodies are added to the spec at runtime. Starting it early
# means a missing file, or worse a stale dump from a previous scene.
MIRROR=${UNITREE_MUJOCO_MIRROR_XML:-/tmp/unitree_mujoco_mirror_model.xml}
rm -f "$MIRROR" 2>/dev/null
echo -n "[live] 3/7 waiting for mirror dump"
for i in $(seq 1 40); do
  [ -s "$MIRROR" ] && break
  echo -n "."; sleep 1
done
if [ ! -s "$MIRROR" ]; then
  echo " FAILED"; echo "[live] no $MIRROR — see $LOGS/sim.log" >&2
  cleanup_quiet; exit 1
fi
echo " ok ($MIRROR)"
sleep 4                       # DDS discovery settle

# ---- 4-6. description -> sidecar -> perception ---------------------------
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true \
    > "$LOGS/desc.log" 2>&1 &
echo "[live] 4/7 description"
sleep 2
ros2 launch g1_perception_bringup source_sim.launch.py > "$LOGS/src.log" 2>&1 &
echo "[live] 5/7 sim Mid-360 sidecar"
sleep 3
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true \
    > "$LOGS/perc.log" 2>&1 &
echo "[live] 6/7 perception container"
sleep 6

# ---- 7. RViz + relays + overlay ------------------------------------------
ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true \
    overlay:=on \
    dpcbf_config:="$TREE/dpcbf/config/dpcbf_config.yaml" \
    rviz_config:="$RVIZ_LIVE" \
    > "$LOGS/viz.log" 2>&1 &
echo "[live] 7/7 RViz + 4 marker relays + dpcbf_overlay"

cat <<EOF

[live] Up. Two windows on $DISPLAY: the MuJoCo simulator and RViz
       (plus the OpenCV "DPCBF Top-Down and Velocity Boundaries" window).

       Timeline, in SIM seconds (walk_profile.txt):
         15.0/15.5  L2 then L2+up   -> FixStand
         21.0/21.5  R2 then R2+A    -> Velocity (policy running)
         34 -> 40   elastic band lowers over 6 s
         40.0       forward 0.20 + yaw 0.35 — the robot starts WALKING

       Logs: $LOGS/{g1_ctrl,sim,desc,src,perc,viz}.log
       Ctrl+C here stops everything.

EOF
wait
