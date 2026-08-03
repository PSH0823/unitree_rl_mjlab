#!/bin/bash
# One-command closed-loop WALKING A/B (§17.3 collision rate + min-clearance).
#
#   walk_ab_run.sh [outdir] [scenarios...]     default: W1 W2 W3 W4
#
# The Phase-4 harness (phase4_ab_run.sh) replays fixture bags against a robot
# pinned at (0,0,0): it scores command tracking and nothing else, because a
# robot that never moves has no collision rate and no clearance. This harness
# runs the real thing — g1_ctrl's policy walking under a scripted command
# profile, DPCBF in the 1 kHz seam, the full perception stack live — once per
# DPCBF mode on an identical seeded obstacle field, and measures what the
# project has never measured.
#
# Robot bring-up is the operator procedure, scripted (see ros2/README.md):
# elastic band holds the robot, L2+up -> FixStand, R2+A -> RLBase, band
# LOWERED over a ramp, then the command profile runs. Chords are STAGGERED —
# the receiver-side Axis low-pass needs ~23 ticks to cross its 0.5 threshold,
# so a simultaneous "L2,up" never fires the transition (workstream B finding).
#
# Prereqs: workspace built & sourced; simulate/build_ros2 built with
# -DUNITREE_MUJOCO_WITH_ROS2=ON; deploy/robots/g1/build/g1_ctrl built; an X
# display (Xvfb is fine) for the simulator's GL context.
set +u   # ROS setup.bash trips over unbound AMENT_* variables
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)          # ros2/
REPO=$(cd "$WS/.." && pwd)
OUT=${1:-/tmp/walk_ab}; shift || true
SCENARIOS=${*:-"W1 W2 W3 W4"}
TREE=${WALK_TREE:-/tmp/sim_walk_ab}
SECS=${WALK_SECS:-110}
SETTLE=${WALK_SETTLE:-40}        # sim s: spawn + FixStand + band lowering
mkdir -p "$OUT"

export PATH=/usr/bin:$PATH; hash -r
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# ---- run tree -------------------------------------------------------------
# dpcbf/ is byte-for-byte untouched in the repo, so the per-scenario obstacle
# fields live as COPIES in a scratch tree; the simulator resolves its config
# relative to its own executable (proj_dir/../dpcbf/config/dpcbf_config.yaml).
rm -rf "$TREE"; mkdir -p "$TREE/simulate/build" "$TREE/dpcbf/config"
ln -sfn "$REPO/src" "$TREE/src"
cp "$REPO/simulate/build_ros2/unitree_mujoco" "$TREE/simulate/build/"
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^print_scene_information: .*/print_scene_information: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    "$REPO/simulate/config.yaml" > "$TREE/simulate/config.yaml"

# ---- scripted bring-up + command profile ---------------------------------
PROFILE="$OUT/walk_profile.txt"
cp "$HERE/../config/walk_profile.txt" "$PROFILE"

cleanup() {
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

run_one() {  # scenario mode
  local scen=$1 mode=$2 tag="$1_$2"
  echo "=== $tag ==="
  cleanup
  /usr/bin/python3 "$HERE/walk_scenarios.py" "$scen" \
      "$REPO/dpcbf/config/dpcbf_config.yaml" "$TREE/dpcbf/config/dpcbf_config.yaml" \
      > "$OUT/$tag.scenario.txt" || return 1

  # g1_ctrl FIRST. It blocks in wait_for_connection() until rt/lowstate
  # appears, so starting it before the simulator puts State_Passive live at
  # sim t~0 — deterministically ahead of the profile's chords. Started after
  # the simulator it races the profile, and losing that race presents as a
  # silent "the robot never stood up" (one smoke run lost exactly this way).
  ( cd "$REPO/deploy/robots/g1/build" && stdbuf -oL -eL ./g1_ctrl --network lo \
      > "$OUT/$tag.g1_ctrl.log" 2>&1 ) &
  sleep 3

  ( cd "$TREE/simulate/build" && env LIBGL_ALWAYS_SOFTWARE=1 \
      __GLX_VENDOR_LIBRARY_NAME=mesa DISPLAY=${DISPLAY:-:1} \
      UNITREE_DPCBF_MODE=$mode \
      UNITREE_MUJOCO_SCRIPTED_COMMANDS="$PROFILE" \
      UNITREE_MUJOCO_BAND_LENGTH=${WALK_BAND_LENGTH:-0.572} \
      UNITREE_MUJOCO_BAND_RELEASE=${WALK_BAND_RELEASE:-24,4} \
      ./unitree_mujoco > "$OUT/$tag.sim.log" 2>&1 ) &
  sleep 6

  ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true \
      > "$OUT/$tag.desc.log" 2>&1 &
  ros2 launch g1_perception_bringup source_sim.launch.py \
      > "$OUT/$tag.src.log" 2>&1 &
  ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true \
      > "$OUT/$tag.perc.log" 2>&1 &

  /usr/bin/python3 "$HERE/walk_ab_probe.py" "$SECS" "$OUT/$tag" \
      --settle "$SETTLE" > "$OUT/$tag.metrics.txt" 2>&1
  grep -E "Change state|FSM: Start" "$OUT/$tag.g1_ctrl.log" >> "$OUT/$tag.metrics.txt"
  cleanup
  cat "$OUT/$tag.metrics.txt"
}

for s in $SCENARIOS; do
  for m in ${WALK_MODES:-oracle estimated}; do
    run_one "$s" "$m"
  done
done

/usr/bin/python3 "$HERE/walk_ab_report.py" "$OUT" | tee "$OUT/walk_ab_report.txt"
