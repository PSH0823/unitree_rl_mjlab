#!/bin/bash
# Re-record the sim fixture bags that test_fixtures/README.md describes in
# prose. Bags are gitignored (30-100 MB each), so a fresh clone has none and
# every replay gate fails for lack of data, not for lack of correctness.
#
#   tools/regen_fixtures.sh                  # all five scenario bags
#   tools/regen_fixtures.sh s2_cross_05      # just one
#   tools/regen_fixtures.sh --force s3_swarm # overwrite an existing bag
#   tools/regen_fixtures.sh s1_static_reference   # needs a live simulator, see below
#
# The five scenario bags need NO simulator binary: scenario_state_source.py is
# a mini-sim (publishes /clock, /sim/mj_state, /sim/gt_obstacles from a scripted
# trajectory) and sim_mjlidar_bridge raycasts against the kinematic mirror. They
# are fully automatic here.
#
# s1_static_reference is the one exception — it is a capture of the REAL
# simulator (90 GT obstacles, robot in the elastic-band rig), so this script
# only drives the recorder and expects you to have `unitree_mujoco` already
# running; it says exactly what to set if it is not.
#
# Ordering matters and is the reason this is a script: the recorder must be up
# BEFORE the scenario starts (or t=0 is missing from the bag), and every run
# must leave no stray node behind (two /clock publishers make the next bag
# silently wrong).
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCEN_OUT=/tmp/scenarios_phase3
TOPICS="/livox/lidar /odom /tf /tf_static /sim/gt_obstacles /sim/mj_state /clock"
FORCE=0
TARGETS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --force) FORCE=1 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    -*) echo "unknown argument: $1" >&2; exit 64 ;;
    *) TARGETS+=("$1") ;;
  esac
  shift
done
[ ${#TARGETS[@]} -eq 0 ] && TARGETS=(s1_surveyed s2_cross_05 s2_cross_08 s3_swarm s4_occlusion)

info() { printf '\n\033[1m--- %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$*"; }
die()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; exit 1; }

[ -n "${ROS_DISTRO:-}" ] || die "source /opt/ros/humble/setup.bash and $WS/install/setup.bash first"
[ -f "$WS/install/setup.bash" ] || die "workspace not built — run tools/bootstrap.sh"
export PATH=/usr/bin:$PATH; hash -r
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# scenario name -> (json, mirror). The bag for the T4 scenario is called
# s1_surveyed but its json is s1_static — the names diverged in Phase 3.
json_for()   { case "$1" in
                 s1_surveyed)  echo "$SCEN_OUT/s1_static.json" ;;
                 s2_cross_05)  echo "$SCEN_OUT/s2_cross_05.json" ;;
                 s2_cross_08)  echo "$SCEN_OUT/s2_cross_08.json" ;;
                 s3_swarm)     echo "$SCEN_OUT/s3_swarm.json" ;;
                 s4_occlusion) echo "$SCEN_OUT/s4_occlusion.json" ;;
               esac; }
mirror_for() { case "$1" in
                 s3_swarm|s4_occlusion) echo "$SCEN_OUT/scenario_mirror_p4.xml" ;;
                 *)                     echo "$SCEN_OUT/scenario_mirror.xml" ;;
               esac; }

PIDS=()
cleanup() {
  for p in ${PIDS+"${PIDS[@]}"}; do kill -INT -"$p" 2>/dev/null; done
  sleep 2
  for p in ${PIDS+"${PIDS[@]}"}; do kill -KILL -"$p" 2>/dev/null; done
  PIDS=()
  # Belt and braces: the runbook's SIGINT+pkill discipline between runs.
  pkill -f 'component_container|sim_mjlidar_bridge|robot_state_publisher|scenario_state_source' 2>/dev/null
  sleep 1
}
trap 'echo; warn "interrupted — cleaning up"; cleanup; exit 130' INT TERM

spawn() { setsid "$@" >/dev/null 2>&1 & PIDS+=("$!"); }

wait_topic() {  # wait_topic <topic> <seconds>
  local t=$1 deadline=$((SECONDS + $2))
  while [ $SECONDS -lt $deadline ]; do
    ros2 topic list 2>/dev/null | grep -qx "$t" && return 0
    sleep 1
  done
  return 1
}

record_scenario() {  # record_scenario <name>
  local name=$1 json mirror out
  json="$(json_for "$name")"; mirror="$(mirror_for "$name")"
  out="$WS/test_fixtures/$name"

  [ -f "$json" ] || die "$json missing (scene generation failed?)"
  if [ -e "$out" ]; then
    [ "$FORCE" = 1 ] || { warn "$name exists — skipping (use --force)"; return 0; }
    rm -rf "$out"
  fi

  info "$name"
  spawn ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true
  spawn ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true
  spawn ros2 launch g1_perception_bringup source_sim.launch.py "mirror_model_path:=$mirror"
  sleep 5

  # Recorder first: the scenario's t=0 must be inside the bag.
  spawn ros2 bag record -o "$out" $TOPICS
  sleep 2
  ok "recorder up, running scenario"

  # Blocks until the scripted trajectory ends, then exits by itself.
  /usr/bin/python3 "$WS/src/g1_perception/g1_perception_bringup/test/scenario_state_source.py" \
      --scenario-json "$json"
  local rc=$?
  sleep 1
  cleanup
  [ $rc -eq 0 ] || { warn "scenario source exited $rc"; }

  if [ -f "$out/metadata.yaml" ]; then
    ok "$(du -sh "$out" | cut -f1)  $out"
    md5sum "$out"/*.db3 2>/dev/null | sed 's/^/       md5 /'
  else
    die "$name produced no bag — check that /livox/lidar was publishing"
  fi
}

record_s1_static_reference() {
  local out="$WS/test_fixtures/s1_static_reference"
  info "s1_static_reference (live simulator)"
  if [ -e "$out" ]; then
    [ "$FORCE" = 1 ] || { warn "exists — skipping (use --force)"; return 0; }
    rm -rf "$out"
  fi
  # The real simulator pins its topics to loopback; a recorder without the same
  # URI sees the names and receives nothing.
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  if ! wait_topic /sim/mj_state 5; then
    cat >&2 <<EOF
  the simulator is not publishing. In another terminal:

    # dpcbf/config/dpcbf_config.yaml : shape: cylinder, count: 90
    # simulate/config.yaml           : enable_elastic_band: 1   (S1 suspension rig)
    cd $(dirname "$WS")/simulate/build_ros2 && ./unitree_mujoco

  then re-run: tools/regen_fixtures.sh s1_static_reference
EOF
    exit 1
  fi
  spawn ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true
  spawn ros2 launch g1_perception_bringup source_sim.launch.py
  sleep 5
  spawn ros2 bag record -o "$out" $TOPICS
  ok "recording 29 s at realtime factor ~1 (do not start the perception container now)"
  sleep 29
  cleanup
  [ -f "$out/metadata.yaml" ] && ok "$(du -sh "$out" | cut -f1)  $out" || die "no bag produced"
}

# --------------------------------------------------------------------------
# A leftover mini-sim from an earlier run (or an earlier checkout) publishes
# /clock at its own sim time; the recording then carries two clocks and every
# consumer's tf2 buffer clears itself on the jump back. Clear before starting.
if pgrep -f 'wall_state_source|scenario_state_source' >/dev/null; then
  warn "stray mini-sim processes found — clearing"
  pkill -f 'wall_state_source|scenario_state_source'; sleep 2
fi

info "scenario scenes -> $SCEN_OUT"
/usr/bin/python3 "$WS/test_fixtures/scenarios/make_scenario_scene.py" --out "$SCEN_OUT" \
  || die "make_scenario_scene.py failed (needs python mujoco)"

for t in "${TARGETS[@]}"; do
  case "$t" in
    s1_static_reference) record_s1_static_reference ;;
    s1_surveyed|s2_cross_05|s2_cross_08|s3_swarm|s4_occlusion) record_scenario "$t" ;;
    *) die "unknown fixture: $t" ;;
  esac
done

info "done"
ls -d "$WS"/test_fixtures/*/ | sed 's/^/  /'
echo
echo "  now: tools/bootstrap.sh --skip-simulate   (it picks up whichever bags exist)"
