#!/bin/bash
# Reproducible hardware capture with provenance (Phase 5C).
#
# For the stages where the stack is ALREADY running and you want a bag out of
# a second shell — stage 3 (driver only), stage 7 (self-hit), stage 8/9
# (CropBox / LaserScan) — where `record:=on` inside the launch is not
# available. `g1_perception_hardware_only.launch.py record:=on` is the
# equivalent for a full-stack session.
#
#   hw_record.sh <session_dir> <stage> [duration_s] [topic ...]
#
# e.g.
#   hw_record.sh evidence/hardware/2026-08-10/s1 3 60 /livox/lidar /livox/imu
#   hw_record.sh evidence/hardware/2026-08-10/s1 12 120     # full chain
#
# It records the bag, writes session.json beside it, copies the loaded YAMLs,
# dumps the environment and the command log, and prints what is still missing
# from the record. It publishes nothing and cannot move the robot.
set -uo pipefail

DIR="${1:?usage: hw_record.sh <session_dir> <stage> [duration_s] [topics...]}"
STAGE="${2:?stage number, e.g. 3}"
DUR="${3:-60}"
shift 3 || true

DEFAULT_TOPICS=(/livox/lidar /livox/imu /odom /tf /tf_static
                /points_self_filtered /scan /raw_obstacles
                /tracked_obstacles /obstacles_safe /diagnostics)
TOPICS=("$@")
[ ${#TOPICS[@]} -eq 0 ] && TOPICS=("${DEFAULT_TOPICS[@]}")

mkdir -p "$DIR"
BAG="$DIR/stage${STAGE}_$(date +%H%M%S)"

echo "=== environment dump"
{
  echo "date        : $(date -Is)"
  echo "host/arch   : $(hostname) $(uname -m)"
  echo "kernel      : $(uname -r)"
  echo "commit      : $(git rev-parse HEAD 2>/dev/null || echo n/a)"
  echo "branch      : $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo n/a)"
  echo "dirty       : $(git status --porcelain 2>/dev/null | wc -l) file(s)"
  echo "ROS_DISTRO  : ${ROS_DISTRO:-unset}"
  echo "RMW         : ${RMW_IMPLEMENTATION:-unset}"
  echo "DOMAIN_ID   : ${ROS_DOMAIN_ID:-unset(0)}"
  echo "CYCLONEDDS  : ${CYCLONEDDS_URI:-unset}"
  echo "topics      : ${TOPICS[*]}"
  echo
  echo "--- ip -br addr"; ip -br addr
  echo "--- ip route";    ip route
  echo "--- df -h";       df -h "$DIR"
  echo "--- ros2 node list"; ros2 node list 2>&1
  echo "--- ros2 topic list"; ros2 topic list 2>&1
} | tee "$DIR/env_stage${STAGE}.txt"

echo
echo "=== recording ${DUR}s -> $BAG"
timeout "$((DUR + 15))" ros2 bag record -o "$BAG" \
        --include-unpublished-topics "${TOPICS[@]}" &
REC=$!
sleep "$DUR"
kill -INT "$REC" 2>/dev/null
wait "$REC" 2>/dev/null

echo
echo "=== bag info"
ros2 bag info "$BAG" 2>&1 | tee "$DIR/baginfo_stage${STAGE}.txt"
( cd "$(dirname "$BAG")" && md5sum "$(basename "$BAG")"/* 2>/dev/null ) \
    | tee "$DIR/md5_stage${STAGE}.txt"

echo
echo "=== metadata"
META="$(dirname "$0")/hw_session_metadata.py"
[ -f "$META" ] || META="$(ros2 pkg prefix g1_perception_bringup)/lib/g1_perception_bringup/hw_session_metadata.py"
/usr/bin/python3 "$META" --out "$BAG.session.json" --bag "$BAG" \
    --stage "$STAGE" --copy-configs \
    --from-json "$DIR/session_defaults.json" 2>/dev/null \
  || /usr/bin/python3 "$META" --out "$BAG.session.json" --bag "$BAG" \
       --stage "$STAGE" --copy-configs

cat <<EOF

=== capture complete
bag       : $BAG
metadata  : $BAG.session.json
env dump  : $DIR/env_stage${STAGE}.txt

Fill in the operator fields NOW (they are listed above if blank). Put the
values you will reuse all session — G1 variant, LiDAR serial and firmware,
operator name, network interfaces — into $DIR/session_defaults.json once and
every later capture in this directory picks them up automatically.
EOF
