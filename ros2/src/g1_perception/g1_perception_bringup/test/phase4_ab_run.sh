#!/bin/bash
# One-command §17.3 offline paired-filter A/B over the S1–S4 fixtures
# (Phase 4; Phases 5/6 re-run this as regression).
#
# Prereqs: workspace built & sourced (system python first, R-11);
#          simulate/build_ros2 built with -DUNITREE_MUJOCO_WITH_ROS2=ON
#          (provides ab_eval); fixture bags present in ros2/test_fixtures.
#
# Pipeline per scenario: fixture bag → perception container replay →
# gt/tracked/safe JSONL dump → oracle/estimated binary streams → ab_eval
# (two DpcbfSafetyFilter arms at 1 kHz) → metrics. Also re-runs the §9.6
# containment sweep on the same dumps.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)          # ros2/
REPO=$(cd "$WS/.." && pwd)
OUT=${1:-/tmp/phase4_ab_rerun}
SCEN=/tmp/scenarios_phase3
AB_EVAL=$REPO/simulate/build_ros2/ab_eval
mkdir -p "$OUT"

[ -x "$AB_EVAL" ] || { echo "ab_eval missing: $AB_EVAL"; exit 77; }
/usr/bin/python3 "$WS/test_fixtures/scenarios/make_scenario_scene.py" --out $SCEN

replay() {  # bag out_jsonl secs
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null; sleep 1
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null; sleep 1
  /usr/bin/python3 "$HERE/phase4_obstacles_dump.py" "$3" "$2" &
  local probe=$!
  ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true \
      > /dev/null 2>&1 &
  local perc=$!
  sleep 6
  ros2 bag play "$1" > /dev/null 2>&1
  wait $probe
  kill -INT $perc 2>/dev/null; sleep 2
  pkill -INT -f 'rclcpp_components/component_containe[r]' 2>/dev/null; sleep 1
  pkill -KILL -f 'rclcpp_components/component_containe[r]' 2>/dev/null
}

declare -A SECS=( [s1_static]=45 [s2_cross_05]=38 [s2_cross_08]=28
                  [s3_swarm]=48 [s4_occlusion]=36 )
declare -A BAGS=( [s1_static]=s1_surveyed [s2_cross_05]=s2_cross_05
                  [s2_cross_08]=s2_cross_08 [s3_swarm]=s3_swarm
                  [s4_occlusion]=s4_occlusion )

METRIC_ARGS=(); CONT_ARGS=()
for s in s1_static s2_cross_05 s2_cross_08 s3_swarm s4_occlusion; do
  bag=$WS/test_fixtures/${BAGS[$s]}
  [ -d "$bag" ] || { echo "SKIP $s: no fixture $bag"; continue; }
  replay "$bag" "$OUT/$s.jsonl" "${SECS[$s]}"
  /usr/bin/python3 "$HERE/phase4_ab_export.py" $SCEN/$s.json "$OUT/$s.jsonl" "$OUT/$s"
  "$AB_EVAL" --oracle "$OUT/${s}_oracle.bin" --estimated "$OUT/${s}_estimated.bin" \
      --config "$REPO/dpcbf/config/dpcbf_config.yaml" \
      --profile "$HERE/../config/ab_profile.txt" \
      --timestep 0.001 --robot "0 0 0" --out "$OUT/$s.csv"
  METRIC_ARGS+=("$s=$OUT/$s.csv")
  CONT_ARGS+=("$s=$OUT/$s.jsonl")
done
/usr/bin/python3 "$HERE/phase4_ab_metrics.py" "${METRIC_ARGS[@]}" | tee "$OUT/ab_metrics.txt"
/usr/bin/python3 "$HERE/phase4_containment.py" "${CONT_ARGS[@]}" | tee "$OUT/containment.txt"
