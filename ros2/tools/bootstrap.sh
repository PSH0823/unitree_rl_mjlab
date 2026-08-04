#!/bin/bash
# One-command bring-up on a fresh machine: preflight -> externals -> build ->
# simulate -> test. Everything the operator runbook §2 describes in prose, in
# the order that actually works on a machine where nothing has been set up.
#
#   tools/bootstrap.sh                  # full: build + build simulate + test
#   tools/bootstrap.sh --skip-simulate  # colcon workspace only
#   tools/bootstrap.sh --skip-tests     # build only
#   tools/bootstrap.sh --jobs 8         # cap parallelism
#
# Bags are NOT built here (they are a recording session, not a build step):
# the test stage detects which fixtures exist and runs exactly the gates those
# fixtures support, so a fresh machine reports PASS on what it can prove
# instead of 30 failures on what it has no data for. Run
# tools/regen_fixtures.sh to record them, then re-run this script.
#
# Exit 0 = every stage that ran, passed.
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(dirname "$WS")"
SKIP_SIMULATE=0
SKIP_TESTS=0
JOBS="$(nproc)"

while [ $# -gt 0 ]; do
  case "$1" in
    --skip-simulate) SKIP_SIMULATE=1 ;;
    --skip-tests)    SKIP_TESTS=1 ;;
    --jobs)          JOBS="$2"; shift ;;
    -h|--help)       sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 64 ;;
  esac
  shift
done

FAILED=()
step()  { printf '\n\033[1m=== %s\033[0m\n' "$*"; }
ok()    { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
warn()  { printf '  \033[33mWARN\033[0m  %s\n' "$*"; }
fail()  { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; FAILED+=("$*"); }
die()   { fail "$*"; printf '\n\033[31maborted\033[0m\n'; exit 1; }

# --------------------------------------------------------------------------
step "1/6  preflight"

# System python 3.10 must win over conda's: every ament_python node in this
# workspace runs under `/usr/bin/python3` and rclpy is only built for it.
export PATH=/usr/bin:$PATH
hash -r
PY=/usr/bin/python3
[ -x "$PY" ] || die "no $PY"
PYVER="$($PY -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
[ "$PYVER" = "3.10" ] && ok "system python $PYVER" || warn "system python is $PYVER, ROS Humble expects 3.10"

# Stray mini-sims are the nastiest failure in this workspace: a leftover
# wall_state_source/scenario_state_source keeps publishing /clock at ITS sim
# time, the next run's clock starts at 0, every tf2 buffer sees "jump back in
# time" and clears itself, and the gates report 0 scans as if the chain were
# broken. They survive `pkill` on the wrong pattern and outlive the checkout
# they came from (one pair here had been running 23 h from a deleted tree).
STRAY="$(pgrep -af 'wall_state_source|scenario_state_source' | grep -v "$$" || true)"
if [ -n "$STRAY" ]; then
  warn "killing stray mini-sim processes (they poison /clock):"
  echo "$STRAY" | sed 's/^/        /'
  pkill -f 'wall_state_source|scenario_state_source'
  sleep 2
  ok "cleared"
fi

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found at /opt/ros/humble"
set +u; source /opt/ros/humble/setup.bash; set -u
ok "ROS $ROS_DISTRO sourced"

command -v colcon >/dev/null || die "colcon not on PATH (apt install python3-colcon-common-extensions)"
command -v vcs    >/dev/null || warn "vcstool not on PATH — needed only if src/external is missing"

# MuJoCo-LiDAR calls mj_multiRay(..., normal=...), which exists from mujoco
# 3.5 on. 3.3.x raises TypeError inside the bridge and the whole sim chain
# goes silent (0 scans) with no error at the launch level.
if $PY -c "import mujoco, sys; sys.exit(0 if 'normal' in mujoco.mj_multiRay.__doc__ else 1)" 2>/dev/null; then
  ok "mujoco $($PY -c 'import mujoco; print(mujoco.__version__)') (mj_multiRay has normal=)"
  MUJOCO_OK=1
else
  warn "mujoco too old or missing — installing >=3.5 for $PY"
  $PY -m pip install -q -U "mujoco>=3.5" \
    && ok "mujoco $($PY -c 'import mujoco; print(mujoco.__version__)') installed" \
    && MUJOCO_OK=1 \
    || { fail "mujoco>=3.5 install failed — sim lidar gates will not run"; MUJOCO_OK=0; }
fi

# --------------------------------------------------------------------------
step "2/6  external sources"
cd "$WS" || die "no workspace at $WS"
if [ -d src/external/cyclonedds ] && [ -d src/external/MuJoCo-LiDAR ]; then
  ok "src/external present ($(ls src/external | wc -l) checkouts)"
else
  command -v vcs >/dev/null || die "src/external missing and vcstool not installed (pip install vcstool)"
  ./setup_external.sh || die "setup_external.sh failed"
  ok "imported and patched"
fi

# --------------------------------------------------------------------------
step "3/6  colcon build"
# rmw_cyclonedds_cpp is BUILT HERE, so it cannot be selected as the RMW while
# building: CMake resolves RMW_IMPLEMENTATION at configure time and fails with
# "Could not find ROS middleware implementation 'rmw_cyclonedds_cpp'" on the
# very first package. It is exported for the run/test stages further down.
env -u RMW_IMPLEMENTATION colcon build --merge-install \
    --parallel-workers "$JOBS" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  || die "colcon build failed"
ok "$(ls install/share | wc -l) packages installed (merged prefix)"

set +u; source "$WS/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ok "workspace sourced, RMW=$RMW_IMPLEMENTATION"

# --------------------------------------------------------------------------
step "4/6  simulate (plain CMake, not colcon)"
if [ "$SKIP_SIMULATE" = 1 ]; then
  warn "skipped (--skip-simulate)"
else
  mkdir -p "$REPO/simulate/build_ros2"
  ( cd "$REPO/simulate/build_ros2" \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=ON \
                -DCMAKE_PREFIX_PATH="$WS/install" >/dev/null \
    && make -j"$JOBS" >/dev/null ) \
    && ok "unitree_mujoco + t1_replay + ab_eval + fsm_button_probe" \
    || fail "simulate build failed (see $REPO/simulate/build_ros2)"
fi

# --------------------------------------------------------------------------
step "5/6  what can be tested here"
have() { [ -f "$WS/test_fixtures/$1/metadata.yaml" ]; }

# Gates that need no recorded data at all.
CTESTS='marker_relay|hw_config_check|hw_offline_gates|bringup_sim'
MISSING=()
[ "$MUJOCO_OK" = 1 ] && CTESTS="$CTESTS|wall_accuracy" || MISSING+=("wall_accuracy (mujoco<3.5)")
if have s1_static_reference; then CTESTS="$CTESTS|projection_replay|hw_source_contract|dlio_wiring"
  else MISSING+=("projection_replay, hw_source_contract, dlio_wiring (no s1_static_reference bag)"); fi
if have s1_surveyed;          then CTESTS="$CTESTS|detection_static|t8_replay_determinism"
  else MISSING+=("detection_static (T4), t8_replay_determinism (no s1_surveyed bag)"); fi
if have s2_cross_05;          then CTESTS="$CTESTS|tracking_dynamic_05"
  else MISSING+=("tracking_dynamic_05 (T5) (no s2_cross_05 bag)"); fi
if have s2_cross_08;          then CTESTS="$CTESTS|tracking_dynamic_08"
  else MISSING+=("tracking_dynamic_08 (T5) (no s2_cross_08 bag)"); fi
ok "bringup gates selected: $CTESTS"
for m in ${MISSING+"${MISSING[@]}"}; do warn "not run: $m"; done

# --------------------------------------------------------------------------
step "6/6  colcon test"
if [ "$SKIP_TESTS" = 1 ]; then
  warn "skipped (--skip-tests)"
else
  PKGS_NOBAG="dpcbf_ros_adapter safety_obstacle_filter g1_perception_utils g1_description sim_mjlidar_bridge"
  # colcon test-result reads whatever XML is on disk, so a previous run's
  # results (including the ones for gates deselected below) would be reported
  # as this run's. Clear them first.
  for p in $PKGS_NOBAG g1_perception_bringup; do rm -rf "build/$p/test_results"; done
  # --merge-install is required on `test` too: without it colcon refuses the
  # merged layout, and the follow-up test-result then prints the PREVIOUS run.
  colcon test --merge-install --packages-select $PKGS_NOBAG >/dev/null 2>&1
  colcon test --merge-install --packages-select g1_perception_bringup \
      --ctest-args -R "$CTESTS" >/dev/null 2>&1
  # sim_mjlidar_bridge's unit tests import the installed package, which colcon
  # does not put on PYTHONPATH for an ament_python package in a MERGED prefix —
  # they collect as "0 tests" instead of failing, so run them explicitly.
  if PYTHONPATH="$WS/install/lib/python3.10/site-packages:${PYTHONPATH:-}" \
       $PY -m pytest -q "$WS/src/g1_perception/sim_mjlidar_bridge/test" >/tmp/mjlidar_pytest.log 2>&1; then
    ok "$(printf '%-24s %s' 'sim_mjlidar_bridge/test' "$(tail -1 /tmp/mjlidar_pytest.log)")"
  else
    fail "$(printf '%-24s %s' 'sim_mjlidar_bridge/test' "$(tail -1 /tmp/mjlidar_pytest.log)")"
  fi

  for p in $PKGS_NOBAG g1_perception_bringup; do
    line="$(colcon test-result --test-result-base "build/$p" 2>/dev/null | tail -1)"
    case "$line" in
      *"0 errors, 0 failures"*) ok "$(printf '%-24s %s' "$p" "$line")" ;;
      "")                       fail "$(printf '%-24s %s' "$p" 'no results — the package did not run')" ;;
      *)                        fail "$(printf '%-24s %s' "$p" "$line")" ;;
    esac
  done
fi

# --------------------------------------------------------------------------
printf '\n\033[1m=== summary\033[0m\n'
if [ ${#FAILED[@]} -eq 0 ]; then
  printf '  \033[32mall stages passed\033[0m\n'
  [ ${#MISSING[@]} -gt 0 ] && printf '  %d gate(s) not run for lack of fixtures — tools/regen_fixtures.sh\n' "${#MISSING[@]}"
  exit 0
fi
printf '  \033[31m%d failure(s)\033[0m\n' "${#FAILED[@]}"
for f in "${FAILED[@]}"; do printf '    - %s\n' "$f"; done
exit 1
