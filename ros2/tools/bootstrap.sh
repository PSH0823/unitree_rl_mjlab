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

# The SYSTEM python must win over conda's: every ament_python node in this
# workspace runs under `/usr/bin/python3` and rclpy is only built for it.
export PATH=/usr/bin:$PATH
hash -r
PY=/usr/bin/python3
[ -x "$PY" ] || die "no $PY"
PYVER="$($PY -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
ok "system python $PYVER"

# ...but prepending /usr/bin is NOT enough for CMake. find_package(Python3)
# defaults to Python3_FIND_VIRTUALENV=FIRST, which consults $VIRTUAL_ENV /
# $CONDA_PREFIX *before* $PATH. A shell that once activated conda leaves
# VIRTUAL_ENV exported even after `conda deactivate` (CONDA_SHLVL=0), so
# ament_cmake picks up a python with no catkin_pkg and EVERY ament package
# dies at configure time with:
#     ModuleNotFoundError: No module named 'catkin_pkg'
# Unset the hints here, and pass Python3_EXECUTABLE explicitly below so the
# answer does not depend on the caller's environment at all.
for _v in VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME; do
  if [ -n "${!_v:-}" ]; then
    warn "unsetting $_v=${!_v} (it outranks PATH in find_package(Python3))"
    unset "$_v"
  fi
done
# Same story for PYTHONPATH: a conda/uv site-packages leaking in shadows the
# distro numpy that rclpy and the C++ PCL bindings were compiled against.
case "${PYTHONPATH:-}" in
  *conda*|*"/.local/share/uv/"*|*pyenv*)
    warn "dropping non-system entries from PYTHONPATH"
    PYTHONPATH="$(printf '%s' "$PYTHONPATH" | tr ':' '\n' \
      | grep -vE 'conda|/\.local/share/uv/|pyenv' | paste -sd: -)"
    export PYTHONPATH
    ;;
esac
# Decisive for every CMake package in the workspace and for the simulate build.
PYCMAKE="-DPython3_EXECUTABLE=$PY -DPYTHON_EXECUTABLE=$PY"

$PY -c 'import catkin_pkg' 2>/dev/null \
  || die "$PY cannot import catkin_pkg (apt install python3-catkin-pkg-modules)"

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

# Distro: whatever $ROS_DISTRO says, else the one actually installed. Humble
# is tried first so an existing setup keeps behaving exactly as before.
if [ -z "${ROS_DISTRO:-}" ] || [ ! -f "/opt/ros/${ROS_DISTRO:-}/setup.bash" ]; then
  for _d in humble jazzy kilted rolling; do
    [ -f "/opt/ros/$_d/setup.bash" ] && ROS_DISTRO="$_d" && break
  done
fi
[ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] \
  || die "no ROS 2 under /opt/ros (set ROS_DISTRO, or install ros-<distro>-ros-base)"
set +u; source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
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
  # Ubuntu 24.04's system python is PEP 668 "externally managed": a plain
  # `pip install` aborts with externally-managed-environment. Install into the
  # USER site instead (~/.local/lib/pythonX.Y/site-packages, which /usr/bin/
  # python3 imports) — --break-system-packages only lifts the PEP 668 refusal
  # and with --user nothing dpkg owns is touched.
  PIPFLAGS=""
  $PY -m pip install --help 2>/dev/null | grep -q -- --break-system-packages \
    && PIPFLAGS="--break-system-packages"
  if $PY -m pip install -q -U --user $PIPFLAGS "mujoco>=3.5"; then
    ok "mujoco $($PY -c 'import mujoco; print(mujoco.__version__)') installed"
    MUJOCO_OK=1
  else
    fail "mujoco>=3.5 install failed — sim lidar gates will not run"
    MUJOCO_OK=0
  fi
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
# A CMakeCache written by an earlier run that picked up conda's/uv's python
# keeps that interpreter forever — Python3_EXECUTABLE is cached, so the fix
# above would be silently ignored and the build fails exactly as before.
# Drop only the build dirs that are actually poisoned.
STALE=0
for _c in build/*/CMakeCache.txt; do
  [ -f "$_c" ] || continue
  _p="$(sed -n 's/^_\?Python3_EXECUTABLE:[A-Z]*=//p' "$_c" | head -1)"
  if [ -n "$_p" ] && [ "$_p" != "$PY" ]; then
    rm -rf "$(dirname "$_c")"
    STALE=$((STALE + 1))
  fi
done
[ "$STALE" -gt 0 ] && warn "purged $STALE build dir(s) cached against a non-system python"

# rmw_cyclonedds_cpp is BUILT HERE, so it cannot be selected as the RMW while
# building: CMake resolves RMW_IMPLEMENTATION at configure time and fails with
# "Could not find ROS middleware implementation 'rmw_cyclonedds_cpp'" on the
# very first package. It is exported for the run/test stages further down.
env -u RMW_IMPLEMENTATION colcon build --merge-install \
    --parallel-workers "$JOBS" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release $PYCMAKE \
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
                -DCMAKE_PREFIX_PATH="$WS/install" -DPython3_EXECUTABLE="$PY" >/dev/null \
    && make -j"$JOBS" >/dev/null ) \
    && ok "unitree_mujoco + t1_replay + ab_eval + fsm_button_probe" \
    || fail "simulate build failed (see $REPO/simulate/build_ros2)"
fi

# --------------------------------------------------------------------------
step "5/6  what can be tested here"
have() { [ -f "$WS/test_fixtures/$1/metadata.yaml" ]; }

# Gates that need no recorded data at all.
CTESTS='marker_relay|bringup_sim'
MISSING=()

# hw_config_check / hw_offline_gates need no BAG, but they do need the robot's
# network: they assert that the host IP in config/MID360_config.json is actually
# assigned to a local interface, which is true on the Jetson wired to the
# Mid-360 and false on any dev machine. Running them anyway reports two
# failures that say nothing about this build, so select them the same way the
# bag gates are selected — by whether the machine can actually prove them.
HOST_IP="$($PY - "$WS/src/g1_perception/g1_perception_bringup/config/MID360_config.json" <<'EOF' 2>/dev/null
import json, sys
try:
    cfg = json.load(open(sys.argv[1]))
    print(cfg['MID360']['host_net_info']['cmd_data_ip'])
except Exception:
    pass
EOF
)"
if [ -n "$HOST_IP" ] && ip -4 -o addr show 2>/dev/null | grep -qw "$HOST_IP"; then
  CTESTS="$CTESTS|hw_config_check|hw_offline_gates"
else
  MISSING+=("hw_config_check, hw_offline_gates (host is not on the Mid-360 network${HOST_IP:+ — no interface has $HOST_IP})")
fi

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
  if PYTHONPATH="$WS/install/lib/python$PYVER/site-packages:${PYTHONPATH:-}" \
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
