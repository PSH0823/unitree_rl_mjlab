#!/bin/bash
# G1 hardware preflight — Phase 5C, stage 1. READ-ONLY.
#
# Run this on the machine that will run the perception stack, before ANY
# launch file, with the robot powered but not commanded. It starts no node,
# opens no LiDAR socket, publishes no topic and cannot move the robot: every
# check below is `ip`, `ping`, `test -f`, a checksum, or a python script that
# only reads files.
#
#   g1_hw_preflight.sh                 # full preflight
#   g1_hw_preflight.sh --lidar-ip X    # override the IP taken from the JSON
#   g1_hw_preflight.sh --quiet         # summary only
#
# Exit 0 = every hard gate passed. Exit 1 = at least one HARD FAIL: do not
# launch anything. Exit 2 = placeholder network configuration (the expected
# state on a dev machine, a hard stop on the robot).
#
# Every FAIL line names the stage of the first-experiment runbook it blocks,
# so a failure here maps directly onto "which paragraph do I go read".
set -uo pipefail

QUIET=0
LIDAR_IP_OVERRIDE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --quiet) QUIET=1 ;;
    --lidar-ip) LIDAR_IP_OVERRIDE="$2"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 64 ;;
  esac
  shift
done

FAILS=0
WARNS=0
PLACEHOLDER=0
say()  { [ "$QUIET" = 1 ] || printf '\n=== %s\n' "$*"; }
info() { [ "$QUIET" = 1 ] || printf '    %s\n' "$*"; }
ok()   { [ "$QUIET" = 1 ] || printf '    ok   %s\n' "$*"; }
warn() { printf '    WARN %s\n' "$*"; WARNS=$((WARNS+1)); }
fail() { printf '    FAIL %s\n' "$*"; FAILS=$((FAILS+1)); }

# --- 0. where are we ------------------------------------------------------
say "0. target machine"
info "host      : $(hostname)"
info "arch      : $(uname -m)"
info "kernel    : $(uname -r)"
if [ -r /etc/os-release ]; then
  info "os        : $(. /etc/os-release; echo "$PRETTY_NAME")"
else
  warn "no /etc/os-release — cannot identify the distribution"
fi
info "cores     : $(nproc 2>/dev/null || echo '?')"
info "memory    : $(free -h 2>/dev/null | awk 'NR==2{print $2" total, "$7" available"}')"
case "$(uname -m)" in
  x86_64|aarch64) ok "architecture is one of the two this workspace targets" ;;
  *) fail "unrecognised architecture $(uname -m) — no build has been attempted here (blocks stage 1)" ;;
esac

# --- 1. ROS environment ---------------------------------------------------
say "1. ROS 2 environment"
if [ -z "${ROS_DISTRO:-}" ]; then
  fail "ROS_DISTRO unset — /opt/ros/*/setup.bash was never sourced (blocks every stage)"
else
  ok "ROS_DISTRO=$ROS_DISTRO"
  [ "$ROS_DISTRO" = humble ] || warn "this workspace is pinned against humble, not $ROS_DISTRO"
fi
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  fail "AMENT_PREFIX_PATH unset — the colcon workspace is not sourced. Run: source <ws>/install/setup.bash (blocks every stage)"
else
  case "$AMENT_PREFIX_PATH" in
    *g1_perception_bringup*|*install*) ok "a colcon prefix is on AMENT_PREFIX_PATH" ;;
    *) warn "AMENT_PREFIX_PATH has no obvious workspace install prefix" ;;
  esac
fi
# Middleware. §12.2 asked for rmw_cyclonedds_cpp because of Unitree SDK2
# coexistence (T10): ONE process linking both unitree_sdk2 and rclcpp must not
# load two CycloneDDS copies. A perception-only session has no such process —
# nothing here links unitree_sdk2 — so rmw_fastrtps_cpp is equally valid and is
# what the Computer 2 <-> Computer 3 link uses (it is the stock middleware of
# both Foxy and Humble, so neither machine has to build or apt-install an rmw).
# Both are accepted; anything else, or unset, is a hard fail, because the two
# machines silently running different vendors looks exactly like a dead network.
RMW="${RMW_IMPLEMENTATION:-<unset>}"
case "$RMW" in
  rmw_cyclonedds_cpp) ok "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ;;
  rmw_fastrtps_cpp)   ok "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ;;
  *) fail "RMW_IMPLEMENTATION=$RMW — expected rmw_fastrtps_cpp (the Fast DDS link, see net_env.sh) or rmw_cyclonedds_cpp (§12.2). The OTHER computer must use the same one (blocks stage 2)" ;;
esac
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset, i.e. 0>}"
[ -n "${ROS_DOMAIN_ID:-}" ] || warn "ROS_DOMAIN_ID unset — it defaults to 0, which must MATCH the other computer (and, once the control seam lands, the robot's SDK2 domain). Set it explicitly so the session log records it"
info "use_sim_time is FALSE on every hardware node (asserted in the launch files, not inherited)"

# The loopback trap, Fast DDS edition: ROS_LOCALHOST_ONLY=1 leaves every topic
# name looking perfectly normal while no byte can reach Computer 3.
if [ "${ROS_LOCALHOST_ONLY:-0}" = 1 ]; then
  fail "ROS_LOCALHOST_ONLY=1 — traffic cannot leave this machine, and the topic list still looks correct (blocks stage 2)"
else
  ok "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
fi

# Fast DDS profile file, when one is in use. An unreadable path is the quiet
# failure here: Fast DDS ignores it without a word and falls back to defaults,
# so a static-peer setup silently becomes a multicast one.
if [ "$RMW" = rmw_fastrtps_cpp ] && [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
  if [ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    info "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
    PEER=$(grep -oP '<address>\K[^<]+' "$FASTRTPS_DEFAULT_PROFILES_FILE" 2>/dev/null | head -1)
    [ -n "$PEER" ] && ok "unicast discovery peer: $PEER"
  else
    fail "FASTRTPS_DEFAULT_PROFILES_FILE points at $FASTRTPS_DEFAULT_PROFILES_FILE which is not readable — Fast DDS ignores it SILENTLY (blocks stage 2)"
  fi
fi

# CycloneDDS interface pin. `lo` here is the Phase-2 trap: topics are visible,
# no data ever crosses the wire to another machine. Only meaningful when
# Cyclone is the selected middleware.
URI="${CYCLONEDDS_URI:-}"
if [ "$RMW" != rmw_cyclonedds_cpp ]; then
  [ -n "$URI" ] && warn "CYCLONEDDS_URI is set but the middleware is $RMW — it has no effect; unset it so later diagnosis is unambiguous"
elif [ -z "$URI" ]; then
  warn "CYCLONEDDS_URI unset — Cyclone will pick an interface itself. Pin it to the robot's perception NIC (§12.2)"
else
  info "CYCLONEDDS_URI=$URI"
  XMLPATH="${URI#file://}"
  if [ -r "$XMLPATH" ]; then
    IFACE=$(grep -oP 'NetworkInterfaceAddress[^>]*>\K[^<]+' "$XMLPATH" 2>/dev/null | head -1)
    [ -z "$IFACE" ] && IFACE=$(grep -oP '<NetworkInterface[^>]*name="\K[^"]+' "$XMLPATH" 2>/dev/null | head -1)
    if [ -n "$IFACE" ]; then
      info "cyclone interface: $IFACE"
      if [ "$IFACE" = lo ] || [ "$IFACE" = 127.0.0.1 ]; then
        fail "CycloneDDS is pinned to loopback — nothing will reach another machine, and topic names will still look correct (blocks stage 2)"
      else
        ok "cyclone interface is not loopback"
      fi
    else
      warn "could not read a NetworkInterface out of $XMLPATH"
    fi
  else
    fail "CYCLONEDDS_URI points at $XMLPATH which is not readable (blocks stage 2)"
  fi
fi

# --- 2. executables the hardware path needs -------------------------------
say "2. required executables"
need_exec() { # pkg exe
  if ros2 pkg executables "$1" 2>/dev/null | grep -qx "$1 $2"; then
    ok "$1/$2"
  else
    fail "$1/$2 not installed — the workspace build is incomplete (blocks the stage that runs it)"
  fi
}
if command -v ros2 >/dev/null 2>&1; then
  need_exec livox_ros_driver2 livox_ros_driver2_node
  need_exec direct_lidar_inertial_odometry dlio_odom_node
  need_exec g1_perception_utils base_footprint_publisher
  need_exec robot_state_publisher robot_state_publisher
  need_exec rclcpp_components component_container
  for lib in libpcl_ros_filters.so; do
    if find "${AMENT_PREFIX_PATH%%:*}" -name "$lib" 2>/dev/null | grep -q .; then
      ok "pcl_ros filter components present ($lib)"
    else
      warn "$lib not found under the first AMENT prefix — CropBox may fail to load"
    fi
  done
else
  fail "no ros2 CLI on PATH (blocks every stage)"
fi

# --- 3. package files -----------------------------------------------------
say "3. installed package files"
SHARE=""
if command -v ros2 >/dev/null 2>&1; then
  SHARE=$(ros2 pkg prefix --share g1_perception_bringup 2>/dev/null || true)
fi
if [ -z "$SHARE" ] || [ ! -d "$SHARE" ]; then
  fail "g1_perception_bringup share directory not found — nothing to launch (blocks every stage)"
  SHARE=""
else
  ok "share: $SHARE"
  for f in config/MID360_config.json config/livox_driver.yaml config/dlio.yaml \
           config/cropbox_self_filter.yaml config/pointcloud_to_laserscan.yaml \
           config/obstacle_detector.yaml config/safety_obstacle_filter.yaml \
           launch/source_hw.launch.py launch/perception.launch.py \
           launch/description.launch.py \
           launch/g1_perception_hardware_only.launch.py \
           rviz/perception.rviz; do
    if [ -f "$SHARE/$f" ]; then ok "$f"; else fail "$f missing from the install prefix (rebuild g1_perception_bringup)"; fi
  done
fi
DESC=$(ros2 pkg prefix --share g1_description 2>/dev/null || true)
if [ -n "$DESC" ] && [ -f "$DESC/urdf/g1_mid360.xacro" ]; then
  ok "g1_description/urdf/g1_mid360.xacro (the extrinsic source of truth)"
else
  fail "g1_mid360.xacro not installed — no robot TF (blocks stage 4)"
fi

# --- 4. source vs installed ----------------------------------------------
say "4. source vs installed configuration"
HERE="$(cd "$(dirname "$0")" && pwd)"
CFGDIFF=""
for c in "$HERE/config_diff.py" "$(dirname "$HERE")/scripts/config_diff.py"; do
  [ -f "$c" ] && CFGDIFF="$c" && break
done
if [ -n "$CFGDIFF" ]; then
  OUT=$(/usr/bin/python3 "$CFGDIFF" --quiet 2>&1); RC=$?
  case $RC in
    0) ok "installed launch/config/rviz match the source tree" ;;
    3) info "install-only machine: no source tree to compare" ;;
    *) fail "installed configuration differs from source — you would run the OLD numbers. Rebuild g1_perception_bringup. Detail: $(echo "$OUT" | tail -3 | tr '\n' ' ')" ;;
  esac
else
  warn "config_diff.py not found next to this script"
fi

# --- 5. Mid-360 network ---------------------------------------------------
say "5. Mid-360 network configuration"
JSON="${SHARE:+$SHARE/config/MID360_config.json}"
[ -f "${JSON:-}" ] || JSON="$(dirname "$HERE")/config/MID360_config.json"
HWCHECK=""
for c in "$(dirname "$HERE")/test/hw_config_check.py" \
         "$(ros2 pkg prefix g1_perception_bringup 2>/dev/null)/lib/g1_perception_bringup/hw_config_check.py"; do
  [ -f "$c" ] && HWCHECK="$c" && break
done
if [ -n "$HWCHECK" ] && [ -f "$JSON" ]; then
  OUT=$(/usr/bin/python3 "$HWCHECK" "$JSON" 2>&1); RC=$?
  [ "$QUIET" = 1 ] || echo "$OUT" | sed 's/^/    /'
  case $RC in
    0) ok "hw_config_check: PASS" ;;
    2) PLACEHOLDER=1
       fail "PLACEHOLDER NETWORK CONFIGURATION — MID360_config.json still carries the upstream sample IPs. The driver will bind nothing, will NOT exit, and will create no /livox topic at all. Fill in this robot's addresses first (blocks stage 3)" ;;
    *) fail "hw_config_check failed (blocks stage 3)" ;;
  esac
else
  fail "hw_config_check.py or MID360_config.json not found (blocks stage 3)"
fi

LIDAR_IP="$LIDAR_IP_OVERRIDE"
if [ -z "$LIDAR_IP" ] && [ -f "${JSON:-}" ]; then
  LIDAR_IP=$(/usr/bin/python3 -c "
import json,sys
c=json.load(open(sys.argv[1]))
print(c['lidar_configs'][0]['ip'])" "$JSON" 2>/dev/null || true)
fi
say "6. reachability"
info "interfaces:"
ip -br addr 2>/dev/null | sed 's/^/      /'
info "routes:"
ip route 2>/dev/null | sed 's/^/      /'
if [ -n "$LIDAR_IP" ]; then
  info "lidar ip  : $LIDAR_IP"
  RT=$(ip route get "$LIDAR_IP" 2>/dev/null | head -1)
  if [ -n "$RT" ]; then
    info "route     : $RT"
    case "$RT" in
      *" dev lo "*) fail "the route to the LiDAR goes over loopback (blocks stage 3)" ;;
      *) ok "a non-loopback route to the LiDAR exists" ;;
    esac
  else
    fail "no route to $LIDAR_IP (blocks stage 3)"
  fi
  if [ "$PLACEHOLDER" = 1 ]; then
    info "ping skipped: the IP is a placeholder, not this robot's LiDAR"
  elif ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
    ok "LiDAR answers ICMP"
  else
    warn "LiDAR $LIDAR_IP does not answer ping. Not conclusive — some units are configured not to reply — but combined with 'no /livox topic' it means the network, not the sensor"
  fi
else
  warn "could not determine the LiDAR IP"
fi
# Livox uses UDP unicast to the host_net_info ports; multicast is not
# required, but a NIC with MULTICAST off also tends to have other surprises.
for dev in $(ip -br link 2>/dev/null | awk '$2=="UP"{print $1}'); do
  [ "$dev" = lo ] && continue
  if ip link show "$dev" 2>/dev/null | grep -q MULTICAST; then
    ok "$dev is UP with MULTICAST"
  else
    warn "$dev is UP without MULTICAST (DDS discovery may need unicast configuration)"
  fi
done

# --- 7. extrinsic guard ---------------------------------------------------
say "7. extrinsic guard (xacro <-> dlio.yaml)"
GUARD=""
for c in "$(dirname "$(dirname "$HERE")")/g1_description/test/t7_hw_extrinsic_guard.py" \
         "$(ros2 pkg prefix g1_description 2>/dev/null)/lib/g1_description/t7_hw_extrinsic_guard.py"; do
  [ -f "$c" ] && GUARD="$c" && break
done
if [ -n "$GUARD" ]; then
  OUT=$(/usr/bin/python3 "$GUARD" 2>&1); RC=$?
  [ "$QUIET" = 1 ] || echo "$OUT" | sed 's/^/    /'
  if [ $RC -eq 0 ]; then
    ok "dlio.yaml extrinsics are derived from the xacro and agree with it"
  else
    fail "DLIO extrinsics disagree with g1_mid360.xacro. DLIO has no TF listener: these parameters ARE its definition of base_link, so this silently moves every obstacle in odom (blocks stage 5)"
  fi
else
  warn "t7_hw_extrinsic_guard.py not found"
fi

# --- 8. storage -----------------------------------------------------------
say "8. storage for recording"
BAGDIR="${BAG_DIR:-$PWD}"
AVAIL_KB=$(df -Pk "$BAGDIR" 2>/dev/null | awk 'NR==2{print $4}')
if [ -n "${AVAIL_KB:-}" ]; then
  AVAIL_GB=$((AVAIL_KB/1024/1024))
  info "free at $BAGDIR: ${AVAIL_GB} GB"
  # Raw 10 Hz Mid-360 clouds run ~3 MB/s; 20 GB is ~1.8 h of raw capture.
  if [ "$AVAIL_GB" -lt 20 ]; then
    fail "less than 20 GB free — raw cloud capture runs ~3 MB/s and a session fills this (blocks recording)"
  else
    ok "at least 20 GB free"
  fi
else
  warn "could not read free space at $BAGDIR"
fi

# --- 9. summary -----------------------------------------------------------
printf '\n=== preflight summary\n'
printf '    hard failures : %d\n' "$FAILS"
printf '    warnings      : %d\n' "$WARNS"
if [ "$PLACEHOLDER" = 1 ]; then
  printf '\nPLACEHOLDER NETWORK CONFIGURATION — STOP.\n'
  printf 'MID360_config.json still holds upstream sample addresses. On the robot this\n'
  printf 'is a hard stop; on a dev machine it is the expected answer.\n'
  exit 2
fi
if [ "$FAILS" -gt 0 ]; then
  printf '\nPREFLIGHT FAILED — do not launch. Fix the FAIL lines above; each names\n'
  printf 'the experiment stage it blocks (doc/g1_first_perception_experiment.md).\n'
  exit 1
fi
printf '\nPREFLIGHT PASSED. This clears the checks a stationary machine can make.\n'
printf 'It does NOT verify the LiDAR produces data, the extrinsic matches the\n'
printf 'physical mount, or that odometry is sane — those are stages 3, 4 and 5.\n'
exit 0
