#!/bin/bash
# Computer 2 <-> Computer 3 Fast DDS link check. READ-ONLY: starts no node of
# the perception stack, publishes nothing a robot could act on.
#
#   g1_link_check.sh            # audit THIS shell's environment (both machines)
#   g1_link_check.sh recv       # multicast probe, receiving end  (run FIRST)
#   g1_link_check.sh send       # multicast probe, sending end
#   g1_link_check.sh topics     # what this machine can actually see, no daemon
#
# The usual order on experiment day:
#   1. both machines:  g1_link_check.sh
#   2. Computer 3:     g1_link_check.sh recv     <- leave it running
#      Computer 2:     g1_link_check.sh send
#      -> datagram arrives  = multicast works, use the default mode
#      -> nothing arrives   = `source net_env.sh peers` on BOTH machines
#   3. Computer 3:     g1_link_check.sh topics   (with the stack up on C2)
#
# Exit 0 = every hard check passed.
set -uo pipefail

FAILS=0
ok()   { printf '    ok   %s\n' "$*"; }
warn() { printf '    WARN %s\n' "$*"; }
fail() { printf '    FAIL %s\n' "$*"; FAILS=$((FAILS+1)); }
say()  { printf '\n=== %s\n' "$*"; }

check_env() {
  say "1. this shell"
  if [ -z "${ROS_DISTRO:-}" ]; then
    fail "ROS_DISTRO unset - /opt/ros/*/setup.bash was never sourced in this terminal"
  else
    ok "ROS_DISTRO=$ROS_DISTRO"
  fi
  [ -n "${AMENT_PREFIX_PATH:-}" ] || warn "AMENT_PREFIX_PATH unset - the workspace install/setup.bash is not sourced"

  say "2. link variables"
  if [ -z "${ROS_DOMAIN_ID:-}" ]; then
    fail "ROS_DOMAIN_ID unset (= 0). Both computers must set the SAME value: source net_env.sh"
  else
    ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  <- write this in the session log and compare with the other machine"
  fi
  case "${RMW_IMPLEMENTATION:-}" in
    rmw_fastrtps_cpp) ok "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ;;
    '') fail "RMW_IMPLEMENTATION unset. It defaults to Fast DDS on both Foxy and Humble, but set it explicitly so the two machines cannot silently differ" ;;
    *)  fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} - the other machine must run the SAME middleware. Two vendors never talk to each other." ;;
  esac
  if [ "${ROS_LOCALHOST_ONLY:-0}" = 1 ]; then
    fail "ROS_LOCALHOST_ONLY=1 - nothing can leave this machine, and the topic list still looks perfectly normal"
  else
    ok "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
  fi
  if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    if [ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
      ok "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
      local peer
      peer=$(grep -oP '<address>\K[^<]+' "$FASTRTPS_DEFAULT_PROFILES_FILE" 2>/dev/null | head -1)
      [ -n "$peer" ] && printf '         unicast peer: %s\n' "$peer"
    else
      fail "FASTRTPS_DEFAULT_PROFILES_FILE points at $FASTRTPS_DEFAULT_PROFILES_FILE which is not readable - Fast DDS ignores it SILENTLY"
    fi
  else
    ok "no Fast DDS profile file (multicast discovery, the default)"
  fi
  [ -n "${CYCLONEDDS_URI:-}" ] && warn "CYCLONEDDS_URI is still set - inert under Fast DDS, but it means some other setup script also ran here"

  # Not a link variable, but the one that is most often copied verbatim from
  # the OTHER machine's setup notes - and a wrong value makes every later
  # `cd "$G1_WS"` fail somewhere the operator is not looking.
  if [ -n "${G1_WS:-}" ]; then
    if [ -r "$G1_WS/deps.repos" ]; then
      ok "G1_WS=$G1_WS"
    else
      fail "G1_WS=$G1_WS has no deps.repos - this is not the workspace on THIS machine (the two computers cloned to different paths)"
    fi
  fi

  say "3. interfaces"
  ip -br addr 2>/dev/null | awk '$2=="UP"||$2=="UNKNOWN"{printf "         %-14s %-8s %s\n",$1,$2,$3}'
  printf '         (Fast DDS uses ALL of these; no interface name has to be configured)\n'

  if [ -n "${G1_PEER_IP:-}" ]; then
    say "4. reachability of G1_PEER_IP=$G1_PEER_IP"
    if ping -c 2 -W 2 "$G1_PEER_IP" >/dev/null 2>&1; then
      ok "the other computer answers ICMP"
    else
      fail "no ICMP answer from $G1_PEER_IP - fix plain IP connectivity before looking at DDS at all"
    fi
  else
    say "4. reachability"
    warn "G1_PEER_IP not set - skipping the ping. Set it in ~/.g1_net_env; it costs nothing in multicast mode and is required for peers mode"
  fi
}

case "${1:-env}" in
  env)
    check_env
    printf '\n=== summary\n    hard failures : %d\n' "$FAILS"
    [ "$FAILS" -eq 0 ] && printf '\nLINK ENV OK.\n' || printf '\nLINK ENV FAILED - do not start the stack.\n'
    exit $(( FAILS > 0 ))
    ;;

  recv)
    printf 'Listening for a multicast datagram (Ctrl-C to stop).\n'
    printf 'Now run  g1_link_check.sh send  on the OTHER computer.\n\n'
    exec ros2 multicast receive
    ;;

  send)
    printf 'Sending one multicast datagram. Watch the other computer.\n\n'
    exec ros2 multicast send
    ;;

  topics)
    # --no-daemon matters: the ros2 daemon caches the FIRST domain/middleware
    # it ever saw in this login session, so `ros2 topic list` can report an
    # empty graph long after the link works. `ros2 daemon stop` also fixes it.
    say "graph as seen from this machine (daemon bypassed)"
    ros2 node list --no-daemon 2>/dev/null | sed 's/^/    node  /'
    ros2 topic list --no-daemon 2>/dev/null | sed 's/^/    topic /'
    say "the three topics Computer 3 needs"
    for t in /odom /obstacles_safe /dpcbf/plot; do
      if ros2 topic list --no-daemon 2>/dev/null | grep -qx "$t"; then
        ok "$t is on the graph"
      elif [ "$t" = /dpcbf/plot ]; then
        warn "$t absent - EXPECTED on hardware: no DPCBF control seam runs on the robot yet"
      else
        fail "$t absent"
      fi
    done
    printf '\n    hard failures : %d\n' "$FAILS"
    exit $(( FAILS > 0 ))
    ;;

  -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
  *) echo "unknown argument: $1" >&2; exit 64 ;;
esac
