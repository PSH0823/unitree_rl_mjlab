#!/bin/bash
# Computer 2 (G1 onboard PC) environment for DPCBF + off-board plotting.
#
#   source viz_env_computer2.sh [multicast|static|localhost]
#
# Sets RMW_IMPLEMENTATION / ROS_DOMAIN_ID / CYCLONEDDS_URI from the G1_VIZ_*
# variables (export them beforehand, or put them in ~/.g1_viz_env which this
# script sources if present). NOTHING is hardcoded here — a wrong-machine
# default is worse than a loud failure.
#
# Variables (see also viz_env.example):
#   G1_VIZ_DOMAIN_ID   ROS_DOMAIN_ID shared by Computers 1/2/3   [required]
#   G1_VIZ_IFACE       NIC toward Computer 3                     [required*]
#   G1_SENSOR_IFACE    NIC toward Computer 1, IF different       [optional]
#   G1_VIZ_PEER        Computer 3 IP        (static mode)        [required*]
#   G1_SENSOR_PEER     Computer 1 IP        (static + dual NIC)  [required*]
#   G1_VIZ_ALLOW_LOCALHOST=1  unlock the loopback config         [dev only]
#
# Mode picks the XML (installed with g1_perception_bringup):
#   multicast  viz_multicast.xml, or c2_dual_nic_multicast.xml when
#              G1_SENSOR_IFACE is set
#   static     viz_static_peers.xml / c2_dual_nic_static_peers.xml
#   localhost  localhost.xml — REFUSED unless G1_VIZ_ALLOW_LOCALHOST=1
#
# This script must be SOURCED (it exports into your shell), and re-sourced
# in every tmux pane (panes inherit the tmux server's env, not yours).

_g1viz_fail() { echo "viz_env: $*" >&2; return 1; }

_g1viz_main() {
    local mode="${1:-multicast}"
    [ -f "$HOME/.g1_viz_env" ] && . "$HOME/.g1_viz_env"

    # XML dir: source tree (scripts/ -> ../config) or merged install prefix
    # (lib/g1_perception_bringup -> ../../share/g1_perception_bringup/config).
    local here xml_dir
    here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    for xml_dir in "${G1_VIZ_XML_DIR:-}" \
                   "$here/../config/cyclonedds" \
                   "$here/../../share/g1_perception_bringup/config/cyclonedds"; do
        [ -n "$xml_dir" ] && [ -d "$xml_dir" ] && break
    done
    [ -d "$xml_dir" ] || { _g1viz_fail "cyclonedds XML dir not found (set G1_VIZ_XML_DIR)"; return 1; }
    xml_dir="$(cd "$xml_dir" && pwd)"

    [ -n "${G1_VIZ_DOMAIN_ID:-}" ] || { _g1viz_fail "G1_VIZ_DOMAIN_ID unset (must match Computers 1/2/3)"; return 1; }

    local xml=""
    case "$mode" in
      multicast)
        [ -n "${G1_VIZ_IFACE:-}" ] || { _g1viz_fail "G1_VIZ_IFACE unset"; return 1; }
        if [ -n "${G1_SENSOR_IFACE:-}" ]; then
            xml="$xml_dir/c2_dual_nic_multicast.xml"
        else
            xml="$xml_dir/viz_multicast.xml"
        fi
        ;;
      static)
        [ -n "${G1_VIZ_IFACE:-}" ] || { _g1viz_fail "G1_VIZ_IFACE unset"; return 1; }
        [ -n "${G1_VIZ_PEER:-}" ]  || { _g1viz_fail "G1_VIZ_PEER unset (Computer 3 IP)"; return 1; }
        if [ -n "${G1_SENSOR_IFACE:-}" ]; then
            [ -n "${G1_SENSOR_PEER:-}" ] || { _g1viz_fail "G1_SENSOR_PEER unset (Computer 1 IP; needed with G1_SENSOR_IFACE)"; return 1; }
            xml="$xml_dir/c2_dual_nic_static_peers.xml"
        else
            xml="$xml_dir/viz_static_peers.xml"
        fi
        ;;
      localhost)
        if [ "${G1_VIZ_ALLOW_LOCALHOST:-0}" != "1" ]; then
            _g1viz_fail "localhost mode is loopback-ONLY (dev). On the robot it would blind the whole stack. Set G1_VIZ_ALLOW_LOCALHOST=1 if this really is a dev machine."
            return 1
        fi
        xml="$xml_dir/localhost.xml"
        ;;
      *) _g1viz_fail "unknown mode '$mode' (multicast|static|localhost)"; return 1 ;;
    esac

    # NIC existence check — catches wlan0-vs-wlp2s0 style mistakes now, not
    # after 10 minutes of "why is there no data".
    if [ "$mode" != "localhost" ]; then
        for ifc in "${G1_VIZ_IFACE:-}" "${G1_SENSOR_IFACE:-}"; do
            [ -z "$ifc" ] && continue
            ip link show "$ifc" >/dev/null 2>&1 || { _g1viz_fail "interface '$ifc' does not exist on this machine"; return 1; }
        done
    fi

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="$G1_VIZ_DOMAIN_ID"
    export CYCLONEDDS_URI="file://$xml"
    echo "viz_env (computer2): mode=$mode"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
    echo "  G1_VIZ_IFACE=${G1_VIZ_IFACE:-}  G1_SENSOR_IFACE=${G1_SENSOR_IFACE:-<same NIC>}"
    [ "$mode" = static ] && echo "  peers: viz=${G1_VIZ_PEER:-} sensor=${G1_SENSOR_PEER:-<n/a>}"
    return 0
}

_g1viz_main "$@"
