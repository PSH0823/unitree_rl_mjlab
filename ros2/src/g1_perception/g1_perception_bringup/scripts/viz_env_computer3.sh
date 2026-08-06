#!/bin/bash
# Computer 3 (operator laptop) environment for the DPCBF plotting client.
#
#   source viz_env_computer3.sh [multicast|static|localhost]
#
# Same contract as viz_env_computer2.sh, minus the sensor NIC (Computer 3
# touches only the visualization network — it is a read-only endpoint):
#
#   G1_VIZ_DOMAIN_ID   ROS_DOMAIN_ID shared with Computer 2      [required]
#   G1_VIZ_IFACE       NIC toward Computer 2                     [required*]
#   G1_VIZ_PEER        Computer 2 IP        (static mode)        [required*]
#   G1_VIZ_ALLOW_LOCALHOST=1  unlock the loopback config         [dev only]
#
# Must be SOURCED. Reads ~/.g1_viz_env first if present.

_g1viz_fail() { echo "viz_env: $*" >&2; return 1; }

_g1viz_main() {
    local mode="${1:-multicast}"
    [ -f "$HOME/.g1_viz_env" ] && . "$HOME/.g1_viz_env"

    local here xml_dir
    here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    for xml_dir in "${G1_VIZ_XML_DIR:-}" \
                   "$here/../config/cyclonedds" \
                   "$here/../../share/g1_perception_bringup/config/cyclonedds"; do
        [ -n "$xml_dir" ] && [ -d "$xml_dir" ] && break
    done
    [ -d "$xml_dir" ] || { _g1viz_fail "cyclonedds XML dir not found (set G1_VIZ_XML_DIR)"; return 1; }
    xml_dir="$(cd "$xml_dir" && pwd)"

    [ -n "${G1_VIZ_DOMAIN_ID:-}" ] || { _g1viz_fail "G1_VIZ_DOMAIN_ID unset (must match Computer 2)"; return 1; }

    local xml=""
    case "$mode" in
      multicast)
        [ -n "${G1_VIZ_IFACE:-}" ] || { _g1viz_fail "G1_VIZ_IFACE unset"; return 1; }
        xml="$xml_dir/viz_multicast.xml"
        ;;
      static)
        [ -n "${G1_VIZ_IFACE:-}" ] || { _g1viz_fail "G1_VIZ_IFACE unset"; return 1; }
        [ -n "${G1_VIZ_PEER:-}" ]  || { _g1viz_fail "G1_VIZ_PEER unset (Computer 2 IP)"; return 1; }
        xml="$xml_dir/viz_static_peers.xml"
        ;;
      localhost)
        if [ "${G1_VIZ_ALLOW_LOCALHOST:-0}" != "1" ]; then
            _g1viz_fail "localhost mode is loopback-ONLY (dev). Set G1_VIZ_ALLOW_LOCALHOST=1 if this really is a dev machine."
            return 1
        fi
        xml="$xml_dir/localhost.xml"
        ;;
      *) _g1viz_fail "unknown mode '$mode' (multicast|static|localhost)"; return 1 ;;
    esac

    if [ "$mode" != "localhost" ] && [ -n "${G1_VIZ_IFACE:-}" ]; then
        ip link show "$G1_VIZ_IFACE" >/dev/null 2>&1 || { _g1viz_fail "interface '$G1_VIZ_IFACE' does not exist on this machine"; return 1; }
    fi

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="$G1_VIZ_DOMAIN_ID"
    export CYCLONEDDS_URI="file://$xml"
    echo "viz_env (computer3): mode=$mode"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
    echo "  G1_VIZ_IFACE=${G1_VIZ_IFACE:-}"
    [ "$mode" = static ] && echo "  peer: ${G1_VIZ_PEER:-}"
    return 0
}

_g1viz_main "$@"
