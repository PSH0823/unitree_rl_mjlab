#!/bin/bash
# Fast DDS link environment for Computer 2 (G1 onboard, Foxy) and Computer 3
# (operator laptop, Humble). ONE script for both machines - nothing in here is
# machine-specific; the per-machine values live in ~/.g1_net_env.
#
#   source net_env.sh              # default: multicast discovery
#   source net_env.sh peers        # multicast blocked -> unicast to G1_PEER_IP
#   source net_env.sh udp-only     # BOTH sides on ONE host (rehearsal only)
#   source net_env.sh peers udp-only
#
# WHY THIS EXISTS, given that ~/.g1_net_env already sets the real variables:
# the file is enough to be ON the link; this script is the part that REFUSES
# to let you continue with a half-configured shell. It checks the values,
# renders the Fast DDS profile when one is needed, prints exactly what the
# next command will inherit, and leaves G1_NET_OK=1 (or 0) behind so the
# runbook can assert on it.
#
# Design rules learned from the CycloneDDS version this replaces:
#   * every variable it touches is a variable ROS itself reads - no private
#     names that silently mean nothing (G1_VIZ_DOMAIN_ID was exactly that);
#   * it never needs an interface NAME. Fast DDS uses every up interface by
#     default, so wlan0-vs-wlp2s0 cannot break the link;
#   * it does not read any file out of the install prefix, so it behaves the
#     same in a source tree, a merge-install prefix and an isolated one.

_g1net_say()  { printf '    %s\n'      "$*"; }
_g1net_ok()   { printf '    ok   %s\n' "$*"; }
_g1net_warn() { printf '    WARN %s\n' "$*"; }
_g1net_fail() { printf '    FAIL %s\n' "$*"; }

_g1net_main() {
    local want_peers=0 want_udp_only=0 arg
    for arg in "$@"; do
        case "$arg" in
            peers)              want_peers=1 ;;
            udp-only|udp_only)  want_udp_only=1 ;;
            multicast|auto|'')  ;;
            *) _g1net_fail "unknown mode '$arg' (peers | udp-only)"; return 1 ;;
        esac
    done

    [ -f "$HOME/.g1_net_env" ] && . "$HOME/.g1_net_env"

    printf '\n=== g1 net env\n'

    local bad=0

    # --- ROS_DOMAIN_ID ----------------------------------------------------
    if [ -z "${ROS_DOMAIN_ID:-}" ]; then
        _g1net_fail "ROS_DOMAIN_ID unset. Create ~/.g1_net_env (see net_env.example) or export it here."
        bad=1
    elif ! printf '%s' "$ROS_DOMAIN_ID" | grep -qE '^[0-9]+$' || [ "$ROS_DOMAIN_ID" -gt 101 ]; then
        _g1net_fail "ROS_DOMAIN_ID='$ROS_DOMAIN_ID' is not an integer in 0..101"
        bad=1
    else
        _g1net_ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID   (must be IDENTICAL on the other computer)"
        [ "$ROS_DOMAIN_ID" = 0 ] && _g1net_warn "domain 0 is every machine's default - anything else on this network joins you"
    fi

    # --- middleware -------------------------------------------------------
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    _g1net_ok "RMW_IMPLEMENTATION=rmw_fastrtps_cpp   (default of both Foxy and Humble; nothing to install)"

    # --- the loopback trap ------------------------------------------------
    if [ "${ROS_LOCALHOST_ONLY:-0}" = 1 ]; then
        _g1net_fail "ROS_LOCALHOST_ONLY=1 - traffic can never leave this machine, while topic names still look correct. Set it to 0 in ~/.g1_net_env."
        bad=1
    else
        export ROS_LOCALHOST_ONLY=0
        _g1net_ok "ROS_LOCALHOST_ONLY=0"
    fi

    # --- leftovers from the CycloneDDS era --------------------------------
    if [ -n "${CYCLONEDDS_URI:-}" ]; then
        _g1net_warn "CYCLONEDDS_URI was set ($CYCLONEDDS_URI) - inert under Fast DDS, unsetting it so later diagnosis is unambiguous"
        unset CYCLONEDDS_URI
    fi

    # --- Fast DDS profile, only when a mode asks for one ------------------
    local xml="$HOME/.g1_fastdds.xml"
    if [ "$want_peers" = 1 ] || [ "$want_udp_only" = 1 ]; then
        if [ "$want_peers" = 1 ] && [ -z "${G1_PEER_IP:-}" ]; then
            _g1net_fail "peers mode needs G1_PEER_IP (the OTHER computer's IP) in ~/.g1_net_env"
            bad=1
        else
            _g1net_render_xml "$xml" "$want_peers" "$want_udp_only" || bad=1
        fi
        if [ "$bad" = 0 ]; then
            export FASTRTPS_DEFAULT_PROFILES_FILE="$xml"
            _g1net_ok "FASTRTPS_DEFAULT_PROFILES_FILE=$xml"
            [ "$want_peers" = 1 ]    && _g1net_say "     unicast discovery peer: $G1_PEER_IP  (the OTHER computer must list THIS one)"
            [ "$want_udp_only" = 1 ] && _g1net_say "     shared memory disabled (Foxy<->Humble on one host)"
        fi
    else
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        _g1net_ok "discovery: multicast (default). No profile file - nothing to get out of date."
    fi

    if [ "$bad" = 0 ]; then
        export G1_NET_OK=1
        printf '    ----\n    PASS - this shell is on the link. Every NEW terminal needs this again.\n\n'
        return 0
    fi
    export G1_NET_OK=0
    printf '    ----\n    FAIL - fix the lines above and source this script again. DO NOT launch anything yet.\n\n'
    return 1
}

# Written, not read from the install prefix: one less path to resolve wrongly.
_g1net_render_xml() {
    local out="$1" peers="$2" udp_only="$3"
    local transports='' builtin=''
    if [ "$udp_only" = 1 ]; then
        transports='
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>g1_udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>'
    fi
    if [ "$peers" = 1 ]; then
        builtin="
        <builtin>
          <initialPeersList>
            <locator><udpv4><address>${G1_PEER_IP}</address></udpv4></locator>
          </initialPeersList>
        </builtin>"
    fi
    local userts=''
    if [ "$udp_only" = 1 ]; then
        userts='
        <userTransports><transport_id>g1_udp_only</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>'
    fi
    cat > "$out" <<EOF || return 1
<?xml version="1.0" encoding="UTF-8" ?>
<!-- GENERATED by net_env.sh - edit ~/.g1_net_env, not this file. -->
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>${transports}
    <participant profile_name="g1_link" is_default_profile="true">
      <rtps>${builtin}${userts}
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
    return 0
}

_g1net_main "$@"
