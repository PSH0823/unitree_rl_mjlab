#!/bin/bash
# Run a command inside an aarch64 rootfs under qemu-user emulation — WITHOUT ROOT.
#
# This is how the workspace was first built for the robot's architecture on a
# machine with no cross toolchain, no qemu-user-static package, no binfmt
# handler and no sudo. Phase 5A recorded all four as blockers; none of them
# actually is.
#
# The trick, in three parts:
#
#   1. `qemu-user-static` does not need to be INSTALLED. Its .deb contains a
#      static-PIE binary; `dpkg-deb -x` into $HOME is enough.
#   2. Linux >= 6.7 allows binfmt_misc to be mounted and registered inside a
#      user+mount namespace. `unshare -Urm --map-root-user` gives uid 0 in that
#      namespace, which is enough to mount it and register a handler visible
#      only there. (Kernel here: 6.8.)
#   3. Registering with flag F ("fix binary") makes the kernel open the
#      interpreter at REGISTRATION time and keep the fd, so implicit dispatch
#      still works after chroot into a rootfs that contains no qemu. With
#      implicit dispatch, execve of aarch64 binaries just works — which is what
#      lets cmake/make/colcon/apt run unmodified.
#
# Setup (one time, no root):
#
#   mkdir -p ~/aarch64/dl && cd ~/aarch64/dl
#   curl -sfLO http://archive.ubuntu.com/ubuntu/pool/universe/q/qemu/qemu-user-static_8.2.2+ds-0ubuntu1.18_amd64.deb
#   dpkg-deb -x qemu-user-static_*.deb ~/aarch64/opt
#   # rootfs from any arm64 image; rootless podman can PULL and EXPORT even
#   # though it cannot EXEC one:
#   C=$(podman create --arch arm64 docker.io/library/ros:humble-perception /bin/true)
#   mkdir -p ~/aarch64/rootfs && podman export "$C" | tar -x -C ~/aarch64/rootfs
#   podman rm "$C"
#
# Usage:
#   ARM64_ROOT=~/aarch64 ros2/tools/arm64_emulate.sh                 # shell
#   ARM64_ROOT=~/aarch64 ros2/tools/arm64_emulate.sh -w /repo/ros2 colcon build ...
#
# Notes:
#   * apt inside the rootfs needs `APT::Sandbox::User "root"` — its drop-privs
#     step crashes under qemu-user ("Method http has died unexpectedly!").
#   * Emulated builds are ~5-10x slower per core; 32 cores absorbs most of it.
#     The full workspace takes tens of minutes, not hours.
set -euo pipefail

ROOT="${ARM64_ROOT:-$HOME/aarch64}"
ROOTFS="${ARM64_ROOTFS:-$ROOT/rootfs}"
QEMU="${ARM64_QEMU:-$ROOT/opt/usr/bin/qemu-aarch64-static}"
# Space-separated host paths bind-mounted at the SAME path inside, so build
# trees and repo-relative CMake paths work identically on both sides.
BIND="${ARM64_BIND:-$(cd "$(dirname "$0")/../.." && pwd)}"
WORKDIR="/"

while [ $# -gt 0 ]; do
  case "$1" in
    -w) WORKDIR="$2"; shift 2 ;;
    *)  break ;;
  esac
done
[ $# -eq 0 ] && set -- /bin/bash -l

[ -x "$QEMU" ]   || { echo "arm64_emulate: no qemu at $QEMU (see header)" >&2; exit 1; }
[ -d "$ROOTFS" ] || { echo "arm64_emulate: no rootfs at $ROOTFS (see header)" >&2; exit 1; }

# AArch64 ELF magic/mask for binfmt_misc (EM_AARCH64 = 0xb7, little-endian).
# These MUST reach the kernel as literal \x escapes — binfmt_misc parses them
# itself. Letting printf interpret them writes raw NULs, which truncate magic
# and mask so the handler matches EVERY ELF, x86-64 included; the symptom is
# the host's own /bin/mkdir dying with "cannot execute binary file".
MAGIC='\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\xb7\x00'
MASK='\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff'

export ARM64_ROOTFS="$ROOTFS" ARM64_QEMU="$QEMU" ARM64_BIND="$BIND" ARM64_WORKDIR="$WORKDIR"
exec unshare -Urm --map-root-user /bin/bash -c '
set -euo pipefail
R="$ARM64_ROOTFS"
mount -t binfmt_misc binfmt_misc /proc/sys/fs/binfmt_misc
printf "%s" ":qemu-aarch64:M::'"$MAGIC"':'"$MASK"':$ARM64_QEMU:PCF" \
  > /proc/sys/fs/binfmt_misc/register

# rbind /proc rather than mounting a fresh procfs: that would need a PID
# namespace, and creating one makes the chrooted shell PID 1.
mount --rbind /proc "$R/proc"; mount --make-rslave "$R/proc"
mount --rbind /sys  "$R/sys";  mount --make-rslave "$R/sys"
mount --rbind /dev  "$R/dev";  mount --make-rslave "$R/dev"
mount -t tmpfs tmpfs "$R/tmp"
for B in $ARM64_BIND; do
  [ -d "$B" ] || continue
  mkdir -p "$R$B"
  mount --bind "$B" "$R$B"
done
cp -f /etc/resolv.conf "$R/etc/resolv.conf" 2>/dev/null || true

exec chroot "$R" /usr/bin/env -i \
  HOME=/root TERM="${TERM:-dumb}" LANG=C.UTF-8 \
  PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  DEBIAN_FRONTEND=noninteractive ARM64_WORKDIR="$ARM64_WORKDIR" \
  /bin/bash -c '"'"'cd "$ARM64_WORKDIR"; exec "$@"'"'"' -- "$@"
' -- "$@"
