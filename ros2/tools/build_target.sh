#!/bin/bash
# On-target build for the DPCBF perception workspace (Phase 5B robot day, or any
# fresh machine — Orin, Pi, CI runner, container).
#
# Every step below exists because it FAILED on a machine other than the one this
# project was developed on. The emulated aarch64 bring-up in the interim block
# after 5A produced all of them; each is annotated with the error it prevents so
# that a failure on robot day is triaged in seconds instead of explored.
#
#   ros2/tools/build_target.sh            # build
#   ros2/tools/build_target.sh --check    # environment preflight only, no build
#
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
ROS_PREFIX="${ROS_PREFIX:-/opt/ros/humble}"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

say() { printf '\n=== %s\n' "$*"; }
fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }

say "0. environment"
[ -d "$ROS_PREFIX" ] || fail "no ROS at $ROS_PREFIX (set ROS_PREFIX)"
echo "arch      : $(uname -m)"
echo "ros       : $ROS_PREFIX"
echo "cmake     : $(cmake --version | head -1)"
echo "cores     : $(nproc)"
echo "free disk : $(df -h "$WS" | awk 'NR==2{print $4}')"

say "1. known-failure diagnostic (informational)"
# Upstream ament_cmake bug, characterised but NOT auto-fixed — read
# diagnose_ament_export_libraries.py before attempting a fix.
python3 "$HERE/diagnose_ament_export_libraries.py" "$ROS_PREFIX"

say "2. system build dependencies"
MISSING=()
for pkg in build-essential cmake libeigen3-dev libpcl-dev libarmadillo-dev \
           libboost-all-dev libssl-dev python3-colcon-common-extensions \
           python3-vcstool; do
  dpkg -s "$pkg" >/dev/null 2>&1 || MISSING+=("$pkg")
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "missing: ${MISSING[*]}"
  echo "  sudo apt-get install -y --no-install-recommends ${MISSING[*]}"
  [ "$CHECK_ONLY" = 1 ] || fail "install the above first"
else
  echo "all present"
fi

say "3. pinned externals"
[ -d "$WS/src/external/cyclonedds" ] || { cd "$WS" && ./setup_external.sh; }
echo "ok"

[ "$CHECK_ONLY" = 1 ] && { echo; echo "preflight only — not building."; exit 0; }

say "4. build"
# shellcheck disable=SC1090,SC1091
set +u; source "$ROS_PREFIX/setup.bash"; set -u
cd "$WS"
#
# --executor sequential: with the default parallel executor and --merge-install,
#   pointcloud_to_laserscan intermittently fails to configure while other
#   packages are installing into the same merged prefix. It builds fine alone.
#   Sequential costs wall-clock, not correctness; drop it once you have a green
#   baseline and want speed.
# CMAKE_MAP_IMPORTED_CONFIG_RELEASE: Ubuntu's libvtk9-dev ships imported targets
#   for RelWithDebInfo ONLY, so a Release build of pcl_ros dies with
#   "IMPORTED_LOCATION not set for imported target VTK::GUISupportQt".
#   This is the documented CMake mechanism for exactly that, not a workaround.
# OPENSSL_*: fastrtps-config.cmake's find_package(OpenSSL) fails inside the ROS
#   config chain with "missing: OPENSSL_CRYPTO_LIBRARY (found version 3.0.2)"
#   even though a bare find_library(crypto) in the same environment resolves it.
#   Naming the paths is the cheap, deterministic fix.
MULTIARCH="$(gcc -print-multiarch)"
colcon build \
  --merge-install \
  --executor sequential \
  --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               "-DCMAKE_CXX_FLAGS=-isystem /usr/include/eigen3" \
               "-DCMAKE_MAP_IMPORTED_CONFIG_RELEASE=RelWithDebInfo;Release;None;" \
               -DOPENSSL_ROOT_DIR=/usr \
               -DOPENSSL_INCLUDE_DIR=/usr/include \
               "-DOPENSSL_CRYPTO_LIBRARY=/usr/lib/$MULTIARCH/libcrypto.so" \
               "-DOPENSSL_SSL_LIBRARY=/usr/lib/$MULTIARCH/libssl.so"

say "5. test"
colcon test --merge-install || true
colcon test-result --all || true

cat <<'EOF'

=== robot-day triage: the first hour ===
If the build fails, match the message here BEFORE investigating.

 "<pkg> exports the library '<pkg>' which couldn't be found"
     -> KNOWN upstream ament bug, NOT a missing package: check
        `ls $ROS_PREFIX/lib/lib<pkg>.so` — it will be there. Package-set
        dependent, so it appears on fresh machines only. Try, in order:
        (a) --executor sequential (already set below); (b) build that one
        package alone with --packages-select, which has a smaller include
        stack and has been seen to succeed where the full run failed.
        Full write-up: tools/diagnose_ament_export_libraries.py.

 "IMPORTED_LOCATION not set for imported target VTK::..."
     -> the CMAKE_MAP_IMPORTED_CONFIG_RELEASE arg above was dropped.

 "Eigen/StdVector: No such file or directory" building pcl_ros
     -> PCL's headers include Eigen unqualified; Debian puts it in
        /usr/include/eigen3. The CMAKE_CXX_FLAGS arg above supplies it.

 "Could NOT find OpenSSL ... (missing: OPENSSL_CRYPTO_LIBRARY) (found version ...)"
     -> the OPENSSL_* args above were dropped. Note it says it FOUND the
        version: the headers are fine, only the library lookup failed. Do not
        go looking for a missing package.

 "Could not find LIVOX_LIDAR_SDK_LIBRARY ... liblivox_lidar_sdk_shared.so"
     -> livox_sdk2_vendor has not built yet. Patch 0008 declares the
        dependency; confirm setup_external.sh applied it
        (`grep livox_sdk2_vendor src/external/livox_ros_driver2/package.xml`).

 pointcloud_to_laserscan / pcl_ros fail with unrelated CMake noise
     -> stale CMakeCache from a half-configured run. `rm -rf build/<pkg>` and
        retry that package alone with --packages-select.

 anything mentioning `_rclpy_pybind11` or a python 3.12 path
     -> a conda/venv python is shadowing the system one. Use /usr/bin/python3
        explicitly (R-11); `head -1 $(which colcon)`.

If the failure is NOT in this list, capture the full log and move on to the
capture blocks that do not need a fresh build. Robot time is for capture.
EOF
