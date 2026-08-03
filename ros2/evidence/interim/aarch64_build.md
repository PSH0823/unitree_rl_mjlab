# aarch64 build — what was retired, what is still carried

Interim block (between Phases 5A and 5B). **The evidence bar was not met.** The
bar was "the full workspace building on aarch64, with the test suite run". What
was achieved is 10 of 18 packages, **emulated**. The risk is reduced, not
retired, and the remainder is characterised rather than fixed.

## Phase 5A's blocker was wrong, and that part IS retired

5A recorded: "no cross toolchain, no `qemu-user-static`, `binfmt_misc` has no
aarch64 handler — registering one needs root". Every clause is true; the
conclusion is not.

1. **`qemu-user-static` does not need installing.** The Ubuntu .deb contains a
   static-PIE binary. `dpkg-deb -x` into `$HOME` gives a working
   `qemu-aarch64-static` (8.2.2) with no root and no build.
2. **binfmt_misc no longer needs real root.** Linux ≥ 6.7 makes it mountable
   inside a user+mount namespace; this kernel is **6.8.0-124**. So
   `unshare -Urm --map-root-user` → `mount -t binfmt_misc` → register a handler
   visible only to that namespace. Registering with flag `F` makes the kernel
   hold the interpreter's fd, so dispatch survives the chroot.
3. **Rootless podman can materialise the rootfs** even though it cannot execute
   it: `podman create --arch arm64 ros:humble-perception` + `podman export`.

Result: a full aarch64 Ubuntu 22.04 + ROS Humble environment, with working
`apt` (needs `APT::Sandbox::User "root"` — the drop-privs step crashes under
qemu), gcc 11.4.0 aarch64, and all 32 cores. Committed as
`tools/arm64_emulate.sh`.

One trap worth recording because it presents catastrophically: the binfmt
magic/mask must reach the kernel as **literal `\x` escapes** — binfmt_misc
parses them itself. Letting `printf` interpret them writes raw NULs, which
truncate magic and mask so the handler matches *every* ELF. The symptom is the
host's own `/usr/bin/mkdir` dying with "cannot execute binary file".

## Build result — 10 of 18, emulated

| built | not built |
|---|---|
| cyclonedds, unitree_sdk2, rmw_cyclonedds_cpp, obstacle_detector (incl. the new P-3 message), sim_msgs, g1_description, dpcbf_ros_adapter, g1_perception_utils, livox_sdk2_vendor, unitree_dds_wrapper_vendor | pcl_ros, pointcloud_to_laserscan, livox_ros_driver2, t10_dds_coexistence, g1_perception_bringup, safety_obstacle_filter, sim_mjlidar_bridge, direct_lidar_inertial_odometry |

Wall clock ≈ 16 min sequential for the packages that build. cyclonedds 2m42,
unitree_sdk2 5m18, obstacle_detector 2m42 — emulation is roughly 5× slower per
core, absorbed by 32 cores.

**No test suite was run on aarch64.** With the workspace incomplete there was
nothing meaningful to run, so the "tests run and reported" half of the bar is
untouched.

## Every failure found was environment, not architecture

Nothing arch-specific appeared — no endianness, no alignment, no intrinsics
(5A's static portability audit holds up). Five distinct faults, four understood
and fixed, one not:

| # | fault | status |
|---|---|---|
| 1 | `livox_ros_driver2` never declared its build-order dependency on `livox_sdk2_vendor`; the dev machine's **parallel** build happened to order it correctly and hid this for a whole phase. A sequential build fails at `find_library(LIVOX_LIDAR_SDK_LIBRARY)` | **FIXED** — patch 0008, a genuine repo bug |
| 2 | Ubuntu's `libvtk9-dev` ships imported targets for `RelWithDebInfo` only; a Release build of `pcl_ros` dies with "IMPORTED_LOCATION not set for VTK::GUISupportQt" | **FIXED** — `CMAKE_MAP_IMPORTED_CONFIG_RELEASE`, the documented CMake mechanism |
| 3 | PCL headers `#include <Eigen/StdVector>` unqualified; Debian puts Eigen in `/usr/include/eigen3` | **FIXED** — `-isystem /usr/include/eigen3` |
| 4 | `find_package(OpenSSL)` inside the ROS config chain reports "missing: OPENSSL_CRYPTO_LIBRARY (found version 3.0.2)" while a bare `find_library(crypto)` in the same shell resolves it | **FIXED** — name the paths explicitly |
| 5 | ament `_lib` shadow bug (below) | **NOT FIXED** |

All four fixes are in `tools/build_target.sh`, each annotated with the error it
prevents, and the script ends with a triage list keyed by error message.

## The one that is still open

    CMake Error at <prefix>/share/<pkg>/cmake/ament_cmake_export_libraries-extras.cmake:48
      Package '<pkg>' exports the library '<pkg>' which couldn't be found

...while `<prefix>/lib/lib<pkg>.so` is present and readable. Traced with
`--trace-source` plus `-DCMAKE_FIND_DEBUG_MODE=ON`: find_library prints **"The
item was found at /opt/ros/humble/lib/liblaser_geometry.so"** on the line above
the error saying it could not be found.

Mechanism: the generated code does `set(_lib "NOTFOUND")` (a NORMAL variable)
then `find_library(_lib ...)` (which writes a CACHE variable), then
`if(NOT _lib)`. `_lib` is one cache slot shared by every such file in a single
configure, and whether the normal variable survives the cache write depends on
CMP0126 in the enclosing scope — which depends on the include stack, which
depends on **which packages are installed**. Same source, same CMake 3.22.1:
fine on the x86_64 dev machine, fatal in the arm64 image, and the package it
blames moves as the installed set changes.

What was tried, and why nothing is shipped:

* `unset(_lib CACHE)` before the search got the original blocker
  (laser_geometry → pointcloud_to_laserscan) through, and the build went from
  3 to 10 of 18 packages.
* It then surfaced the identical failure in a second generated-file family
  (`rosidl_cmake_export_typesupport_libraries-extras.cmake`), and neither
  `unset(_lib)` before nor after the search resolved that one.
* Reverting every edit and re-testing with the now-larger installed package set
  reproduced the failure **on pristine files**, at a different package — which
  confirms the order-dependence and confirms the patch was not the cause.

Because the mechanism is not fully pinned down, **no automatic patch of the ROS
install is shipped**. Editing `/opt/ros` on the robot with a fix whose effect on
link lines is not understood is worse than the failure it papers over.
`tools/diagnose_ament_export_libraries.py` recognises the pattern, states
plainly that it is unsolved, and lists what to try in order. Upstream fix is
obvious and belongs in ros2/ament_cmake: stop using a shared cache variable
(`find_library(_lib_${_library} ...)`).

## Carried risk after this block

Smaller and much better understood, but real: **the target build is still not
proven**. What robot day now has that it did not: a script that encodes four
fixed faults, a named diagnosis for the fifth, a repo-side bug fixed for good
(patch 0008), and an emulator to reproduce any of it off-robot in minutes
rather than on robot time.
