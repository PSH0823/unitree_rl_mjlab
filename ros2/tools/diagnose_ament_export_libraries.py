#!/usr/bin/env python3
"""DIAGNOSTIC for an upstream ament_cmake failure that names the wrong culprit.

This tool does not fix anything. It recognises a specific build failure so that
whoever hits it on the robot stops looking for a missing package. Read the
"STATUS" section before spending time on it.

THE FAILURE

    CMake Error at <prefix>/share/<pkg>/cmake/ament_cmake_export_libraries-extras.cmake:48
      Package '<pkg>' exports the library '<pkg>' which couldn't be found

  or the same message from `rosidl_cmake_export_typesupport_libraries-extras.cmake`
  naming a typesupport library. In every observed case **the library exists**:

      $ ls <prefix>/lib/lib<pkg>.so        -> present, readable, right arch

WHAT THE GENERATED CODE DOES

    set(_lib "NOTFOUND")                          # a NORMAL variable
    find_library(_lib NAMES "<lib>" PATHS <prefix>/lib NO_DEFAULT_PATH ...)
    if(NOT _lib)
      message(FATAL_ERROR "... exports the library '<lib>' which couldn't be found")

  `_lib` is a single CACHE slot shared by every such file in one configure, and
  the `set()` above puts a NORMAL variable of the same name in front of it.
  Whether the normal variable survives find_library's cache write depends on
  policy CMP0126 in the enclosing scope, which depends on the include stack,
  which depends on WHICH PACKAGES ARE INSTALLED. Hence: the same source tree,
  the same CMake 3.22.1, builds on one machine and fails on another, and the
  package it blames moves around as the installed set changes.

  With `--trace-source` + `-DCMAKE_FIND_DEBUG_MODE=ON` you can watch
  find_library print "The item was found at <prefix>/lib/lib<pkg>.so" on the
  line above the error that says it could not be found.

STATUS — read this before "fixing" it

  Measured on an aarch64 `ros:humble-perception` image (interim block after
  Phase 5A):

    * Inserting `unset(_lib CACHE)` before the search got the ORIGINAL failing
      package (laser_geometry / pointcloud_to_laserscan) to configure, and the
      build advanced from 3 to 10 of 18 packages.
    * It then surfaced the same failure in a second generated-file family, and
      neither `unset(_lib)` before the search nor after it resolved that one.
    * So there is NO workaround here that has been shown to be correct, and
      none is applied automatically. Patching a ROS install with a fix whose
      mechanism is not fully pinned down is worse than the failure, because it
      silently changes link lines.

  What to do on the robot, in order:
    1. Run this tool. If it reports the pattern present, you know the failure
       class and that the library is NOT missing.
    2. Try a build with `--executor sequential` first — some of these are
       ordering-sensitive and clear on their own.
    3. If a specific package blocks, build it alone with `--packages-select`:
       a single-package configure has a different (much smaller) include stack
       and has been observed to succeed where the full run failed.
    4. Only then consider editing the generated extras files, and record
       exactly what you changed.

  Upstream: ros2/ament_cmake, ament_cmake_export_libraries and
  rosidl_cmake's typesupport equivalent. The clean upstream fix is to stop
  using a shared cache variable — e.g. `find_library(_lib_${_library} ...)`.

    ros2/tools/diagnose_ament_export_libraries.py [prefix]   # default /opt/ros/humble
"""
import os
import sys

NEEDLE = 'set(_lib "NOTFOUND")'
SUFFIX = '-extras.cmake'


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else '/opt/ros/humble'
    share = os.path.join(prefix, 'share')
    if not os.path.isdir(share):
        print(f'no such prefix: {share}')
        return 2

    hits, families = [], set()
    for root, _dirs, names in os.walk(share):
        for n in names:
            if not n.endswith(SUFFIX):
                continue
            p = os.path.join(root, n)
            try:
                with open(p) as fh:
                    if NEEDLE in fh.read():
                        hits.append(p)
                        families.add(n)
            except OSError:
                continue

    print(f'prefix: {prefix}')
    print(f'files using the shared `_lib` cache slot: {len(hits)}')
    for f in sorted(families):
        print(f'  family: {f}')
    if hits:
        print('\nIf a build fails with "exports the library \'X\' which couldn\'t')
        print('be found", check `ls %s/lib/libX.so` FIRST. If it is there, this' % prefix)
        print('is the known ament bug — see this file\'s header, not a missing package.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
