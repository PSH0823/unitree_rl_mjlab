cmake_minimum_required(VERSION 3.16)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/t7_extrinsic_guard.py test/t7_hw_extrinsic_guard.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # T7 extrinsic guard (§16.2): MJCF site pose == xacro pose.
  # Pinned to system python (R-11): conda 3.12 must never run this.
  add_test(NAME t7_extrinsic_guard
           COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/t7_extrinsic_guard.py)
  # Phase 5A: the same guard extended to the hardware odometry config. DLIO
  # has no TF listener, so its extrinsics parameters ARE its definition of
  # base_link — a stale copy silently redefines the frame instead of
  # producing a visible TF conflict.
  add_test(NAME t7_hw_extrinsic_guard
           COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/t7_hw_extrinsic_guard.py)
endif()

ament_package()
