cmake_minimum_required(VERSION 3.16)
project(g1_perception_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Integration smoke (§16.1): needs the ROS2-enabled simulator running;
  # SKIPs cleanly otherwise.
  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(test/test_bringup_sim.launch_test.py TIMEOUT 90)
    # Phase 2: measured-wall gate (±2 cm @ 1/2/4 m) + inline T9 + /scan
    # sanity. Self-contained (mini-sim source); SKIPs without python mujoco.
    add_launch_test(test/test_wall_accuracy.launch_test.py TIMEOUT 180)
    # Phase 2: bag-driven container integration (rates, CropBox accounting,
    # T9). SKIPs when the gitignored fixture bag is absent.
    add_launch_test(test/test_projection_replay.launch_test.py TIMEOUT 120)
  endif()
endif()

ament_package()
