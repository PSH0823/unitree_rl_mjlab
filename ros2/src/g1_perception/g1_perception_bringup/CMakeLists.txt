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
    # Phase 3 gates. T4 static accuracy + extractor integration on the
    # s1_surveyed fixture; T5 dynamic tracking at both speeds on the
    # s2_cross fixtures; marker-relay mapping. All SKIP without the
    # gitignored fixture bags.
    add_launch_test(test/test_detection_static.launch_test.py TIMEOUT 120)
    add_launch_test(test/test_tracking_dynamic_05.launch_test.py TIMEOUT 120)
    add_launch_test(test/test_tracking_dynamic_08.launch_test.py TIMEOUT 120)
    add_launch_test(test/test_marker_relay.launch_test.py TIMEOUT 60)
    # T8 replay determinism — HARD gate now that P-2 landed. Two sequential
    # container sessions, so it is a plain CTest script (skip code 77 when
    # the fixture bag / workspace is unavailable).
    add_test(NAME t8_replay_determinism
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t8_replay_determinism.py)
    set_tests_properties(t8_replay_determinism PROPERTIES
                         TIMEOUT 300 SKIP_RETURN_CODE 77)
  endif()
endif()

ament_package()
