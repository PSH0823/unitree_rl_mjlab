cmake_minimum_required(VERSION 3.16)
project(g1_perception_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
# Phase 5A: robot-day tools, runnable as `ros2 run g1_perception_bringup <x>`
# on a machine that only has the install prefix (the G1 onboard PC).
install(PROGRAMS
        test/hw_config_check.py
        test/hw_source_stub.py
        test/selfhit_analysis.py
        DESTINATION lib/${PROJECT_NAME})

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
    # Phase 5A hardware seam. Both drive hw_source_stub.py — the pinned
    # livox_ros_driver2's OUTPUT format (7 fields, 26-byte packed stride,
    # RELIABLE publisher, wall-clock stamps) plus a synthetic IMU — so the
    # cloud contract, the QoS wire-check and the TF tree shape are gated on
    # the bench instead of on robot time. SKIP without the fixture bag.
    add_launch_test(test/test_hw_source_contract.launch_test.py TIMEOUT 180)
    add_launch_test(test/test_dlio_wiring.launch_test.py TIMEOUT 180)
  endif()
  # Phase 5A: hardware config preflight. Exits 2 while MID360_config.json
  # still carries the Q-1 placeholder IPs, which is the expected state on the
  # dev machine — the check is a robot-day gate, not a dev-machine gate.
  add_test(NAME hw_config_check
           COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/hw_config_check.py)
  set_tests_properties(hw_config_check PROPERTIES SKIP_RETURN_CODE 2)
endif()

ament_package()
