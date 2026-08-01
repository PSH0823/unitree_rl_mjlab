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
  endif()
endif()

ament_package()
