cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

include(CTest)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dpcbf ${CMAKE_BINARY_DIR}/dpcbf)
target_include_directories(dpcbf_dynamic_obstacles PRIVATE mujoco/include)
target_link_libraries(dpcbf_dynamic_obstacles PUBLIC mujoco yaml-cpp)
if(NOT TARGET dpcbf_visualizer)
  message(FATAL_ERROR "OpenCV (core, imgproc, highgui) is required for the DPCBF visualization window")
endif()

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco PRIVATE
  dpcbf_dynamic_obstacles
  dpcbf_safety_filter
  dpcbf_visualizer
)
# Phase-4 seam headers (axis/command map, Filter-I/O capture, ObstacleSource
# declaration) come straight from the adapter package's source tree; they are
# rclcpp-free, so the non-ROS2 build keeps zero new link dependencies.
set(DPCBF_ROS_ADAPTER_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../ros2/src/g1_perception/dpcbf_ros_adapter)
target_include_directories(unitree_mujoco PRIVATE
  ${DPCBF_ROS_ADAPTER_DIR}/include)

# Optional ROS2 sim bridge (§11.2). OFF => byte-identical baseline behavior.
option(UNITREE_MUJOCO_WITH_ROS2
       "Publish /clock, /sim/mj_state, /sim/gt_obstacles and dump the mirror model" OFF)
if(UNITREE_MUJOCO_WITH_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sim_msgs REQUIRED)
  find_package(obstacle_detector REQUIRED)
  find_package(dpcbf_ros_adapter REQUIRED)
  target_sources(unitree_mujoco PRIVATE src/ros2_bridge.cc)
  target_compile_definitions(unitree_mujoco PRIVATE UNITREE_MUJOCO_WITH_ROS2)
  target_link_libraries(unitree_mujoco PRIVATE
    rclcpp::rclcpp
    ${rosgraph_msgs_TARGETS}
    ${sim_msgs_TARGETS}
    ${obstacle_detector_TARGETS}
    dpcbf_ros_adapter::dpcbf_ros_adapter
  )

  # Gate T1 (oracle equivalence) replay harness + the offline paired-filter
  # A/B evaluator (§17.3). Sources live in the adapter package to respect the
  # Phase-4 outside-ros2/ file budget; they need the dpcbf static libs, so
  # they build here. t1_replay returns 77 (SKIP) without a capture fixture.
  add_executable(t1_replay ${DPCBF_ROS_ADAPTER_DIR}/tools/t1_replay.cc)
  target_link_libraries(t1_replay PRIVATE
    dpcbf_safety_filter dpcbf_ros_adapter::dpcbf_ros_adapter)
  add_executable(ab_eval ${DPCBF_ROS_ADAPTER_DIR}/tools/ab_eval.cc)
  target_link_libraries(ab_eval PRIVATE
    dpcbf_safety_filter dpcbf_ros_adapter::dpcbf_ros_adapter)
  add_test(NAME t1_oracle_equivalence
           COMMAND t1_replay
             ${CMAKE_CURRENT_SOURCE_DIR}/../ros2/test_fixtures/t1_baseline/t1_baseline_capture.bin
             ${CMAKE_CURRENT_SOURCE_DIR}/../dpcbf/config/dpcbf_config.yaml)
  set_tests_properties(t1_oracle_equivalence PROPERTIES SKIP_RETURN_CODE 77)
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
