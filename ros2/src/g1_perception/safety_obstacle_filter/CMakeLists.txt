cmake_minimum_required(VERSION 3.16)
project(safety_obstacle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(obstacle_detector REQUIRED)

add_library(safety_obstacle_filter_component SHARED
  src/safety_obstacle_filter_node.cpp)
target_include_directories(safety_obstacle_filter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(safety_obstacle_filter_component
  rclcpp rclcpp_components obstacle_detector)
rclcpp_components_register_nodes(safety_obstacle_filter_component
  "safety_obstacle_filter::SafetyObstacleFilterNode")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS safety_obstacle_filter_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gating test/test_gating.cpp)
  target_include_directories(test_gating PRIVATE include)
  ament_target_dependencies(test_gating rclcpp obstacle_detector)
endif()

ament_package()
