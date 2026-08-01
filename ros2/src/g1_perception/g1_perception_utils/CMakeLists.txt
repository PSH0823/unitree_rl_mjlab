cmake_minimum_required(VERSION 3.16)
project(g1_perception_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)

add_executable(base_footprint_publisher src/base_footprint_publisher.cpp)
target_include_directories(base_footprint_publisher PRIVATE include)
ament_target_dependencies(base_footprint_publisher rclcpp tf2 tf2_ros geometry_msgs)

add_executable(obstacles_marker_relay src/obstacles_marker_relay.cpp)
ament_target_dependencies(obstacles_marker_relay rclcpp visualization_msgs obstacle_detector)

install(TARGETS base_footprint_publisher obstacles_marker_relay
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footprint test/test_footprint.cpp)
  target_include_directories(test_footprint PRIVATE include)
endif()

ament_package()
