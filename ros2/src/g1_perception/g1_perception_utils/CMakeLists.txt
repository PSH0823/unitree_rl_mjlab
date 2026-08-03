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
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dpcbf_ros_adapter REQUIRED)
find_package(yaml-cpp REQUIRED)

# The overlay recomputes the DPCBF constraint geometry from the SAME frozen
# headers the seam uses (D3: dpcbf/ is read, never written). Same repo-relative
# CACHE PATH trick as dpcbf_ros_adapter, for the same reason.
set(DPCBF_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../dpcbf/include"
    CACHE PATH "Include dir of the frozen dpcbf package")
if(NOT EXISTS "${DPCBF_INCLUDE_DIR}/dpcbf/dpcbf_safety_filter.h")
  message(FATAL_ERROR "dpcbf headers not found at ${DPCBF_INCLUDE_DIR}")
endif()

add_executable(base_footprint_publisher src/base_footprint_publisher.cpp)
target_include_directories(base_footprint_publisher PRIVATE include)
ament_target_dependencies(base_footprint_publisher rclcpp tf2 tf2_ros geometry_msgs)

add_executable(obstacles_marker_relay src/obstacles_marker_relay.cpp)
ament_target_dependencies(obstacles_marker_relay rclcpp visualization_msgs obstacle_detector)

add_executable(dpcbf_overlay src/dpcbf_overlay_node.cpp)
target_include_directories(dpcbf_overlay PRIVATE ${DPCBF_INCLUDE_DIR})
ament_target_dependencies(dpcbf_overlay
  rclcpp tf2 tf2_ros geometry_msgs visualization_msgs obstacle_detector
  nav_msgs diagnostic_msgs dpcbf_ros_adapter)
target_link_libraries(dpcbf_overlay yaml-cpp)

install(TARGETS base_footprint_publisher obstacles_marker_relay dpcbf_overlay
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footprint test/test_footprint.cpp)
  target_include_directories(test_footprint PRIVATE include)
endif()

ament_package()
