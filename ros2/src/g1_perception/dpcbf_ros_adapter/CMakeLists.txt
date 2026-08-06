cmake_minimum_required(VERSION 3.16)
project(dpcbf_ros_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(obstacle_detector REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dpcbf_viz_msgs REQUIRED)

# The frozen DPCBF package (D3) provides the ObstacleState contract as
# plain headers (no Eigen/OSQP in dpcbf_safety_filter.h). This workspace
# lives at <repo>/ros2, so the repo-relative path is stable under colcon.
set(DPCBF_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../dpcbf/include"
    CACHE PATH "Include dir of the frozen dpcbf package")
if(NOT EXISTS "${DPCBF_INCLUDE_DIR}/dpcbf/dpcbf_safety_filter.h")
  message(FATAL_ERROR "dpcbf headers not found at ${DPCBF_INCLUDE_DIR}")
endif()

add_library(dpcbf_ros_adapter STATIC
  src/obstacle_source.cpp
  src/viz_publisher.cpp)
target_include_directories(dpcbf_ros_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${DPCBF_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dpcbf_ros_adapter
  rclcpp obstacle_detector diagnostic_msgs dpcbf_viz_msgs)
set_target_properties(dpcbf_ros_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dpcbf_ros_adapter
        EXPORT export_dpcbf_ros_adapter
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_targets(export_dpcbf_ros_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp obstacle_detector diagnostic_msgs
                          dpcbf_viz_msgs)
# Consumers installed from this prefix must supply the dpcbf include dir
# themselves (simulate/deploy already have it via the dpcbf CMake targets);
# only the build-tree interface carries the repo-relative path.

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle_buffer test/test_obstacle_buffer.cpp)
  target_link_libraries(test_obstacle_buffer dpcbf_ros_adapter)
  ament_add_gtest(test_obstacle_source test/test_obstacle_source.cpp
                  TIMEOUT 120)
  target_link_libraries(test_obstacle_source dpcbf_ros_adapter)
  # Plot bridge: rate decimation, latest-wins content, non-blocking Push,
  # no-new-tick silence, yaml section loader.
  ament_add_gtest(test_viz_publisher test/test_viz_publisher.cpp TIMEOUT 120)
  target_link_libraries(test_viz_publisher dpcbf_ros_adapter yaml-cpp)
  # Header-only overlay geometry (§4.6). The authoritative gate is
  # `boundary_check math` against the frozen library, which needs a capture
  # fixture; these run anywhere.
  ament_add_gtest(test_dpcbf_boundary test/test_dpcbf_boundary.cpp)
  target_include_directories(test_dpcbf_boundary PRIVATE
                             include ${DPCBF_INCLUDE_DIR})
endif()

ament_package()
