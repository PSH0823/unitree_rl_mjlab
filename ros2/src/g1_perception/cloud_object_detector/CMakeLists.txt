cmake_minimum_required(VERSION 3.16)
project(cloud_object_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)  # tf2_eigen links the Eigen3::Eigen target
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search surface)
find_package(perception_3d_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(cloud_object_detector_component SHARED
  src/cloud_object_detector_node.cpp)
target_include_directories(cloud_object_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(cloud_object_detector_component SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_object_detector_component ${PCL_LIBRARIES})
ament_target_dependencies(cloud_object_detector_component
  rclcpp rclcpp_components autoware_perception_msgs geometry_msgs
  sensor_msgs pcl_conversions tf2_eigen tf2_ros perception_3d_msgs)
rclcpp_components_register_nodes(cloud_object_detector_component
  "cloud_object_detector::CloudObjectDetectorNode")

# Joins the autoware track stream with the detector's 3D hulls by pose.
add_library(tracked_convex_attach_component SHARED
  src/tracked_convex_attach_node.cpp)
ament_target_dependencies(tracked_convex_attach_component
  rclcpp rclcpp_components autoware_perception_msgs geometry_msgs
  perception_3d_msgs unique_identifier_msgs)
rclcpp_components_register_nodes(tracked_convex_attach_component
  "cloud_object_detector::TrackedConvexAttachNode")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS cloud_object_detector_component tracked_convex_attach_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection_geometry test/test_detection_geometry.cpp)
  target_include_directories(test_detection_geometry PRIVATE include)
endif()

ament_package()
