cmake_minimum_required(VERSION 3.16)
project(tracked_objects_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception_3d_msgs REQUIRED)

add_library(tracked_objects_viz_component SHARED
  src/tracked_objects_viz_node.cpp)
ament_target_dependencies(tracked_objects_viz_component
  rclcpp rclcpp_components autoware_perception_msgs visualization_msgs)
rclcpp_components_register_nodes(tracked_objects_viz_component
  "tracked_objects_viz::TrackedObjectsVizNode")

add_library(convex_objects_viz_component SHARED
  src/convex_objects_viz_node.cpp)
ament_target_dependencies(convex_objects_viz_component
  rclcpp rclcpp_components perception_3d_msgs visualization_msgs)
rclcpp_components_register_nodes(convex_objects_viz_component
  "tracked_objects_viz::ConvexObjectsVizNode")

install(TARGETS tracked_objects_viz_component convex_objects_viz_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
