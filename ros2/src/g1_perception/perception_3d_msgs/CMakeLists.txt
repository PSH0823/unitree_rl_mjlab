cmake_minimum_required(VERSION 3.16)
project(perception_3d_msgs)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConvexObject.msg"
  "msg/ConvexObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs unique_identifier_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
