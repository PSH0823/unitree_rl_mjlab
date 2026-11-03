cmake_minimum_required(VERSION 3.16)
project(sim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MjState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
