cmake_minimum_required(VERSION 3.16)
project(t10_dds_coexistence)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

# ddscxx headers live under <prefix>/include/ddscxx (installed by
# unitree_sdk2's vendored thirdparty).
find_path(DDSCXX_INCLUDE dds/dds.hpp PATH_SUFFIXES ddscxx REQUIRED)

add_executable(t10_smoke src/t10_smoke.cpp)
target_include_directories(t10_smoke PRIVATE ${DDSCXX_INCLUDE})
target_link_libraries(t10_smoke unitree_sdk2)
ament_target_dependencies(t10_smoke rclcpp std_msgs)

install(TARGETS t10_smoke DESTINATION lib/${PROJECT_NAME})

ament_package()
