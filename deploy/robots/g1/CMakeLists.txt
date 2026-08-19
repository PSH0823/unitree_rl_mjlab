cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
  ${PROJECT_SOURCE_DIR}/../../../dpcbf/include
  ${PROJECT_SOURCE_DIR}/../../../ros2/src/g1_perception/dpcbf_ros_adapter/include
)

file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/*.cpp)

add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  cnpy_lib
  ZLIB::ZLIB
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${obstacle_detector_TARGETS}
)

add_executable(g1_ctrl main.cpp)
target_link_libraries(g1_ctrl PRIVATE ${PROJECT_NAME}_lib)
