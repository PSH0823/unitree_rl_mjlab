cmake_minimum_required(VERSION 3.16)
project(unitree_dds_wrapper_vendor)

find_package(ament_cmake REQUIRED)

set(WRAPPER_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../external/unitree_dds_wrapper/cpp/include")
if(NOT EXISTS "${WRAPPER_INCLUDE}/unitree/dds_wrapper/common/Publisher.h")
  message(FATAL_ERROR
    "unitree_dds_wrapper cpp headers not found at ${WRAPPER_INCLUDE}. "
    "Run: cd ros2 && vcs import src < deps.repos")
endif()

install(DIRECTORY "${WRAPPER_INCLUDE}/" DESTINATION include)

ament_export_include_directories(include)
ament_package()
