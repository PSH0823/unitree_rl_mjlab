cmake_minimum_required(VERSION 3.16)
project(livox_sdk2_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(SDK2_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../external/Livox-SDK2")
if(NOT EXISTS "${SDK2_SRC}/include/livox_lidar_api.h")
  message(FATAL_ERROR
    "Livox-SDK2 sources not found at ${SDK2_SRC}. "
    "Run: cd ros2 && ./setup_external.sh")
endif()

# Build the SDK with its own CMake and install it straight into this package's
# install prefix, so livox_ros_driver2's find_library/find_path resolve it via
# CMAKE_PREFIX_PATH. BUILD_ALWAYS keeps it honest if the checkout is re-pinned.
ExternalProject_Add(livox_sdk2
  SOURCE_DIR "${SDK2_SRC}"
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2"
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=$<IF:$<BOOL:${CMAKE_BUILD_TYPE}>,${CMAKE_BUILD_TYPE},Release>
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  BUILD_ALWAYS TRUE
  INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
)

ament_export_include_directories(include)
ament_export_libraries(livox_lidar_sdk_shared)
ament_package()
