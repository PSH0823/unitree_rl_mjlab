cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

include(CTest)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dpcbf ${CMAKE_BINARY_DIR}/dpcbf)
target_include_directories(dpcbf_dynamic_obstacles PRIVATE mujoco/include)
target_link_libraries(dpcbf_dynamic_obstacles PUBLIC mujoco yaml-cpp)
if(NOT TARGET dpcbf_visualizer)
  message(FATAL_ERROR "OpenCV (core, imgproc, highgui) is required for the DPCBF visualization window")
endif()

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco PRIVATE
  dpcbf_dynamic_obstacles
  dpcbf_safety_filter
  dpcbf_visualizer
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
