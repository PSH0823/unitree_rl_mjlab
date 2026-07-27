cmake_minimum_required(VERSION 3.16)

include(FetchContent)

# Avoid network checks on every incremental build and pin Abseil instead of
# inheriting osqp-cpp's moving origin/master dependency.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
FetchContent_Declare(
  abseil-cpp
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG 07651e4145e496fa8e5b3b852412cd608645e8cd
)
FetchContent_MakeAvailable(abseil-cpp)

set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  osqp-cpp
  GIT_REPOSITORY https://github.com/google/osqp-cpp.git
  GIT_TAG 43433736334d6b515ea4b0247156fea9e56c0d3f
)
FetchContent_MakeAvailable(osqp-cpp)

add_library(dpcbf_dynamic_obstacles STATIC
  src/dynamic_obstacles.cpp
)

add_library(dpcbf_safety_filter STATIC
  src/dpcbf_safety_filter.cpp
)

target_include_directories(dpcbf_dynamic_obstacles
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(dpcbf_dynamic_obstacles PUBLIC cxx_std_17)

target_include_directories(dpcbf_safety_filter
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(dpcbf_safety_filter PUBLIC cxx_std_17)
target_link_libraries(dpcbf_safety_filter PUBLIC osqp-cpp yaml-cpp)

add_executable(dpcbf_rollout_evaluator
  parameter_optimization/dpcbf_rollout_evaluator.cpp
)
target_compile_features(dpcbf_rollout_evaluator PRIVATE cxx_std_17)
target_link_libraries(dpcbf_rollout_evaluator PRIVATE dpcbf_safety_filter yaml-cpp)

find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
if(OpenCV_FOUND)
  add_library(dpcbf_visualizer STATIC src/dpcbf_visualizer.cpp)
  target_include_directories(dpcbf_visualizer
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${OpenCV_INCLUDE_DIRS}
  )
  target_compile_features(dpcbf_visualizer PUBLIC cxx_std_17)
  target_link_libraries(dpcbf_visualizer
    PUBLIC dpcbf_safety_filter yaml-cpp
    PRIVATE ${OpenCV_LIBS}
  )
endif()

option(DPCBF_BUILD_TESTS "Build DPCBF numerical tests" ON)
if(DPCBF_BUILD_TESTS)
  add_executable(dpcbf_safety_filter_test tests/dpcbf_safety_filter_test.cpp)
  target_link_libraries(dpcbf_safety_filter_test PRIVATE dpcbf_safety_filter)
  enable_testing()
  add_test(
    NAME dpcbf_safety_filter_test
    COMMAND dpcbf_safety_filter_test
            ${CMAKE_CURRENT_SOURCE_DIR}/config/dpcbf_config.yaml
  )
endif()
