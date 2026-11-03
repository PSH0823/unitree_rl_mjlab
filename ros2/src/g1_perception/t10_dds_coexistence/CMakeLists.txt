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

# The C CycloneDDS headers that ddscxx's TopicTraits/Topic templates expand
# against (ddsi_sertype, dds_create_topic_sertype, the XCDR representation
# ids). Located by a 0.10-only header ON PURPOSE.
#
# On Foxy this is the difference between building and not. Foxy's middleware
# is the ros-foxy-rmw-cyclonedds-cpp deb, which drags ros-foxy-cyclonedds 0.7
# headers into /opt/ros/foxy/include; 0.7 predates ddsi_sertype and the
# templates fail to instantiate. unitree_sdk2 installs its own 0.10.2 copy,
# and this target must see THAT one first — hence BEFORE, which also outranks
# the ament dependency includes (added as -isystem, i.e. searched last).
# Harmless on Humble, where every CycloneDDS in the prefix is already 0.10.2.
find_path(CYCLONEDDS_C_INCLUDE dds/ddsi/ddsi_sertype.h REQUIRED)

add_executable(t10_smoke src/t10_smoke.cpp)
target_include_directories(t10_smoke BEFORE PRIVATE
  ${CYCLONEDDS_C_INCLUDE} ${DDSCXX_INCLUDE})
target_link_libraries(t10_smoke unitree_sdk2)
ament_target_dependencies(t10_smoke rclcpp std_msgs)

install(TARGETS t10_smoke DESTINATION lib/${PROJECT_NAME})

ament_package()
