# ROS 2 Foxy (Ubuntu 20.04 focal) build environment for the G1 perception stack.
#
# WHY: the G1 onboard computer runs Foxy, but this workspace was written and is
# developed on Humble (Ubuntu 22.04). Foxy cannot be installed natively on
# jammy — there are no focal debs for jammy and no foxy debs for jammy — so the
# only way to *prove* the code builds for the robot is a focal container.
#
# Foxy is EOL: packages.ros.org no longer carries it, but the frozen final
# archive at snapshots.ros.org/foxy/final does, and the stock ros:foxy-ros-base
# image already points there. Do not "fix" the apt sources.
#
# Build:  see ../../tools/foxy_docker.sh  (do not docker build by hand)
FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]

# --- apt dependencies -------------------------------------------------------
# Mirrors the union of every package.xml <depend> in ros2/src, resolved to the
# Foxy/focal deb names. Kept explicit rather than rosdep-driven so the image is
# reproducible after Foxy's rosdep keys inevitably rot.
#
# ros-foxy-rmw-cyclonedds-cpp is a DEB here, not the source build the Humble
# side uses. It has to be: rmw_cyclonedds's foxy branch (0.7.11) is written
# against CycloneDDS 0.7's `ddsi_sertopic` API, which the 0.10.2 this
# workspace pins for unitree_sdk2 deleted in favour of `ddsi_sertype`. There
# is no source pair that satisfies both, so on Foxy the ROS middleware uses
# the distro's CycloneDDS 0.7 and unitree_sdk2 keeps its own 0.10.2 — see the
# R-3 note in ros2/README.md before linking both into one process.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-vcstool \
      python3-pytest \
      python3-pytest-cov \
      python3-matplotlib \
      python3-pyqtgraph \
      python3-pyqt5 \
      python3-pyqt5.qtopengl \
      libarmadillo-dev \
      libpcl-dev \
      libssl-dev \
      libcunit1-dev \
      libyaml-cpp-dev \
      libeigen3-dev \
      libfmt-dev \
      libomp-dev \
      libpcap-dev \
      libapr1-dev \
      ros-foxy-ament-cmake \
      ros-foxy-ament-cmake-auto \
      ros-foxy-ament-cmake-gtest \
      ros-foxy-ament-lint-auto \
      ros-foxy-ament-lint-common \
      ros-foxy-diagnostic-msgs \
      ros-foxy-diagnostic-updater \
      ros-foxy-geometry-msgs \
      ros-foxy-laser-geometry \
      ros-foxy-launch \
      ros-foxy-launch-ros \
      ros-foxy-launch-testing \
      ros-foxy-launch-testing-ament-cmake \
      ros-foxy-launch-testing-ros \
      ros-foxy-message-filters \
      ros-foxy-nav-msgs \
      ros-foxy-pcl-conversions \
      ros-foxy-pcl-msgs \
      ros-foxy-rclcpp \
      ros-foxy-rclcpp-components \
      ros-foxy-rclpy \
      ros-foxy-rmw \
      ros-foxy-rmw-dds-common \
      ros-foxy-rmw-implementation \
      ros-foxy-rmw-cyclonedds-cpp \
      ros-foxy-robot-state-publisher \
      ros-foxy-rosbag2 \
      ros-foxy-rosbag2-storage-default-plugins \
      ros-foxy-rosidl-default-generators \
      ros-foxy-rosidl-default-runtime \
      ros-foxy-sensor-msgs \
      ros-foxy-std-msgs \
      ros-foxy-std-srvs \
      ros-foxy-tf2 \
      ros-foxy-tf2-eigen \
      ros-foxy-tf2-geometry-msgs \
      ros-foxy-tf2-ros \
      ros-foxy-tf2-sensor-msgs \
      ros-foxy-visualization-msgs \
      ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

# --- match the host user ----------------------------------------------------
# Build artefacts land in the bind-mounted workspace; root-owned build_foxy/
# would be unusable from the host afterwards.
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG HOST_USER=user
RUN if ! getent group ${HOST_GID} >/dev/null; then groupadd -g ${HOST_GID} ${HOST_USER}; fi \
 && if ! getent passwd ${HOST_UID} >/dev/null; then \
        useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${HOST_USER}; \
    fi \
 && echo "source /opt/ros/foxy/setup.bash" >> /home/${HOST_USER}/.bashrc \
 && echo 'export PS1="(foxy) \w\$ "' >> /home/${HOST_USER}/.bashrc

USER ${HOST_UID}:${HOST_GID}
WORKDIR /home/user/unitree_rl_mjlab/ros2
CMD ["/bin/bash"]
