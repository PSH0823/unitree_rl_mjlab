# DPCBF hardware setup: G1 and remote computer

This guide prepares the two computers used in a hardware run.

| Computer | Typical environment | Role |
|---|---|---|
| G1 onboard PC | Ubuntu 20.04, ROS 2 Foxy | LiDAR, DLIO, obstacle detection, DPCBF-QP and locomotion policy |
| Remote computer | Ubuntu 22.04/Humble or 24.04/Jazzy | Read-only DPCBF visualization and rosbag recording |

The perception pipeline is identical to `dodger`:

```text
Mid-360 -> DLIO -> obstacle_detector_2 -> multi-object tracker
        -> safety_obstacle_filter -> /obstacles_safe -> DpcbfVelocity
```

## 1. Use the same source revision

Run on both computers:

```bash
cd ~
git clone https://github.com/PSH0823/unitree_rl_mjlab.git
cd ~/unitree_rl_mjlab
git checkout dpcbf
git pull --ff-only
git rev-parse HEAD
```

The two commit IDs must match. Verify that the branch contains the required
files before continuing:

```bash
test -f ros2/README.md
test -f ros2/src/g1_perception/g1_perception_bringup/launch/g1_perception_dpcbf.launch.py
test -f ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp
test -f deploy/robots/g1/src/State_DpcbfVelocity.cpp
test -s deploy/robots/g1/config/policy/velocity/v1/exported/policy.onnx
```

## 2. G1 onboard PC

### 2.1 Install dependencies

The G1 normally already has ROS 2 Foxy. Confirm it first:

```bash
ls /opt/ros/foxy/setup.bash
```

Install the build and perception dependencies:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git tmux net-tools iputils-ping iproute2 \
    python3-colcon-common-extensions python3-vcstool \
    python3-pytest python3-yaml python3-numpy \
    libarmadillo-dev libpcl-dev libyaml-cpp-dev libeigen3-dev \
    libfmt-dev libomp-dev libpcap-dev libapr1-dev \
    ros-foxy-ament-cmake ros-foxy-ament-cmake-auto \
    ros-foxy-diagnostic-msgs ros-foxy-diagnostic-updater \
    ros-foxy-geometry-msgs ros-foxy-laser-geometry \
    ros-foxy-launch ros-foxy-launch-ros ros-foxy-launch-testing \
    ros-foxy-message-filters ros-foxy-nav-msgs \
    ros-foxy-pcl-conversions ros-foxy-pcl-msgs \
    ros-foxy-rclcpp ros-foxy-rclcpp-components ros-foxy-rclpy \
    ros-foxy-rmw-fastrtps-cpp ros-foxy-robot-state-publisher \
    ros-foxy-rosbag2 ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
    ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-std-srvs \
    ros-foxy-tf2 ros-foxy-tf2-eigen ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-ros ros-foxy-tf2-sensor-msgs \
    ros-foxy-visualization-msgs ros-foxy-xacro
```

Complete the Unitree SDK prerequisites in
[`doc/setup_en.md`](../../../../../../../doc/setup_en.md) before building
`g1_ctrl`.

### 2.2 Fetch and build the ROS workspace

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
./setup_external.sh

colcon build \
    --parallel-workers 2 \
    --packages-skip cyclonedds rmw_cyclonedds_cpp unitree_sdk2 \
                    unitree_dds_wrapper_vendor t10_dds_coexistence \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

Foxy uses the default isolated install: do not add `--merge-install`.
Verify the important packages:

```bash
ros2 pkg prefix obstacle_detector
ros2 pkg prefix safety_obstacle_filter
ros2 pkg prefix g1_perception_bringup
ros2 pkg prefix dpcbf_viz_msgs
ros2 pkg prefix dpcbf_ros_adapter
```

### 2.3 Build `g1_ctrl`

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/foxy/setup.bash
source ros2/install/setup.bash

cmake -S deploy/robots/g1 -B deploy/robots/g1/build \
    -DCMAKE_BUILD_TYPE=Release
cmake --build deploy/robots/g1/build -j2
```

### 2.4 Check the Mid-360 network

```bash
ip -br addr
ping 192.168.123.120
```

The file
`ros2/src/g1_perception/g1_perception_bringup/config/MID360_config.json`
must contain the G1 LiDAR-side NIC address as its host address and the actual
Mid-360 address as `lidar_configs[0].ip`. Leave its extrinsic at identity;
TF supplies the physical mount transform.

If the JSON or another perception YAML is changed, install it again:

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select g1_perception_bringup
source install/setup.bash
ros2 run g1_perception_bringup config_diff.py
```

## 3. Remote computer

### 3.1 Install viewer dependencies

Install ROS 2 Humble on Ubuntu 22.04 or Jazzy on Ubuntu 24.04, then:

```bash
# Choose the one installed on this computer:
source /opt/ros/humble/setup.bash   # Ubuntu 22.04
# source /opt/ros/jazzy/setup.bash  # Ubuntu 24.04
sudo apt-get update
sudo apt-get install -y \
    python3-colcon-common-extensions python3-vcstool \
    python3-pyqtgraph python3-matplotlib \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-tf2-ros
```

### 3.2 Build the message packages and viewer

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/$ROS_DISTRO/setup.bash
./setup_external.sh

colcon build --merge-install \
    --packages-up-to obstacle_detector dpcbf_viz_msgs dpcbf_plot_client

source install/setup.bash
ros2 pkg executables dpcbf_plot_client
```

The last command must list `dpcbf_qp_view`.

## 4. Connect the two computers with Fast DDS

Create `~/.g1_net_env` separately on both computers:

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=$HOME/unitree_rl_mjlab/ros2
```

`ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` must match. `G1_WS` is a local path
and can differ. Source the file in every hardware terminal:

```bash
source ~/.g1_net_env
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
ping <other-computer-ip>
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh
```

Test multicast. Wait on the remote computer:

```bash
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh recv
```

Then send from the G1:

```bash
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh send
```

If multicast is blocked, set `G1_PEER_IP` to the other computer's address in
each `~/.g1_net_env`, then run on both sides:

```bash
source ~/.g1_net_env
source "$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/net_env.sh peers
ros2 daemon stop
```

## 5. Verify before walking

On the stationary G1:

```bash
cd ~/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/foxy/setup.bash
source ros2/install/setup.bash

./ros2/src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh
```

Do not proceed unless it ends with `PREFLIGHT PASSED`. Start perception and
leave the robot still for the first three seconds while DLIO calibrates:

```bash
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio use_rviz:=false \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

Check on the G1, one command at a time on Foxy:

```bash
ros2 topic hz /livox/lidar       # approximately 10 Hz
ros2 topic hz /livox/imu         # approximately 200 Hz
ros2 topic hz /odom              # approximately 100 Hz
ros2 topic hz /obstacles_safe    # approximately 10 Hz
```

Check and open the read-only viewer on the remote computer:

```bash
cd ~/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2/install/setup.bash
ros2 daemon stop
ros2 topic hz /odom --no-daemon
ros2 topic echo /obstacles_safe --once
ros2 run dpcbf_plot_client dpcbf_qp_view
```

After setup, follow [`RUN.md`](RUN.md) for the controller, FSM, dashboard and
bag-recording sequence.

## 6. Rules that prevent common failures

- Never source `ros2/sim_navigation_env.sh` during a hardware run.
- Both computers must use the same domain and `rmw_fastrtps_cpp`.
- Foxy builds without `--merge-install`; Humble/Jazzy use it.
- The remote viewer is read-only and is not part of the safety loop.
- An empty `/obstacles_safe` message is valid; a stale stream is not.
- After editing installed YAML/JSON, rebuild `g1_perception_bringup`, source
  the install space again, and restart perception.
