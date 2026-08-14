# ROS 2 Foxy Setup (the robot's onboard computer)

The G1's onboard PC runs **ROS 2 Foxy on Ubuntu 20.04**. This is the full setup
for that machine, from `git clone` to a working build.

## Why this is a separate document

The main setup guide is written around a `ROS_DISTRO` variable, so one set of
commands covers Humble, Jazzy and later. **Foxy does not fit that pattern**, for
four reasons:

| | Humble / Jazzy / … | Foxy |
|---|---|---|
| apt source | `packages.ros.org` | `snapshots.ros.org/foxy/final` (it is end-of-life) |
| signing key | current `ros.key` | the **old** key `AD19BAB3CBF125EA` |
| Ubuntu | 22.04 / 24.04 | 20.04 only |
| build | plain `colcon build --merge-install` | five packages skipped, and **no** `--merge-install` |

---

## 1. Install ROS 2 Foxy

Skip this if `ls /opt/ros` already shows `foxy` — on a G1 it will.

Foxy is end-of-life and `packages.ros.org` dropped it, but the frozen archive
still serves every package this workspace needs.

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg
curl -sSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xAD19BAB3CBF125EA" \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-snapshots-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-snapshots-keyring.gpg] http://snapshots.ros.org/foxy/final/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update && sudo apt-get install -y ros-foxy-ros-base
```

> The snapshot archive is signed with the **old** ROS key. Using the current
> `ros.key` — the one the Humble instructions use — fails at `apt-get update`
> with `NO_PUBKEY AD19BAB3CBF125EA … is not signed`.
>
> Do not "fix" this apt source any other way. The snapshot archive is the only
> place Foxy still exists.

---

## 2. Prerequisites

You need a `sudo` account, **20 GB free disk**, and an internet connection.

Working over SSH? Start `tmux` first — if the connection drops mid-build, the
build dies with it.

```bash
ssh -o ServerAliveInterval=15 <user>@<onboard-pc-ip>
tmux new -s g1          # reattach later with: tmux attach -t g1
```

### System packages

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git tmux \
    net-tools iputils-ping iproute2 \
    python3-colcon-common-extensions python3-vcstool \
    python3-pytest python3-pytest-cov python3-yaml python3-numpy \
    libarmadillo-dev libpcl-dev libyaml-cpp-dev libeigen3-dev \
    libfmt-dev libomp-dev libpcap-dev libapr1-dev \
    ros-foxy-ament-cmake ros-foxy-ament-cmake-auto ros-foxy-ament-cmake-gtest \
    ros-foxy-ament-lint-auto ros-foxy-ament-lint-common \
    ros-foxy-diagnostic-msgs ros-foxy-diagnostic-updater \
    ros-foxy-geometry-msgs ros-foxy-laser-geometry \
    ros-foxy-launch ros-foxy-launch-ros ros-foxy-launch-testing \
    ros-foxy-launch-testing-ament-cmake ros-foxy-launch-testing-ros \
    ros-foxy-message-filters ros-foxy-nav-msgs \
    ros-foxy-pcl-conversions ros-foxy-pcl-msgs \
    ros-foxy-rclcpp ros-foxy-rclcpp-components ros-foxy-rclpy \
    ros-foxy-rmw ros-foxy-rmw-dds-common ros-foxy-rmw-implementation \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-robot-state-publisher \
    ros-foxy-rosbag2 ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
    ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-std-srvs \
    ros-foxy-tf2 ros-foxy-tf2-eigen ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-ros ros-foxy-tf2-sensor-msgs \
    ros-foxy-visualization-msgs ros-foxy-xacro
```

Optional, only if this machine has a monitor and you want to run a GUI on it:

```bash
sudo apt-get install -y ros-foxy-rviz2 \
    python3-pyqtgraph python3-pyqt5 python3-pyqt5.qtopengl python3-matplotlib
```

---

## 3. Clone and fetch external sources

```bash
cd ~
git clone https://github.com/PSH0823/unitree_rl_mjlab.git
cd ~/unitree_rl_mjlab
git checkout obstacle_detection
git log --oneline -1          # note the commit in your session log

cd ~/unitree_rl_mjlab/ros2
./setup_external.sh
```

`setup_external.sh` clones the pinned external repositories into
`src/external/` and applies the recorded patches. Last line of a good run:

```
External sources ready.
```

> On a **fresh** clone it should print nothing else. If you see
> `… patch already applied`, that is a warning sign, not reassurance — the
> script prints the same line whether a patch is already in the tree or could
> not be applied at all. On a fresh clone nothing can legitimately be applied
> yet, so that message means the patch is missing and the Foxy build will fail
> later in an unrelated place. Check it by hand:
>
> ```bash
> git -C src/external/obstacle_detector_2 apply --check \
>     patches/0011-obstacle-detector-f1-foxy-portability.patch
> ```
>
> If that errors, `rm -rf src/external/obstacle_detector_2` and re-run
> `./setup_external.sh`.

---

## 4. Build — 15 packages

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash      # do not source any other workspace

colcon build \
    --packages-skip cyclonedds rmw_cyclonedds_cpp unitree_sdk2 \
                    unitree_dds_wrapper_vendor t10_dds_coexistence \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```
Summary: 15 packages finished
```

Takes **40–90 minutes** on the onboard PC. If it runs out of memory, add
`--parallel-workers 2`.

Three rules to follow exactly:

1. **Keep `--packages-skip` exactly as written.** Those five packages belong to
   the CycloneDDS and Unitree-SDK paths, which a perception session does not
   use — and `cyclonedds` in particular *cannot* build on Foxy: its `idlc`
   loads Foxy's own CycloneDDS 0.7 `libddsc` first and dies with
   `undefined symbol: DDS_XTypes_TypeObject_desc`.
2. **No `--merge-install` on Foxy.** Foxy uses the default isolated layout.
   The Humble side uses `--merge-install`; this machine does not.
3. **Source only `/opt/ros/foxy/setup.bash`.** If another workspace is sourced,
   colcon chains it as an underlay and the build breaks silently on any machine
   that lacks that path.

If it stops on one package, read `log/latest_build/<package>/stdout_stderr.log`.

---

## 5. Check the build

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash            # these two lines in every new terminal

# Foxy's `ros2 pkg executables` takes ONE package at a time
ros2 pkg executables livox_ros_driver2
ros2 pkg executables direct_lidar_inertial_odometry
ros2 pkg executables g1_perception_utils
find install -name libpcl_ros_filters.so
```

Expected:

```
livox_ros_driver2 livox_ros_driver2_node
direct_lidar_inertial_odometry dlio_map_node
direct_lidar_inertial_odometry dlio_odom_node
g1_perception_utils base_footprint_publisher
g1_perception_utils dpcbf_overlay
g1_perception_utils obstacles_marker_relay
install/pcl_ros/lib/libpcl_ros_filters.so
```

**If any of these is missing the build did not finish. Do not continue.**

---

## 6. Point the driver at the LiDAR

Skip this if the Mid-360 is **not** plugged into this computer — i.e. another
machine publishes `/livox/*` and you will launch with `driver:=off`.

```bash
ip -br addr                  # this computer's NICs
ping 192.168.123.120         # the Mid-360 at its G1 factory address
```

`ros2/src/g1_perception/g1_perception_bringup/config/MID360_config.json`
already holds the values measured on the G1 — host `192.168.123.164`, LiDAR
`192.168.123.120`. If those match this machine, there is nothing to change.

Otherwise edit two things:

- `MID360.host_net_info` — the four `cmd_data_ip` / `push_msg_ip` /
  `point_data_ip` / `imu_data_ip` fields all get **this computer's** LiDAR-side
  NIC address. Leave `log_data_ip` empty and leave every port alone.
- `lidar_configs[0].ip` — the **Mid-360's** own address.

Leave `extrinsic_parameter` at identity: the driver emits points in the sensor
frame and TF does the rest, so filling it in here applies the mount transform
twice.

Then **rebuild** — the config is an installed artefact, so editing the source
file alone changes nothing:

```bash
colcon build --packages-select g1_perception_bringup
ros2 run g1_perception_bringup config_diff.py     # must print PASS
```

---

## 7. Run

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash

./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh
# stop here unless the last line is "PREFLIGHT PASSED"

ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

**Do not touch the robot for the first 3 seconds** — DLIO is calibrating the
IMU.

Use `driver:=off` if another computer already publishes `/livox/lidar` and
`/livox/imu`; DLIO still runs.

> Run the preflight by its **source-tree path**, as written, not with
> `ros2 run` — two of its checks compare source against installed files and
> report false failures from the install prefix.

### Is the chain alive?

In a second terminal. Foxy's `ros2 topic hz` takes **one topic at a time** —
`Ctrl-C` between lines.

```bash
ros2 topic hz /livox/lidar          # 10 Hz
ros2 topic hz /livox/imu            # 200 Hz
ros2 topic hz /odom                 # 100 Hz
ros2 topic hz /points_self_filtered # 10 Hz
ros2 topic hz /scan                 # 10 Hz
ros2 topic hz /obstacles_safe       # 10 Hz

ros2 run g1_perception_bringup hw_obstacle_watch.py   # console read-out
```

If `/scan` is empty, suspect `/odom` first — the chain is
`/livox/lidar → CropBox → /points_self_filtered → (needs TF odom→base_footprint) → /scan`.

### Record a bag

```bash
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1
mkdir -p "$SESSION"
ros2 bag record -o "$SESSION/run_$(date +%H%M%S)" \
    /livox/lidar /livox/imu /odom /tf /tf_static \
    /points_self_filtered /scan /raw_obstacles \
    /tracked_obstacles /obstacles_safe /diagnostics
```

Foxy has no `--include-unpublished-topics`; list the topics explicitly.

---

## 8. Foxy-specific quirks

Things that work on Humble and silently do not here.

| | Foxy |
|---|---|
| `ros2 topic hz` | one topic per invocation |
| `ros2 pkg executables` | one package per invocation |
| `ros2 bag record` | no `--include-unpublished-topics` |
| `--merge-install` | not used for this workspace |
| `rmw_cyclonedds_cpp` | not built from source; not needed at all when the link is Fast DDS |

---

## 9. Connecting to the laptop

The onboard PC publishes; the laptop subscribes and displays. That link is
Fast DDS and is set up in **`ros2_teleop.md`**, in this directory. Do that after
the build above passes.
