# G1 Obstacle Perception

## What this code does

It turns the surroundings into **2D circles** and estimates each circle's
**radius, position and velocity**.

```
Mid-360 LiDAR  →  3D point cloud  →  remove the robot's own body
               →  flatten to a 2D laser scan
               →  group nearby points into one object, fit a circle
               →  Kalman filter per axis  →  radius / x / y / vx / vy
```

Each detected object becomes one obstacle with a fitted radius. Every obstacle
carries three independent 1D Kalman filters — one for `x`, one for `y`, one for
the radius — so it keeps the same id across frames and gets a real velocity
estimate instead of a per-frame guess.

---

## 1. Prerequisites

Ubuntu 22.04 with ROS 2 Humble. If `ls /opt/ros` does not show `humble`:

```bash
sudo apt-get update && sudo apt-get install -y curl gnupg software-properties-common
sudo add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update && sudo apt-get install -y ros-humble-ros-base
```

### Conda environment

**If you already use the `unitree_rl_mjlab` environment**, just activate it and
make sure the package is installed in editable mode:

```bash
conda activate unitree_rl_mjlab
cd ~/unitree_rl_mjlab
pip install -e .
```

**If you do not have it yet:**

```bash
conda create -n unitree_rl_mjlab python=3.11
conda activate unitree_rl_mjlab
cd ~/unitree_rl_mjlab
pip install -e .
```

> **The ROS 2 side does not use conda.** ROS 2 Humble runs on the system
> Python 3.10, and the perception nodes are only built for it. Run
> `conda deactivate` before you build or launch anything below, or put
> `export PATH=/usr/bin:$PATH` at the top of the shell. Conda is only for
> training and playing policies.

### System packages

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    python3-pytest python3-pytest-cov python3-numpy python3-yaml \
    python3-pyqtgraph python3-pyqt5 python3-pyqt5.qtopengl python3-matplotlib \
    libarmadillo-dev libboost-all-dev libpcl-dev libeigen3-dev \
    libyaml-cpp-dev libspdlog-dev libfmt-dev libssl-dev libcunit1-dev \
    libomp-dev libpcap-dev libapr1-dev \
    ros-humble-ament-cmake ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-gtest ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-diagnostic-msgs ros-humble-diagnostic-updater \
    ros-humble-geometry-msgs ros-humble-laser-geometry \
    ros-humble-launch ros-humble-launch-ros ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
    ros-humble-message-filters ros-humble-nav-msgs \
    ros-humble-pcl-conversions ros-humble-pcl-msgs \
    ros-humble-rclcpp ros-humble-rclcpp-components ros-humble-rclpy \
    ros-humble-rmw ros-humble-rmw-dds-common ros-humble-rmw-implementation \
    ros-humble-robot-state-publisher ros-humble-rviz2 \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
    ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
    ros-humble-tf2 ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros ros-humble-tf2-sensor-msgs \
    ros-humble-visualization-msgs ros-humble-xacro
```

The simulated LiDAR needs MuJoCo 3.5 or newer **on the system Python**:

```bash
/usr/bin/python3 -m pip install -U "mujoco>=3.5"
```

---

## 2. Build

### 2.1 Fetch the external sources (once)

```bash
cd ~/unitree_rl_mjlab/ros2
./setup_external.sh
```

This clones the pinned external repositories into `src/external/` and applies
the recorded patches. It is safe to re-run.

### 2.2 Build everything

```bash
cd ~/unitree_rl_mjlab/ros2
./tools/bootstrap.sh
```

One command: it builds the ROS 2 workspace, builds the simulator, and runs the
tests it has data for. Useful variants:

```bash
./tools/bootstrap.sh --skip-tests       # build only
./tools/bootstrap.sh --jobs 8           # limit parallelism if you run out of RAM
```

### 2.3 Or build the pieces by hand

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The simulator is plain CMake, not colcon:

```bash
cd ~/unitree_rl_mjlab/simulate && mkdir -p build_ros2 && cd build_ros2
cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=ON \
      -DCMAKE_PREFIX_PATH=$PWD/../../ros2/install
make -j$(nproc)
```

The walking controller, needed only if you want the robot to walk:

```bash
cd ~/unitree_rl_mjlab/deploy/robots/g1 && mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$PWD/../../../../ros2/install
make -j$(nproc)
```

---

## 3. Run

Every new terminal needs these three lines first:

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 3.1 Watch it in simulation

```bash
./src/g1_perception/g1_perception_bringup/test/run_live_w4_view.sh
```

MuJoCo and RViz both open on your desktop and **stay up until you press
Ctrl+C**. The robot walks, the LiDAR spins, and the tracked circles are drawn
over the point cloud.

```bash
run_live_w4_view.sh [scenario] [mode]     # default: W4 estimated
```

| Scenario | Obstacle field |
|---|---|
| `W1` | 6 obstacles, all stationary |
| `W2` | 6 obstacles, moving 0.5–0.8 m/s |
| `W3` | 20 obstacles, moving 0.2–0.8 m/s |
| `W4` | 90 obstacles, 0–0.8 m/s |

It needs a real display. If `DISPLAY` is unset it assumes `:1`, and it refuses
to start on a virtual/Xvfb display rather than run where you cannot see it.

### 3.2 Watch the live plot instead

```bash
./src/g1_perception/g1_perception_bringup/test/walk_plot_session.sh
```

This brings the same stack up and leaves it running. Attach the plotting client
from a **second terminal** — the script prints the exact lines to paste, with
paths already filled in:

```bash
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
```

### 3.3 Try the plot client with no simulator at all

```bash
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py synthetic:=on
```

A fake robot drives in a circle while an obstacle crosses its path. Good for
learning the display. Never run it next to a real stack — it publishes.

### 3.4 Perception only, against a running simulator

```bash
ros2 launch g1_perception_bringup bringup.launch.py source:=sim viz:=rviz
```

| Argument | Values | Meaning |
|---|---|---|
| `source` | `sim`, `hw` | simulated LiDAR or the real Mid-360 |
| `mode` | `oracle`, `shadow`, `estimated` | `estimated` = obstacles from this pipeline |
| `viz` | `off`, `rviz` | open RViz |
| `record` | `off`, `on` | record a bag |

---

## 4. What comes out

### Topics

Each stage publishes, so you can look at any point in the chain.

| Topic | Type | What it is |
|---|---|---|
| `/livox/lidar` | `PointCloud2` | raw 3D cloud from the LiDAR |
| `/points_self_filtered` | `PointCloud2` | after the robot's own body is cut out |
| `/scan` | `LaserScan` | the 2D scan the detector actually reads |
| `/raw_obstacles` | `Obstacles` | circles fitted this frame, no tracking, no velocity |
| `/tracked_obstacles` | `Obstacles` | after the Kalman filter — **has velocity** |
| `/obstacles_safe` | `Obstacles` | after the safety margin is added — **use this one** |
| `/odom` | `Odometry` | robot pose |

```bash
ros2 topic echo /obstacles_safe --once
ros2 topic hz /obstacles_safe          # 10 Hz
```

### The obstacle message

`obstacle_detector/Obstacles` carries a list of circles:

```python
from obstacle_detector.msg import Obstacles

def callback(msg):
    for c in msg.circles:
        c.uid          # id, stays the same while the obstacle is tracked
        c.center.x     # position [m]
        c.center.y
        c.velocity.x   # velocity [m/s]
        c.velocity.y
        c.radius       # radius INCLUDING the safety margin [m]
        c.true_radius  # radius as measured [m]
```

`radius` is what you plan around; `true_radius` is what the sensor saw. The
difference is the safety margin.

Positions and velocities are in the `odom` frame.

On `/obstacles_safe` the margin has two more parts on top of
`radius_enlargement`: a fixed `0.051 m` and a term that grows with the
obstacle's speed, so something moving fast is treated as bigger. Both live in
`config/safety_obstacle_filter.yaml` as `fixed_inflation` and
`latency_horizon`.

---

## 5. Parameters worth touching

In `src/g1_perception/g1_perception_bringup/config/obstacle_detector.yaml`.
Rebuild after editing: `colcon build --packages-select g1_perception_bringup`.

**Detection** — deciding what is one object:

| Parameter | Default | Effect |
|---|---|---|
| `min_group_points` | `5` | fewer points than this is ignored as noise. Lower it to see small or distant objects |
| `max_group_distance` | `0.10` | points farther apart than this belong to different objects. Raise it if one object splits into several circles |
| `max_circle_radius` | `0.60` | anything fitting larger is dropped — walls and pillars are not circles |
| `radius_enlargement` | `0.17` | safety margin added to every radius |

**Tracking** — deciding it is the same object as last frame:

| Parameter | Default | Effect |
|---|---|---|
| `min_correspondence_cost` | `0.3` | how far an obstacle may move between frames and still count as the same one. Raise it for fast-moving objects |
| `tracking_duration` | `1.0` | how long a track survives with no measurement [s] |
| `process_rate_variance` | `0.03` | higher = velocity reacts faster but is noisier |
| `measurement_variance` | `0.04` | how much you trust the sensor; lower = trust it more |

If an obstacle flickers or disappears, check `min_group_points` and
`max_group_distance` first. If the velocity lags behind a walking person, raise
`process_rate_variance` and `min_correspondence_cost`.
