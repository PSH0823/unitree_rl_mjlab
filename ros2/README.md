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

## Which machine are you setting up?

There are three, and they do not share a procedure.

### This file — a development / simulation machine

Ubuntu 22.04 with ROS 2 Humble, or Ubuntu 24.04 with ROS 2 Jazzy. Builds the
whole workspace plus the MuJoCo simulator, so you can watch the detector work
without a robot. Everything below is for this machine.

### The robot's onboard computer — [`../doc/ros2_foxy_setup.md`](../doc/ros2_foxy_setup.md)

Ubuntu 20.04 with **ROS 2 Foxy**, which is end-of-life. `ROS_DISTRO=foxy` in
the commands below will not work: Foxy needs a different apt server
(`snapshots.ros.org`), a different signing key, and a different build command —
five packages skipped and no `--merge-install`. It also runs the real Mid-360
driver and DLIO odometry, which this machine never does. That document is the
complete build for it, ending in a 15-package workspace.

### A laptop watching the robot over the network — [`../doc/ros2_teleop.md`](../doc/ros2_teleop.md)

The onboard computer publishes and the laptop subscribes. Getting the two to
see each other is its own job: both machines need the same `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and `ROS_LOCALHOST_ONLY=0` — the usual
cause of "the topic list looks right but nothing arrives". That document has
the environment file to write on each machine, the three link checks to run in
order, what to do when the network blocks multicast, and the failure table.

---

## 1. Prerequisites

Ubuntu with ROS 2. **Pick your distro once** — every command below uses this
variable, so nothing else has to change:

```bash
export ROS_DISTRO=<my_ros_distro>        # <my_ros_distro> = humble | jazzy | kilted | rolling — NOT foxy
```

Put that line in your `~/.bashrc` so new terminals inherit it.

Check what you already have with `ls /opt/ros`. If your distro is not there,
install it — the Ubuntu codename is detected automatically, so this block is
the same on every release:

```bash
sudo apt-get update && sudo apt-get install -y curl gnupg software-properties-common
sudo add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-ros-base
```

> Each ROS 2 release targets one Ubuntu release — Humble on 22.04, Jazzy on
> 24.04. Install the distro that matches the Ubuntu you are on; there are no
> Jazzy packages for 22.04 and no Humble packages for 24.04.

> **Humble on 22.04 and Jazzy on 24.04 both build and run.** Everything below
> is the same on either one except two commands, each marked
> **Ubuntu 24.04 only** where it appears. On 24.04 `libpcl-dev` drags in a full
> JRE — that is normal, if surprising.

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

> **The ROS 2 side does not use conda.** ROS 2 runs on the system Python that
> ships with your Ubuntu release, and the perception nodes are only built for
> it. Conda is only for training and playing policies.
>
> Before building or launching anything below, leave the environment properly —
> `conda deactivate` on its own is not enough, because it leaves `VIRTUAL_ENV`
> set and CMake picks the conda Python up from there:
>
> ```bash
> conda deactivate
> unset VIRTUAL_ENV CONDA_PREFIX PYTHONHOME
> ```
>
> `tools/bootstrap.sh` does this for you. You only need it when you run `colcon`
> by hand (§2.3).

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
    ros-$ROS_DISTRO-ament-cmake ros-$ROS_DISTRO-ament-cmake-auto \
    ros-$ROS_DISTRO-ament-cmake-gtest ros-$ROS_DISTRO-ament-lint-auto \
    ros-$ROS_DISTRO-ament-lint-common \
    ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-laser-geometry \
    ros-$ROS_DISTRO-launch ros-$ROS_DISTRO-launch-ros ros-$ROS_DISTRO-launch-testing \
    ros-$ROS_DISTRO-launch-testing-ament-cmake ros-$ROS_DISTRO-launch-testing-ros \
    ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclcpp-components ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-rmw ros-$ROS_DISTRO-rmw-dds-common ros-$ROS_DISTRO-rmw-implementation \
    ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
    ros-$ROS_DISTRO-rosidl-default-generators ros-$ROS_DISTRO-rosidl-default-runtime \
    ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-std-srvs \
    ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-eigen ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-visualization-msgs ros-$ROS_DISTRO-xacro
```

The simulated LiDAR needs MuJoCo 3.5 or newer **on the system Python** — not in
the conda env, which the sim LiDAR bridge never sees:

```bash
# Ubuntu 22.04
/usr/bin/python3 -m pip install -U "mujoco>=3.5"

# Ubuntu 24.04 only — its system Python refuses a plain install
/usr/bin/python3 -m pip install -U --user --break-system-packages "mujoco>=3.5"
```

`tools/bootstrap.sh` picks the right one for you. If MuJoCo is missing the build
still succeeds — the `wall_accuracy` gate is then listed as *not run* instead of
failing.

---

## 2. Build

### 2.1 Fetch the external sources (once)

```bash
cd ~/unitree_rl_mjlab/ros2
./setup_external.sh
```

This clones the pinned external repositories into `src/external/` and applies
the recorded patches. It is safe to re-run.

It is `git` and `vcstool` only — no compiler, and the same command is used on
the Foxy machine. It does read `$ROS_DISTRO`, to pick the matching branch of
`rmw_cyclonedds`, so make sure you exported it in §1 first.

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
unset VIRTUAL_ENV CONDA_PREFIX PYTHONHOME
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

Both the `unset` and `-DPython3_EXECUTABLE` matter if conda has ever been
activated in that shell — see §1.

The simulator is plain CMake, not colcon:

```bash
cd ~/unitree_rl_mjlab/simulate && mkdir -p build_ros2 && cd build_ros2
cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=ON \
      -DCMAKE_PREFIX_PATH=$PWD/../../ros2/install \
      -DPython3_EXECUTABLE=/usr/bin/python3
make -j$(nproc)
```

The walking controller, needed only if you want the robot to walk:

```bash
cd ~/unitree_rl_mjlab/deploy/robots/g1 && mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$PWD/../../../../ros2/install
make -j$(nproc)
```

### 2.4 If the build stops

| It says | Do this |
|---|---|
| `No module named 'catkin_pkg'` | conda is still in the environment — see §1, then re-run `bootstrap.sh` (it clears the build dirs that cached the wrong Python) |
| `externally-managed-environment` | you are on 24.04 and used the 22.04 MuJoCo command — see §1 |
| `node_update_mutex is private` | `setup_external.sh` ran without `ROS_DISTRO` set — export it (§1) and re-run it |
| `Too many levels of symbolic links` on `libddsc.so` | `rm -f install/lib/libddsc.so*` and `rm -rf build/unitree_sdk2 build/cyclonedds`, then rebuild |

---

## 3. Run

Every new terminal needs these three lines first:

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/$ROS_DISTRO/setup.bash
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
