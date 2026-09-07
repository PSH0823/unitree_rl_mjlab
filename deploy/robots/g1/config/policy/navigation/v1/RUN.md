# Navigation runtime

Every ROS terminal must use the same `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION`.  The field setup uses `source ~/.g1_net_env`.

## Build

`g1_ctrl` links against the ROS 2 perception messages, so build the
`ros2/` workspace first, then configure and build the controller:

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2/install/setup.bash
cmake -S deploy/robots/g1 -B deploy/robots/g1/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PWD/ros2/install"
cmake --build deploy/robots/g1/build -j"$(nproc)"
```

Rebuild after changing any C++ source.  The YAML files under
`config/policy/navigation/v1/params/` are read at start-up, so changing
them (for example the `actions.base_command` ranges in `navigation.yaml`
or `commands.base_velocity.ranges` in `low_level_deploy.yaml`) only
requires restarting `g1_ctrl`, not rebuilding.

## Simulation

In every simulation terminal, first select the same loopback DDS config:

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2/install/setup.bash
source ros2/sim_navigation_env.sh
```

The optional Conda environment is only needed in the MuJoCo terminal when
that machine's graphics/runtime setup requires it.  Keep the perception,
RViz, goal-view and `g1_ctrl` terminals on the system ROS Python environment.

1. Start the ROS-enabled simulator:

   ```bash
   cd ~/unitree_rl_mjlab
   source /opt/ros/$ROS_DISTRO/setup.bash
   source ros2/install/setup.bash
   ./simulate/build/unitree_mujoco
   ```

2. Start the controller in another terminal:

   ```bash
   cd ~/unitree_rl_mjlab
   source /opt/ros/$ROS_DISTRO/setup.bash
   source ros2/install/setup.bash
   ./deploy/robots/g1/build/g1_ctrl --network=lo
   ```

   Press `2` for FixStand, `4` for CustomVelocity, then `5` for Navigation.
   Number keys only follow transitions configured in `config/config.yaml`.

3. Start simulated perception:

   ```bash
   cd ~/unitree_rl_mjlab/ros2
   source /opt/ros/$ROS_DISTRO/setup.bash
   source install/setup.bash
   ros2 launch g1_perception_bringup bringup.launch.py source:=sim viz:=rviz
   ```

4. Start the interactive goal view:

   ```bash
   ros2 run dpcbf_plot_client navigation_goal_view
   ```

   Left-click and drag to set the initial position and heading. Right-click
   stops and clears the goal. Navigation remains stopped until this first
   external goal arrives. When `enable_random_goal` is true, random simulation
   goals begin only after the externally supplied goal is fully reached. When
   random goals are disabled and `hold_goal_after_reaching` is true, the
   reached pose is retained: commands remain zero unless a nearby obstacle is
   approaching, in which case Navigation avoids it and returns to that pose.

## Hardware

Never source `ros2/sim_navigation_env.sh` in a hardware session.  The robot
and operator laptop must instead have matching `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION` values in `~/.g1_net_env` (`rmw_fastrtps_cpp` in the
field setup).  Run each command block below from the repository root
(`unitree_rl_mjlab/`); all workspace and executable paths are relative to it.

1. On the robot computer, start `g1_ctrl`.  Replace `<control_nic>` with the
   Unitree control interface reported by `ip link` (for example `enp5s0`; it
   must not be `lo`):

   ```bash
   source ~/.g1_net_env
   source /opt/ros/foxy/setup.bash
   source ros2/install/setup.bash
   ./deploy/robots/g1/build/g1_ctrl --network=<control_nic>
   ```

   Hardware console-number transitions are disabled by default. To opt in,
   set `console_fsm_control.simulation_only: false` in
   `deploy/robots/g1/config/config.yaml`, restart `g1_ctrl`, and then press `2`
   for FixStand and `4` for CustomVelocity. Number keys only
   follow transitions configured for the current state. Otherwise, use the
   joystick: `LT + Up` for FixStand, then `RT + X` for CustomVelocity. Keep the
   velocity command at zero while LiDAR/LIO initializes.

2. In another terminal on the robot computer, keep the robot stationary and
   start LiDAR/LIO/perception:

   ```bash
   source ~/.g1_net_env
   source /opt/ros/foxy/setup.bash
   source ros2/install/setup.bash
   ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
       driver:=on lio:=dlio enable_plot_bridge:=true use_rviz:=false
   ```

   Wait until `/odom` and `/obstacles_safe` are updating.  The initial LIO
   calibration must be performed while the robot is motionless.

3. On the operator laptop, use the same field domain/RMW and start the goal
   interface:

   ```bash
   source ~/.g1_net_env
   source /opt/ros/humble/setup.bash
   source ros2/install/setup.bash
   ros2 run dpcbf_plot_client navigation_goal_view
   ```

4. Press `RT + Y` on the robot joystick to enter Navigation (or press `5` when
   `console_fsm_control.simulation_only` is `false`). The robot stays at zero
   command until an external goal is received. Left-click and drag in the goal
   interface to send position and heading; right-click stops and clears the
   goal. `enable_random_goal` is ignored on hardware even if it is true in the
   Navigation YAML.

If `/odom`, TF or `/obstacles_safe` stops updating, Navigation stays selected,
clears the active goal and sends a zero velocity command. Sensor recovery does
not resume motion; the operator must send a new goal. LowState loss, persistent
bad tilt, or repeated low-level ONNX failure transitions to Passive.

Navigation intentionally has no direct operator transition to Passive. Exit
with `RT + X` to CustomVelocity first, then use `LT + B` for Passive. Automatic
Passive transitions for the hard faults listed above remain active.

### Recommended LAN split: robot driver, laptop perception

When the robot computer and operator laptop are connected by a reliable wired
LAN, it is recommended to leave only the Mid-360 driver on the robot computer
and run DLIO, obstacle perception and RViz on the more powerful laptop.  The
Mid-360 is configured to send its raw UDP packets to the robot computer, so the
driver remains there; ROS 2 then carries `/livox/lidar` and `/livox/imu` over
the LAN to the laptop.

This split is optional.  Use the all-on-robot procedure above if the LAN cannot
reliably carry the raw point cloud.  Both computers must use the same
`ROS_DOMAIN_ID`, `rmw_fastrtps_cpp`, and `ROS_LOCALHOST_ONLY=0`. If multicast
discovery is blocked, configure reciprocal `G1_PEER_IP` values and use the
repository's `net_env.sh peers` setup. Do not run a second Livox driver or a
second DLIO instance on the other computer.

1. On the stationary robot, start only the hardware driver:

   ```bash
   cd ~/unitree_rl_mjlab
   source ~/.g1_net_env
   source /opt/ros/foxy/setup.bash
   source ros2/install/setup.bash
   ros2 launch g1_perception_bringup source_hw.launch.py \
       driver:=on lio:=off
   ```

2. On the operator laptop, start DLIO, perception and RViz from the received
   ROS topics:

   ```bash
   cd ~/unitree_rl_mjlab
   source ~/.g1_net_env
   source /opt/ros/humble/setup.bash
   source ros2/install/setup.bash
   ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
       driver:=off lio:=dlio diagnostics:=on use_rviz:=true
   ```

3. Keep the robot motionless until DLIO initializes, then verify from the
   laptop that the remote sensor input and local outputs are continuous:

   ```bash
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/imu
   ros2 topic hz /odom
   ros2 topic hz /obstacles_safe
   ```

   Check one topic at a time. Expected nominal rates are approximately 10 Hz
   for `/livox/lidar`, 200 Hz for `/livox/imu`, 100 Hz for `/odom`, and 10 Hz
   for `/obstacles_safe`. Only enter Navigation after these streams are stable.
   If `/livox/lidar` or `/livox/imu` is already irregular on the laptop, first
   investigate LAN/DDS throughput. If those inputs remain regular while
   `/odom` stalls, investigate DLIO or laptop scheduling instead.


cd ~/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

mkdir -p ~/nav_diagnosis

ros2 run g1_perception_bringup hw_record.sh \
    ~/nav_diagnosis 12 1200 \
    /livox/imu \
    /odom \
    /tf \
    /tf_static \
    /scan \
    /raw_obstacles \
    /tracked_obstacles \
    /obstacles_safe \
    /diagnostics

cd ~/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
stdbuf -oL ros2 topic hz /livox/lidar --window 100 2>&1 \
    | tee ~/nav_diagnosis/lidar_hz.log

cd ~/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
stdbuf -oL ros2 topic hz /livox/lidar --window 100 2>&1 \
    | awk '{print strftime("%Y-%m-%dT%H:%M:%S%z"), $0; fflush()}' \
    | tee ~/nav_diagnosis/lidar_hz.log