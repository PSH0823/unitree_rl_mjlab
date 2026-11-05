# DPCBF velocity controller

For a fresh G1 onboard PC and remote visualization computer, complete
[`SETUP.md`](SETUP.md) first.

`DpcbfVelocity` maps the joystick to a nominal `(vx, vy, wz)` command and
runs `DPCBF-QP -> low-level locomotion ONNX` sequentially in one 50 Hz
worker. The 1 kHz FSM thread only publishes the latest joint targets.

All ROS terminals in one session must use the same `ROS_DOMAIN_ID` and RMW
configuration. If [`SETUP.md`](SETUP.md) completed successfully, the full ROS
workspace is already built. Run the following only after pulling source
changes to the DPCBF messages, adapter or plot client.

### Humble/Jazzy development or simulation computer

The plot client is source-compatible with Humble's usual Qt5 stack and
Jazzy's Qt6 stack. These distributions use the merged install created during
setup:

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash

cd ros2
colcon build --merge-install --packages-select \
    dpcbf_viz_msgs dpcbf_ros_adapter dpcbf_plot_client
cd ..

source ros2/install/setup.bash
cmake -S deploy/robots/g1 -B deploy/robots/g1/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PWD/ros2/install"
cmake --build deploy/robots/g1/build -j2
```

### Foxy G1 onboard computer

Foxy uses the isolated install created by `SETUP.md`; never switch it to
`--merge-install`. After pulling changes, build only the changed control-side
packages and then rebuild `g1_ctrl`:

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select dpcbf_viz_msgs dpcbf_ros_adapter

cd ..
source ros2/install/setup.bash
cmake --build deploy/robots/g1/build -j2
```

The first CMake configure downloads the pinned OSQP dependencies. Later C++
controller edits require only `cmake --build`. Changes to
`dpcbf/config/dpcbf_config.yaml` require a controller restart but no rebuild.
Changes under `g1_perception_bringup/config/` require rebuilding
`g1_perception_bringup`, sourcing the install space again, and restarting
perception:

```bash
# Humble/Jazzy, from the repository root
colcon --log-base ros2/log build \
    --merge-install \
    --base-paths ros2/src/g1_perception \
    --build-base ros2/build \
    --install-base ros2/install \
    --packages-select g1_perception_bringup

# Foxy, from ~/unitree_rl_mjlab/ros2
colcon build --packages-select g1_perception_bringup
```

## Simulation

Use the following environment in every terminal. Conda is needed only in the
MuJoCo terminal when required by that machine's MuJoCo installation.

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2/install/setup.bash
source ros2/sim_navigation_env.sh
```

1. Start MuJoCo:

   ```bash
   conda activate unitree_rl_mjlab   # MuJoCo terminal only, if needed
   ./simulate/build/unitree_mujoco
   ```

2. Start simulated perception in a system-Python terminal:

   ```bash
   ros2 launch g1_perception_bringup bringup.launch.py \
       source:=sim viz:=rviz
   ```

3. Start the controller:

   ```bash
   ./deploy/robots/g1/build/g1_ctrl --network=lo
   ```

   Press `2` for FixStand and `5` for DpcbfVelocity. With a joystick the
   equivalent transition is `LT + Up`, followed by `RT + Y`.

4. Open the off-board DPCBF dashboard in another system-Python terminal:

   ```bash
   ros2 run dpcbf_plot_client dpcbf_qp_view
   ```

The dashboard shows nominal/safe commands, selected obstacles, relative
velocity vectors, DPCBF `h=0` boundaries and minimum `h`. The drawing scales
come from `dpcbf/config/dpcbf_config.yaml`. The old simulator-side joystick
filter is disabled by `qp_parameters.simulate_filter_enabled: false`, so the
command is filtered only once inside `g1_ctrl`.

## Hardware

Never source `ros2/sim_navigation_env.sh` on the robot. Use the field network
environment on both the onboard computer and operator laptop.

1. Keep G1 stationary and start LiDAR, DLIO and obstacle_detector_2 on the
   onboard computer:

   ```bash
   cd ~/unitree_rl_mjlab
   source ~/.g1_net_env
   source /opt/ros/foxy/setup.bash
   source ros2/install/setup.bash
   ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
       driver:=on lio:=dlio use_rviz:=false
   ```

   Wait until both topics are continuous:

   ```bash
   ros2 topic hz /odom
   ros2 topic hz /obstacles_safe
   ```

2. Start the controller on the onboard computer. Replace `<control_nic>`
   with the Unitree control interface from `ip link`:

   ```bash
   cd ~/unitree_rl_mjlab
   source ~/.g1_net_env
   source /opt/ros/foxy/setup.bash
   source ros2/install/setup.bash

   echo "$RMW_IMPLEMENTATION"
   ./deploy/robots/g1/build/g1_ctrl --network=<control_nic>
   ```

   `RMW_IMPLEMENTATION` must print `rmw_fastrtps_cpp`. If the repository was
   cloned somewhere other than `~/unitree_rl_mjlab`, use its actual root path
   instead (for example, `/home/unitree/sang/unitree_rl_mjlab` on the current
   G1 onboard computer).

   Enter FixStand with `LT + Up`, then DpcbfVelocity with `RT + Y`.
   Numeric keys are hardware-enabled only if
   `console_fsm_control.simulation_only: false` is set in `config.yaml`.

3. On the operator laptop, open the dashboard:

   ```bash
   cd ~/unitree_rl_mjlab
   source ~/.g1_net_env
   source /opt/ros/$ROS_DISTRO/setup.bash
   source ros2/install/setup.bash
   ros2 run dpcbf_plot_client dpcbf_qp_view
   ```

An empty `/obstacles_safe` message means “zero detected obstacles” and is
valid. If the message itself stops, the command begins fading after 0.30 s.
If OSQP fails, the previous safe command is held for one 50 Hz tick and then
rate-limited toward zero; the low-level locomotion policy remains active.
The dashboard publisher is controlled by the `visualization` section of
`dpcbf/config/dpcbf_config.yaml`.

## Recording a DPCBF run

The controller publishes the joystick intent and the command actually sent
to the locomotion policy as synchronized `geometry_msgs/TwistStamped` topics
at the full 50 Hz control rate. It also publishes the raw and filtered robot
velocities at the odometry callback rate:

- `/dpcbf/nominal_velocity_command`: original joystick velocity command
- `/dpcbf/safe_velocity_command`: velocity command after DPCBF-QP and safety
  gating
- `/dpcbf/raw_robot_velocity`: body-frame velocity obtained by odom position
  differencing with the odometry message timestamp
- `/dpcbf/filtered_robot_velocity`: body-frame constant-velocity Kalman
  estimate actually used by DPCBF/eCBF

`linear.x`, `linear.y`, and `angular.z` contain sagittal velocity, lateral
velocity, and yaw rate respectively. Record the command pair together with
the state, obstacles, and complete QP/visualization sample:

```bash
cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash
source ros2/install/setup.bash

ros2 bag record -o ~/dpcbf_qp_$(date +%Y%m%d_%H%M%S) \
    /odom \
    /obstacles_safe \
    /dpcbf/nominal_velocity_command \
    /dpcbf/safe_velocity_command \
    /dpcbf/raw_robot_velocity \
    /dpcbf/filtered_robot_velocity \
    /dpcbf/plot
```

Stop recording with `Ctrl+C`. The existing hardware capture helper also
includes all five `/dpcbf/*` topics in its default topic list.

When the joystick is moved, `g1_ctrl` prints the nominal command, sensor
freshness, command scale, QP status and final safe command periodically.
`scale=0`, `odom=STALE`, `obstacles=MISSING` or `qp=FAILED` identifies why
DpcbfVelocity is standing while CustomVelocity still walks.

If you adjust a radius scale through: ros2/src/g1_perception/g1_perception_bringup/config/safety_obstacle_filter.yaml

cd ~/unitree_rl_mjlab
source /opt/ros/$ROS_DISTRO/setup.bash

cd ros2
colcon build --merge-install \
    --packages-select g1_perception_bringup
source install/setup.bash
