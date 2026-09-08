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

### Power mode: all eight CPU cores must be online

`g1_ctrl` aborts during start-up unless every CPU core is online:

    std::vector<unsigned int>::operator[]:
    Assertion '__n < this->size()' failed.  Aborted (core dumped)

The Orin NX boots into the 15 W power mode, which takes cores 4-7 offline.
ONNX Runtime's bundled cpuinfo enumerates the cores listed in
`/sys/devices/system/cpu/present` (0-7), then reads
`/sys/devices/system/cpu/cpu<N>/topology/` for each one.  Offline cores have
no `topology/` directory, so it indexes a short vector and aborts.  This runs
in a global constructor while `libonnxruntime.so` is being loaded - before
`main()` - so no session option or thread setting avoids it, and it kills any
process that merely links the library.

Check before every field session:

```bash
cat /sys/devices/system/cpu/online   # must print 0-7
sudo nvpmodel -q                     # must print: NV Power Mode: MAXN
```

Without `sudo`, `nvpmodel -q` still reports the mode but adds two harmless
`emc_iso_cap` permission errors.

If those print `0-3` and `15W`, switch to MAXN:

```bash
sudo nvpmodel -m 0                   # answer YES when it asks to reboot
```

The reboot is not optional.  MAXN also rewrites the GPU TPC power-gating
mask, which cannot change while a desktop session holds the GPU: `nvpmodel`
reports `Error writing 240 to /sys/devices/gpu.0/tpc_pg_mask: 16` (EBUSY)
and then asks to reboot.  Declining the prompt leaves the mode unchanged.

The choice survives power cycles.  `nvpmodel -m` records it in
`/var/lib/nvpmodel/status` and the boot service restores that file;
`PM_CONFIG DEFAULT` in `/etc/nvpmodel.conf` is only the fallback used when
that file is missing, so editing the config alone does not change the boot
mode.  Both are set to `0` on this robot.

Do not use the 25 W mode (`-m 3`).  It brings all eight cores online but caps
them at 1497 MHz, below the 15 W mode's own 1651 MHz; MAXN runs all eight at
1984 MHz.  MAXN also removes the static power cap, so watch the junction
temperature on long runs - sustained thermal throttling is worse for
control-loop timing than a fixed cap:

```bash
cat /sys/class/thermal/thermal_zone8/temp   # tj, milli-degrees Celsius
```

### Session start-up

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

`LT + B` now drops Navigation straight to Passive, the same abort binding the
other states have. Passive is `kp = 0`, `kd = 3` on every joint - a damped
collapse, not a stand - so it is an abort, not a way to pause. To keep the
robot on its feet, exit with `RT + X` to CustomVelocity instead. Automatic
Passive transitions for the hard faults listed above remain active.

### Running perception on the robot: measure before partitioning cores

The all-on-robot configuration puts the Mid-360 stream and the control loop on
one eight-core Orin NX.  Two placement facts are already established on this
robot and do not need re-measuring:

* `eth0` is a single-queue NIC (IRQ 311).  Every one of its hard interrupts
  and 99.4% of `NET_RX` softirq work lands on **CPU 0**.  `irqbalance` is
  inactive and RPS is off, so nothing moves it.  The Mid-360 sends its raw
  point and IMU UDP to `eth0`, and the Unitree control DDS uses the same NIC.
* The 1 kHz FSM control thread used to be hard-pinned to CPU 0 as well.  It is
  not any more: `control_thread.cpu` in `config.yaml` defaults to `-1`, which
  lets the scheduler place it.

What is *not* established is whether that contention actually breaks the 50 Hz
policy loop's deadline on this board.  Pinning the control thread to a specific
core trades away free migration and reserves nothing, so it is a change to make
from data, not from the topology argument.  Measure first:

```bash
ros2 run g1_perception_bringup cpu_probe.sh ~/nav_diagnosis <A|B|C> 60
```

The script only reads `/proc`, sysstat and topic rates.  It publishes nothing,
changes no affinity or priority, and cannot move the robot.  Hang the robot on
the gantry, or at minimum leave it in FixStand with a zero command; it must not
walk during any of the three runs.

| Run | What is running | What it gives |
| --- | --- | --- |
| A | `g1_ctrl` only, in Navigation, no goal sent | Uncontended baseline. FSM involuntary preemptions should be ~0/s |
| B | Perception only (`driver:=on lio:=dlio`), `g1_ctrl` not started | The perception stack's real peak core-equivalent load |
| C | Both at once | The A/B delta, which is the only number that attributes anything |

Run C alone attributes nothing: without A there is no baseline for the FSM
thread's involuntary preemptions, and without B there is no way to separate the
controller's own cost from the perception stack's.

Each run writes `~/nav_diagnosis/run_<label>_<timestamp>/summary.txt`.  Read
these four lines:

* **`involuntary preemptions`** for the FSM thread.  Near zero in A and
  hundreds per second in C means CPU contention is real, and pinning the
  control thread away from CPU 0 (`control_thread.cpu: 5`) is the first change
  to try.  If it stays near zero, the contention theory is wrong and the answer
  is the policy loop's own deadline, not core placement.
* **`NET_RX delta per CPU`**.  Confirms which core is absorbing the LiDAR
  stream while the controller runs.
* **`tj_end_mC` and the per-core kHz**.  MAXN has no power cap, so a sustained
  run can throttle below 1984 MHz.  If it does, the core budget is wrong and
  the answer is less work, not a different core assignment.
* **`si/so` from vmstat**.  One page fault in the 1 kHz loop is a latency
  spike; any non-zero swap activity invalidates the rest.

The controller now reports its own deadline health, which is the measurement
the probe cannot take from outside.  With Navigation running, watch for:

```
Navigation policy loop missed its 20 ms deadline by ... (streak ..., total ..., worst ...)
```

`worst` is the running maximum overrun for the entry and is the number to size
`safety.policy_deadline.tolerance` and either escalation threshold from.  It
reads a negative value (the real margin) while the loop is healthy.  Both
escalation tiers ship disabled, so until they are set the loop is warn-only.

Two safety features also ship disabled and should be enabled only after these
runs have produced real numbers, and only after watching them once on the
gantry:

* `safety.policy_deadline.stop_streak` / `fault_streak` — clear the goal, or
  request Passive, after N consecutive missed deadlines.
* `safety.joint_target_watchdog.enabled` — fades stiffness and then requests
  Passive when the 1 kHz thread has been re-sending the same joint target for
  too long.  This is the direct guard against the frozen-mid-gait target, but
  its ladder ends in Passive, which on an upright robot is a damped collapse.

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
    | awk '{print strftime("%Y-%m-%dT%H:%M:%S%z"), $0; fflush()}' \
    | tee ~/nav_diagnosis/lidar_hz.log

cd /home/unitree/kwan/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select g1_perception_bringup
source install/setup.bash

cd /home/kist/kwan/unitree_rl_mjlab
source ~/.g1_net_env
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

ros2 bag record -o "nav_log_$(date +%Y%m%d_%H%M%S)" \
  /odom \
  /obstacles_safe \
  /navigation/cmd_vel \
  /navigation/goal \
  /navigation/stop