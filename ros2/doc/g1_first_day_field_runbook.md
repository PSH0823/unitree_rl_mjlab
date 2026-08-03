# G1 first-day field runbook — perception only

**Use this on the G1 onboard PC.** The MacBook is an SSH terminal and file-transfer client only. The onboard PC runs ROS 2, the Livox driver, DLIO, perception, and every bag recorder. Do not assume the Mac has ROS 2 Humble, RViz, X11, or a compatible architecture.

**Hard boundary:** this is a perception measurement session. Never connect `/obstacles_safe` (or any perception output) to a velocity, low-level, joystick, Unitree `rt/*`, or DPCBF command path. Do not start `deploy`, `g1_ctrl`, or any DPCBF hardware actuation. The hardware entry point is deliberately isolated and the `hw_offline_gates` test checks that its launch closure has no command publisher.

> ## Field checklist — keep this page open
>
> **Current stage:** __________  **Current blocker:** __________
>
> **Do not debug on robot time.** Capture the prescribed bag/log, write the failure, and move to an independent stage or stop safely.
>
> - [ ] G1 power procedure identified
> - [ ] E-stop operator assigned
> - [ ] onboard PC IP, username and authentication confirmed
> - [ ] target architecture recorded
> - [ ] Ubuntu and ROS versions recorded
> - [ ] internet access confirmed or offline dependency plan selected
> - [ ] sudo availability recorded
> - [ ] repository cloned
> - [ ] correct branch and commit recorded
> - [ ] target build passed
> - [ ] `config_diff.py` PASS
> - [ ] actual ROS interface recorded
> - [ ] actual Livox interface recorded
> - [ ] actual G1 and Mid-360 IP addresses recorded
> - [ ] `ROS_DOMAIN_ID` recorded
> - [ ] CycloneDDS pinned to the correct NIC
> - [ ] hardware preflight passed
> - [ ] `/livox/lidar` live
> - [ ] `/livox/imu` live
> - [ ] stage-3 bag complete
> - [ ] physical extrinsic checked
> - [ ] `/odom` live and stationary drift recorded
> - [ ] axis signs verified
> - [ ] raw self-hit bags captured
> - [ ] CropBox retains external near-field obstacles
> - [ ] `/scan` validated
> - [ ] surveyed fixtures recorded
> - [ ] tracker scenarios captured
> - [ ] all six full-pipeline topics live
> - [ ] no command publisher active
> - [ ] all bags have metadata, bag info and checksums
> - [ ] robot returned to Passive and powered down safely

## 1. Safety, people, and topology

Do not power or stand the robot until the owner demonstrates the G1-specific power-on, return-to-Passive, and power-down procedures. This repository cannot identify the G1 variant, controller UI, E-stop location, controller network, or safe standing procedure for this particular robot. The E-stop operator does nothing else. Keep the robot supported or tethered for all posture work; no one is inside the fall radius.

The intended topology is:

```text
MacBook --SSH/SCP--> G1 onboard PC ---- Ethernet ---- Mid-360
                         |                  (or robot sensor switch)
                         +---- Unitree controller network, if separate
                         +---- optional lab router/switch for SSH
```

| Machine/network | Role in this runbook | Do not assume |
|---|---|---|
| Operator MacBook | SSH, `scp`, notes, optionally a browser | ROS 2, RViz, X11, or access to the LiDAR subnet |
| G1 onboard PC | **Only** runtime host: build, driver, DLIO, perception, rosbag | Its hostname, Linux account, sudo, architecture, or NIC names |
| Unitree controller computer (if distinct) | Maintains the robot; its state topics are read only in stage 2 | It is the onboard PC or shares the Livox NIC |
| Mid-360 | Raw UDP source for `/livox/lidar` and `/livox/imu` | Its current IP is the sample IP in this repo |
| Lab router/switch | Optional SSH reachability and/or sensor wiring | It routes DDS correctly without an explicit CycloneDDS choice |

Visualization is optional and is not a gate. The repository contains an RViz configuration and can launch `rviz2` **on an Ubuntu host with a verified display**, but it does not provide a Mac viewer, remote desktop, X11 setup, or a Foxglove launch. Use, in order: a monitor directly on the onboard PC; an already-working lab remote desktop; X11 forwarding only after a simple X client is verified; Foxglove only if the lab already has it working. Otherwise record bags and inspect them later on Ubuntu.

## 2. Arrival record — fill before changing anything

Fill this on paper or in an arrival note before changing the robot. Copy it into `$SESSION/arrival.md` once the session directory is created in section 5. “Wrong” means do not guess; stop the affected stage.

| Value to record | Who knows it | Used in / edit or command | If wrong |
|---|---|---|---|
| Date; operator; G1 variant; robot serial | lab owner / labels | metadata and all evidence | evidence cannot be attributed |
| Onboard hostname, IP, SSH username, authentication | lab owner / `hostname`, `ip -br addr` | Mac SSH command | no access or wrong machine |
| Target architecture; Ubuntu; ROS distribution; free disk | onboard PC | build selection / `$SESSION` | build or bags fail |
| Sudo yes/no; internet yes/no | onboard PC / owner | dependency retrieval | choose offline plan before robot time |
| G1 control NIC and IP | owner / cabling | CycloneDDS choice, stage 2 | no Unitree discovery |
| Mid-360 NIC, host IP/CIDR, sensor IP, serial, firmware | owner / label / Livox tool | `MID360_config.json`, network command | driver binds wrong address or no packets |
| ROS domain ID | controller owner | `export ROS_DOMAIN_ID=...`; T10 | DDS isolation/crosstalk |
| CycloneDDS XML path | operator | `export CYCLONEDDS_URI=file://...` | DDS binds the wrong NIC |
| Power-on, shutdown, E-stop, E-stop operator | experienced operator | safety procedure | unsafe experiment |
| Controller idle state; Passive, FixStand, RLBase procedures | experienced operator | stage 13/14 approval only | unsafe posture/motion |
| Mechanical support method | lab owner | posture/motion decision | do not perform those stages |
| RViz display method | lab IT/operator | optional visualization | use bag-only workflow |
| Bag storage path | operator | `SESSION` | disk loss or inaccessible evidence |

Also photograph the sensor mount, cabling, robot labels, and the E-stop. The repository assumes an upside-down Mid-360 mount but has not measured this physical robot.

## 3. SSH and target inspection

On the MacBook, use the values supplied by the owner; do not replace them with examples from another robot:

```bash
ssh <ssh-user>@<onboard-pc-ip>
```

On the onboard PC, run these read-only commands and save their output. They work from any directory.

```bash
uname -m
hostname
lsb_release -a || cat /etc/os-release
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
ls /opt/ros
free -h
df -h
ip -br link
ip -br addr
ip route
sudo -v
ping -c 1 github.com
```

Record whether `sudo -v` succeeds and whether the ping succeeds. If either fails, that is not a reason to improvise during robot time. With no internet, obtain a repository mirror and the required apt packages/external sources in advance; the build cannot fetch missing dependencies offline. `aarch64` is a real build risk: the repository’s emulated build did not complete all packages. Budget a target build before powering the robot.

## 4. Clone, branch, dependencies, build, and installed files

The verified remote for this checkout is `https://github.com/lim-0219/unitree_rl_mjlab_.git`; the required branch is `dpcbf_perception_ros2`. These commands are for the onboard PC and may be run from any directory. They require internet for `git clone`, `git fetch`, `vcs import`, and any missing apt packages.

```bash
cd "$HOME"
git clone https://github.com/lim-0219/unitree_rl_mjlab_.git unitree_rl_mjlab_
cd "$HOME/unitree_rl_mjlab_"
git fetch origin dpcbf_perception_ros2
git switch --track origin/dpcbf_perception_ros2
git submodule update --init --recursive
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
git status --short
cd ros2
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake libeigen3-dev libpcl-dev libarmadillo-dev \
  libboost-all-dev libssl-dev python3-colcon-common-extensions python3-vcstool
./setup_external.sh
./tools/build_target.sh --check
./tools/build_target.sh
```

There are no Git submodules in the audited checkout, so `git submodule update --init --recursive` is a verified no-op today; it is included to make a future submodule explicit rather than silently omitted. The apt commands require sudo and internet and install the exact build dependencies checked by `build_target.sh`; do not run them if the target already has a lab-managed dependency image unless its owner approves. `setup_external.sh` is executable from `ros2/`, needs `vcs` from `python3-vcstool`, imports the pinned external repositories, and applies patches. `build_target.sh` is also executable from `ros2/`, checks `/opt/ros/humble` by default (override only with a known `ROS_PREFIX`), installs nothing itself, and uses a required merged install. If offline or without sudo, stop here and select the recorded offline plan; a fresh clone cannot retrieve its pinned external source tree. Do not use `build_target.sh` from the repository root.

Record the branch and commit. If the build passes, source the exact installed workspace and run the offline gates before robot work:

```bash
cd "$HOME/unitree_rl_mjlab_/ros2"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --merge-install --packages-select g1_description g1_perception_bringup t10_dds_coexistence
colcon test-result --all --verbose
```

The relevant no-hardware gates are `t7_hw_extrinsic_guard` (xacro/DLIO extrinsics) and `hw_offline_gates` (placeholder rejection, hardware-only isolation, probes, metadata). A test failure is a build/configuration problem; fix it before the robot session.

The launch system reads installed `launch/`, `config/`, and `rviz/` files, not the source copy. After any source config or launch edit, rebuild and prove the installed copy is identical:

```bash
cd "$HOME/unitree_rl_mjlab_/ros2"
colcon build --merge-install --packages-select g1_description g1_perception_bringup
source install/setup.bash
ros2 run g1_perception_bringup config_diff.py \
  --source "$HOME/unitree_rl_mjlab_/ros2/src/g1_perception/g1_perception_bringup" \
  --install "$(ros2 pkg prefix --share g1_perception_bringup)"
```

Expected final line: `config_diff: PASS — installed copies match source`. A bare installed-script invocation intentionally returns 3 if it cannot locate a source tree; the explicit paths above make this a real source-vs-install check.

## 5. Set the real network and DDS environment

### 5.1 Mid-360 UDP configuration

`config/MID360_config.json` contains upstream samples only: host `192.168.1.5`, LiDAR `192.168.1.12`. They are **not** G1 defaults and cannot be used. The file tells the driver where to bind; it does not configure the Mid-360’s own IP. Use the lab’s verified Livox configuration method (for example, a previously working vendor tool) to discover/configure the physical sensor IP. This repository contains no command-line tool that changes the sensor IP, so do not invent one.

On the onboard PC, first identify the physical Livox NIC by cable tracing and `ip -br link`. Pick a host IP/CIDR on the same subnet that does not overlap the controller network. The following is a temporary, on-target Linux address assignment and requires sudo; use it only after confirming the interface has no needed address:

```bash
export LIVOX_IFACE='<actual-livox-interface>'
export LIVOX_HOST_CIDR='<actual-host-ip/prefix>'
sudo ip addr add "$LIVOX_HOST_CIDR" dev "$LIVOX_IFACE"
ip -br addr show dev "$LIVOX_IFACE"
```

Persistent NetworkManager/netplan setup is lab-OS-specific and is not supplied by this repository. Record the owner’s persistent method rather than copying a temporary command into production.

Edit the **source** JSON with the actual sensor IP and the exact host IP in every non-empty `MID360.host_net_info.*_ip` field. Leave `extrinsic_parameter` identity; mount geometry belongs in `g1_description/urdf/g1_mid360.xacro`, not in the Livox JSON. Then verify it before rebuilding:

```bash
export REPO="$HOME/unitree_rl_mjlab_"
export MID360_JSON="$REPO/ros2/src/g1_perception/g1_perception_bringup/config/MID360_config.json"
export LIVOX_YAML="$REPO/ros2/src/g1_perception/g1_perception_bringup/config/livox_driver.yaml"
/usr/bin/python3 "$REPO/ros2/src/g1_perception/g1_perception_bringup/test/hw_config_check.py" \
  "$MID360_JSON" "$LIVOX_YAML"
```

Expected final line: `hw_config_check: PASS`. Exit 2 means sample addresses remain. Exit 1 means, among other things, a host address is not actually assigned or a UDP port is occupied. Rebuild `g1_perception_bringup`, source `install/setup.bash`, and rerun the explicit `config_diff.py` command from section 4 after a successful source check.

### 5.2 Pin CycloneDDS to the ROS/Unitree NIC

Livox traffic is direct UDP and is governed by the JSON above. DDS carries ROS and, if applicable, Unitree state. Pin DDS to the NIC that carries the controller/ROS network; do not use `lo`, and do not assume the Livox NIC is also the DDS NIC.

Create the file below on the onboard PC, replacing only `ACTUAL_ROS_INTERFACE`. The commands create both the directory and file automatically:

```bash
mkdir -p "$HOME/.config/g1-perception"
export DDS_XML="$HOME/.config/g1-perception/cyclonedds.xml"
cat > "$DDS_XML" <<'EOF'
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface name="ACTUAL_ROS_INTERFACE"/></Interfaces></General>
    <Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>
  </Domain>
</CycloneDDS>
EOF
```

In every onboard-PC shell used below, source this environment. It creates the session directory automatically; all other files mentioned under `$SESSION` are created by the commands that write them.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/unitree_rl_mjlab_/ros2/install/setup.bash"
export REPO="$HOME/unitree_rl_mjlab_"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID='<actual-controller-domain>'
export CYCLONEDDS_URI="file://$DDS_XML"
export SESSION="$HOME/unitree_rl_mjlab_/ros2/evidence/hardware/$(date +%F)/first_day"
mkdir -p "$SESSION"
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI | tee "$SESSION/environment.txt"
```

Run the repository preflight on the onboard PC, before every initial hardware launch:

```bash
/usr/bin/env bash "$REPO/ros2/src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh" \
  2>&1 | tee "$SESSION/preflight.txt"
```

Proceed only on exit 0 and `PREFLIGHT PASSED`. It checks required installed executables, installed config files, Mid-360 JSON, a non-loopback route, and the T7 guard. It does not configure the network for you. Run the **source-tree** script as shown: the installed T7 guard currently derives its config path from its source-tree layout, so `ros2 run g1_perception_bringup g1_hw_preflight.sh` is not the reliable clone-present invocation.

## 6. Stage sequence

For each stage, record the time, robot state, prop layout, result, and blocker in `$SESSION/operator_notes.md`. Before any `hw_record.sh` invocation, run `cd "$REPO"`; its provenance collector obtains branch and commit from its current directory. `hw_record.sh` records for its requested duration, writes a bag directory, `.session.json`, `env_stage*.txt`, `baginfo_stage*.txt`, `md5_stage*.txt`, and copied configs. It records only the specified topics (or its full default list). Complete any metadata fields it reports while the robot is present.

The operator must create/fill `arrival.md`, `operator_notes.md`, `stage4_measured.md`, photographs/sketches, the stage-10 `t4_layout.yaml`, and any prop timing log. `$SESSION`, recorder bag directories, `env_stage*.txt`, bag-info/checksum files, probe JSON files, session JSON, and copied configs are created automatically by the commands that name them.

### Stage 2 — read-only Unitree DDS coexistence (before perception)

Robot powered, controller idle; no controller command operation. First inspect topic names without publishing:

```bash
ros2 topic list | tee "$SESSION/stage2_topics.txt"
ros2 topic list | grep -E '^/(lowstate|wirelesscontroller|sportmodestate|rt)' \
  | tee "$SESSION/stage2_unitree_topics.txt"
ros2 run t10_dds_coexistence t10_smoke --duration 10 --domain "$ROS_DOMAIN_ID" \
  --require-lowstate 2>&1 | tee "$SESSION/stage2_t10.txt"
```

`t10_smoke` publishes only its own `/t10_ping` and `rt/t10_smoke` self-test traffic; it sends no robot command. It must print `[t10] PASS`; `--require-lowstate` also requires received read-only `rt/lowstate`. If the topic name/type cannot be inspected by `ros2 topic hz`, record that CLI limitation rather than publishing anything. Failure means wrong domain/NIC, no controller state, or a DDS library conflict; stop this stage.

### Stage 3 — Mid-360 driver only and raw capture

Keep DLIO and perception off. In shell A:

```bash
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off \
  2>&1 | tee "$SESSION/stage3_driver.log"
```

In shell B, start the capture before inspecting data, then run the non-publishing probe:

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 3 60 /livox/lidar /livox/imu
ros2 run g1_perception_bringup hw_source_probe.py --ros-args \
  -p duration:=60.0 -p json:="$SESSION/stage3_source_probe.json" \
  2>&1 | tee "$SESSION/stage3_source_probe.txt"
```

Gate: `/livox/lidar` near 10 Hz; `/livox/imu` near 200 Hz; both frame IDs `mid360_link`; PointCloud2 has the expected 7 fields/26-byte stride; no backwards stamps; stationary IMU is plausible; record the reported clock domain. The driver creates publishers lazily: no topic after 15 s plus bind errors usually means addressing, not a dead sensor. On its failed bind path it may ignore normal termination; stop it with `pkill -9 -f 'livo[x]'`, correct the network, and rerun preflight. Do not make more than two driver-debug attempts during robot time.

### Stage 4 — physical mount/extrinsic and static TF

Measure the mount before editing anything. Start source + DLIO + the robot TF tree (the hardware-only launch also starts perception, so use this source-only sequence for a clean mount check):

```bash
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=dlio \
  2>&1 | tee "$SESSION/stage4_source_dlio.log"
```

In another shell:

```bash
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=false \
  2>&1 | tee "$SESSION/stage4_description.log"
ros2 run tf2_ros tf2_echo torso_link mid360_link \
  | tee "$SESSION/stage4_tf_mid360.txt"
ros2 run tf2_ros tf2_echo odom base_link \
  | tee "$SESSION/stage4_tf_odom_base.txt"
```

With the robot level and still, tape-measure/photograph sensor x/y/z and roll/pitch/yaw relative to the pelvis/base reference; use the stage-3 raw bag to confirm floor returns have the expected sign for the upside-down mount. The repository’s current transform is only a hypothesis for this robot. If it differs, edit `g1_description/urdf/g1_mid360.xacro`, regenerate the two derived DLIO `extrinsics/baselink2{lidar,imu}` values, run `/usr/bin/python3 "$REPO/ros2/src/g1_perception/g1_description/test/t7_hw_extrinsic_guard.py`, rebuild **both** `g1_description g1_perception_bringup`, and rerun config-diff. Never put the mount extrinsic in `MID360_config.json`.

### Stage 5 — DLIO stationary initialization and drift

The robot must not move for the first three seconds (`odom/imu/calibration/time: 3.0`) or during the ten-minute drift capture. Stop prior source/description launches first. In shell A:

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
  driver:=on lio:=dlio diagnostics:=on \
  2>&1 | tee "$SESSION/stage5_dlio.log"
```

In shell B:

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 5 600 \
  /livox/lidar /livox/imu /odom /tf /tf_static /diagnostics
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
  -p duration:=120.0 -p json:="$SESSION/stage5_tf_probe.json" \
  2>&1 | tee "$SESSION/stage5_tf_probe.txt"
ros2 topic hz /odom | tee "$SESSION/stage5_odom_hz.txt"
```

Record `/odom` rate, pose/yaw drift, and TF probe result. The target is drift below 1 cm/min and stamped TF lookup success at least 0.95. `/odom` can be near 100 Hz while `odom→base_link` TF is near the 10 Hz scan rate; that is DLIO’s implementation. A >1 m stationary pose jump: capture two more minutes, stop the stage, and do not use odometry downstream.

### Stage 6 — externally moved sign check

No self-powered motion. While supported/carried, record a forward move, left move, positive (counter-clockwise) yaw, and return to start:

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 6 180 \
  /livox/lidar /livox/imu /odom /tf /tf_static
```

Write timestamps of each move. ROS signs must be forward `+x`, left `+y`, up `+z`, counter-clockwise `+yaw`. An inverted sign is an extrinsic/frame failure: return to stage 4.

### Stage 7 — raw robot self-hit data

Stop the hardware-only launch. Clear a 1.5 m radius: within 0.8 m of the sensor there must be only robot returns. Start driver only, then make separate labelled captures for stationary nominal pose, slow arm sweep, and torso yaw/roll/pitch:

```bash
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 7 30 /livox/lidar
```

Run that capture command once per pose and rename/record each bag path printed by the tool. Offline, analyze each raw bag (the script accepts SQLite rosbag directories):

```bash
ros2 run g1_perception_bringup selfhit_analysis.py <stage7-bag-directory> \
  --radius 0.8 --json "$SESSION/stage7_selfhit.json"
```

The installed CropBox values are simulation-derived. Do not enlarge the box at the robot. If the 99.9th-percentile `|x|`/`|y|` reaches the projection’s 0.30 m `range_min`, a larger box would hide real near-field obstacles; record that a shaped/pose-aware mask is required.

### Stages 8–12 — run the full perception-only graph

For each of stages 8–12, launch the verified perception-only entry point in shell A. `use_rviz:=true` is valid only with a verified onboard display/remote desktop/X11 display; otherwise leave it false. This launch has no actuation argument.

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
  driver:=on lio:=dlio diagnostics:=on use_rviz:=false \
  2>&1 | tee "$SESSION/stage_full_launch.log"
```

**Stage 8, CropBox:** place a soft external obstacle at 0.3, 0.5, 1.0, and 1.5 m. Record raw and filtered clouds. Verify robot-body returns reduce **and** the external obstacle remains. A clean but blind output fails.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 8 120 \
  /livox/lidar /points_self_filtered /tf /tf_static /diagnostics
```

**Stage 9, height band and LaserScan:** record empty flat floor, low/medium objects, table leg/edge if relevant, and a modest supported posture change. The CropBox is in `mid360_link`; the projection height band is after transform to `base_footprint`. There is no ground-segmentation package in this hardware launch: `min_height: 0.15` is the flat-floor rejection mechanism.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 9 120 \
  /livox/lidar /points_self_filtered /scan /tf /tf_static /diagnostics
```

Validate that `/scan` is live, obstacles appear at expected range, and an empty flat floor does not create a persistent near-field ring. Do not claim uneven terrain support.

**Stage 10, static obstacle detector:** tape/survey at least three circular props at 1–3 m. Use radius no larger than 0.52 m; the detector’s circle model can drop larger objects depending on range. Record each prop’s centre in `odom`, radius, height, material, and visibility. Create `$SESSION/t4_layout.yaml` manually; it is not generated by this repository.

```yaml
match_radius: 0.5
targets:
  - {name: cyl_1, x: <measured-x>, y: <measured-y>, r: <measured-radius>}
```

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 10 120 \
  /livox/lidar /scan /raw_obstacles /tracked_obstacles /obstacles_safe \
  /odom /tf /tf_static /diagnostics
```

Targets, not pre-proven hardware acceptance: report detection probability, centre/radius errors, false positives, and delay. Never retune safety inflation to hide an extractor error.

**Stage 11, tracker:** capture static, a manually moved obstacle, two crossing obstacles, temporary occlusion, and reappearance. Record the scenario timing and surveyed geometry.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 11 120 \
  /scan /raw_obstacles /tracked_obstacles /obstacles_safe /odom /tf /tf_static
/usr/bin/python3 "$HOME/unitree_rl_mjlab_/ros2/src/g1_perception/g1_perception_bringup/test/phase4_obstacles_dump.py" \
  120 "$SESSION/stage11_obstacles.jsonl"
```

The dumper must run while the graph is live. Later run `measure_measurement_variance.py` on its JSONL. `measurement_variance: 1.0` and covariance inflation are uncalibrated for hardware; leave `use_covariance: false`.

**Stage 12, complete pipeline:** use the six exact topics below and record the end-to-end graph. Either use `hw_record.sh` or launch with `record:=on bag_path:=...`; do not use both recorders for the same bag.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 12 180 \
  /livox/lidar /points_self_filtered /scan /raw_obstacles /tracked_obstacles \
  /obstacles_safe /odom /tf /tf_static /diagnostics
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
  -p duration:=120.0 -p json:="$SESSION/stage12_tf_probe.json" \
  2>&1 | tee "$SESSION/stage12_tf_probe.txt"
/usr/bin/python3 "$HOME/unitree_rl_mjlab_/ros2/src/g1_perception/g1_perception_bringup/test/phase4_latency_probe.py" \
  60 "$SESSION/stage12_latency.txt"
```

The six topics are `/livox/lidar`, `/points_self_filtered`, `/scan`, `/raw_obstacles`, `/tracked_obstacles`, `/obstacles_safe`. Confirm each separately (the ROS CLI accepts one topic per invocation):

```bash
for topic in /livox/lidar /points_self_filtered /scan /raw_obstacles /tracked_obstacles /obstacles_safe; do
  ros2 topic info "$topic" --verbose
done | tee "$SESSION/stage12_topic_info.txt"
ros2 node list | tee "$SESSION/stage12_nodes.txt"
ros2 topic info /obstacles_safe --verbose | tee "$SESSION/stage12_safe_info.txt"
for node in /livox_lidar_publisher /dlio_odom_node /robot_state_publisher \
  /base_footprint_publisher /perception_container /hw_diagnostics; do
  ros2 node info "$node"
done | tee "$SESSION/stage12_our_node_interfaces.txt"
```

The hardware launch publishes `/obstacles_safe` but no hardware consumer is launched. Inspect the last file: none of these nodes may list a command, low-command, wireless-controller, or sport publisher. Stop immediately if `g1_ctrl`/`deploy` appears, or if a node you started publishes a command topic. Do not treat a grep that sees pre-existing controller topics as proof that this perception stack published them; use `ros2 topic info <topic> --verbose` and record publisher node names.

### Stage 13 — controlled posture changes, no walking

Only after the preceding data is captured, with support and E-stop operator, use the owner-demonstrated Unitree procedure to enter approved posture states. Record a separate bag for body sway, small pitch/yaw, arm motion, and posture transitions. The G1’s controller procedure is external to this repository; do not infer it from a button chord in old notes. Perception remains isolated and no output may control velocity.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 13 120 \
  /livox/lidar /livox/imu /odom /tf /tf_static /points_self_filtered /scan \
  /raw_obstacles /tracked_obstacles /obstacles_safe /diagnostics
```

Stop and return to Passive if DLIO/TF becomes unstable, self-hits reappear as persistent obstacles, or the support/safety condition changes.

### Stage 14 — optional low-speed motion

This stage requires explicit lab/owner approval, a controller operator, clear space, support as required, and an E-stop operator. The movement command must come only from the established Unitree controller path; this repository supplies no command and must remain unchanged. Record its exact velocity profile and controller operator. If not explicitly approved, write “not approved” and skip.

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 14 180 \
  /livox/lidar /livox/imu /odom /tf /tf_static /points_self_filtered /scan \
  /raw_obstacles /tracked_obstacles /obstacles_safe /diagnostics
```

Any unexpected motion, command publisher, odometry divergence, or perception degradation that would matter to control ends the stage immediately. This is still not DPCBF hardware actuation.

## 7. Bag completion, metadata, and offline work

For every printed bag path, verify its contents and checksum. `hw_record.sh` does this automatically, but repeat after any interrupted capture:

```bash
ros2 bag info <bag-directory>
md5sum <bag-directory>/*
```

Each `hw_record.sh` invocation writes `<bag>.session.json` and copies loaded configs. It exits after writing even if human fields are incomplete. Fill `g1_variant`, LiDAR serial/firmware, measured mount, operator, robot state, scenario, NICs, and surveyed obstacles before leaving the lab. For a full-stack automatic recorder, `g1_perception_hardware_only.launch.py record:=on bag_path:=<path>` starts `record_hw.launch.py`, which records the fixed hardware topic list and writes `<path>.session.json`; it does not create a session directory for you.

Offline—not during robot time—analyze self-hit bags, fixture errors, tracker covariance, and latency. Keep the raw bags even if a stage failed: they are the evidence required to diagnose it.

## 8. Shutdown

1. Stop rosbag (`Ctrl-C`) and wait until it finishes writing metadata.
2. Stop the launch (`Ctrl-C`).
3. If the Livox failed-bind process survives normal termination, use only the targeted command below. Then stop remaining perception processes.

```bash
pkill -9 -f 'livo[x]'
pkill -9 -f 'component_containe[r]'
pkill -9 -f 'dlio_odom_nod[e]'
ros2 node list
```

The bracketed patterns avoid matching the killing shell itself. Return the robot to Passive with the owner-demonstrated procedure, then use the documented G1 power-down sequence. Verify the bags, metadata, `ros2 bag info`, checksums, photos, and notes are copied off the onboard PC before declaring the day complete.

## Corrections embodied in this runbook

- The Mid-360 JSON’s sample IPs are placeholders; the repository cannot set the physical sensor IP.
- `ros2 topic hz` is used one topic at a time here; it is not a multi-topic gate.
- `t10_smoke` needs `--domain "$ROS_DOMAIN_ID"` and `--require-lowstate` for the hardware coexistence gate.
- RViz/X11/Foxglove is not verified for a MacBook by this repository. Bag capture is the portable visualization fallback.
- Hardware launch configuration is installed at build time. A source edit without a rebuild is not active.
