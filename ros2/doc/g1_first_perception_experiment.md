# First G1 perception experiment — exact runbook

**Audience: a first-time G1 operator.** Every stage below gives its purpose,
preconditions, the exact commands, the expected output, the success criterion,
the failure symptoms, the stop condition, and the files to save.

**This session is perception-only.** No perception output reaches the robot's
velocity command. Nothing here starts the controller. The stack in
`g1_perception_hardware_only.launch.py` publishes no command topic and cannot,
and `hw_offline_gates` asserts that on every build. See §9 for what the later
DPCBF phase needs and why it is not this session.

**Read first:**
[`g1_hardware_preflight.md`](g1_hardware_preflight.md) — the information you
must have before the robot is powered; and
[`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) — what is verified
and what is not.

**Session rule (carried from the 5B checklists, and it is the right rule):
capture, do not debug.** Every number is re-derivable offline from a bag. If a
stage does not produce its bag, the stage failed — write that down and move
on. Do not troubleshoot on robot time.

---

## Environment block — paste at the top of every shell

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<the robot's; record it>
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml     # pinned to the ROS NIC
export SESSION=~/unitree_rl_mjlab_/ros2/evidence/hardware/$(date +%F)/s1
mkdir -p "$SESSION"
```

**`use_sim_time` is false everywhere on hardware** and is not an argument on
the hardware launch. There is no `/clock`; a node left `true` never advances
its clock and its timers never fire.

**Every config is an installed artefact.** After editing any YAML, launch file
or RViz layout:

```bash
cd ~/unitree_rl_mjlab_/ros2
colcon build --merge-install --packages-select g1_perception_bringup
source install/setup.bash
ros2 run g1_perception_bringup config_diff.py     # must print PASS
```

---

## Stage 0 — robot familiarity, no software

**Purpose.** Know what you are standing next to before anything is running.

**Preconditions.** Robot present. Someone who has operated it before, or its
manual, available.

**Do (no commands):**

1. Locate and operate the **power controls**. Write down the sequence.
2. Locate the **E-stop**. Press it once with the robot powered but on its
   stand, and confirm what happens. Assign a person to it for the session.
3. Identify every **network port** on the robot and what is plugged into it.
4. Identify the **onboard PC** and how you get a shell on it.
5. Identify the **Mid-360**: its power lead, its Ethernet lead, its mount.
6. Establish whether the robot is **mechanically supported** (stand, gantry,
   tether) and whether it can be for the whole session.
7. Read the **controller's current operating state** (Passive / damping /
   FixStand / off) and how to return it to Passive.
8. Rehearse the **safe shutdown** procedure out loud.
9. State out loud, and confirm with everyone present: **this session sends no
   commands to the robot.**

**Success criterion.** Every item above has a written answer in
`$SESSION/stage0.md`.

**Failure symptom.** "We'll find the E-stop if we need it."

**Stop condition.** No E-stop, or no assigned E-stop operator → the session
does not start.

**Files to save.** `$SESSION/stage0.md`, photographs of the mount and cabling.

---

## Stage 1 — target PC and network inspection

**Purpose.** Prove the machine that will run perception is the machine you
think it is, on the network you think it is.

**Preconditions.** Stage 0 complete. Shell on the target. Workspace built —
if the target is aarch64 this is **not** a formality (see the audit §2.2).

**Commands:**

```bash
uname -m
lsb_release -a
ip -br addr
ip route
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI
df -h "$SESSION"

ros2 run g1_perception_bringup g1_hw_preflight.sh 2>&1 | tee "$SESSION/preflight.txt"
```

**Expected output.** Sections 0–8 with `ok` lines; final `PREFLIGHT PASSED`.

**Success criterion.** Preflight exit **0**, and specifically:

- correct ROS network interface, not loopback;
- LiDAR on its own subnet with a route that is not `dev lo`;
- **no placeholder IPs**;
- no IP collision between the Livox subnet and the control network;
- ≥ 20 GB free where bags go;
- `config_diff.py` PASS (installed == source).

**Failure symptoms and what they mean.**

| Exit / line | Meaning |
|---|---|
| exit 2, `PLACEHOLDER NETWORK CONFIGURATION` | `MID360_config.json` still has upstream sample IPs. Fill in this robot's, rebuild, re-run |
| `RMW_IMPLEMENTATION=<unset>` | the environment block was not sourced in this shell |
| `CycloneDDS is pinned to loopback` | topics will be visible and carry no data off-box |
| `installed configuration differs from source` | you would run the **old** numbers; rebuild |
| `<pkg>/<exe> not installed` | incomplete build; go back to `tools/build_target.sh` |

**Stop condition.** Any hard FAIL. Do not proceed on a failed network
preflight — every later symptom will be misattributed to the sensor.

**Files to save.** `$SESSION/preflight.txt`, the raw command outputs above.

---

## Stage 2 — Unitree SDK2 and ROS 2 coexistence

**Purpose.** Confirm both DDS worlds work on the chosen interface, and that
neither suppresses the other. **Publishes no robot command.**

**Preconditions.** Stage 1 passed. Robot powered, controller in its normal
idle state.

**Commands:**

```bash
# 1. ROS 2 discovery works at all
ros2 topic list                     | tee "$SESSION/stage2_topics_before.txt"
ros2 doctor --report 2>/dev/null | head -40 | tee "$SESSION/stage2_doctor.txt"

# 2. Unitree state is being received (READ ONLY — no publisher)
ros2 topic list | grep -E '^/(rt|lowstate|sportmodestate)' \
                                    | tee "$SESSION/stage2_unitree_topics.txt"
ros2 topic hz /lowstate             # or whatever name step 2 actually found

# 3. the coexistence smoke test (links SDK2 and rclcpp in one process)
ros2 run t10_dds_coexistence t10_smoke | tee "$SESSION/stage2_t10.txt"

# 4. which CycloneDDS is actually loaded
ldd "$(ros2 pkg prefix rmw_cyclonedds_cpp)/lib/librmw_cyclonedds_cpp.so" \
    | grep -i ddsc      | tee "$SESSION/stage2_ddslibs.txt"
```

**Expected output.** ROS topics listed; at least one Unitree state topic
present and ticking; `t10_smoke` completing; exactly **one** `libddsc.so.0` on
the link line (mitigation R-3 — the whole workspace is built against one
CycloneDDS).

**Success criterion.** Both worlds visible simultaneously on the chosen
interface, and `t10_smoke` passes.

**Failure symptoms.**

- Unitree topics absent → wrong `ROS_DOMAIN_ID`, or the SDK uses a different
  interface than `CYCLONEDDS_URI` pins.
- `ros2 topic hz` errors on a Unitree message type → the CLI cannot resolve
  `unitree_hg` types. Known, recorded in Phase 4; **not** a coexistence
  failure. Note it and move on.
- Two `libddsc` versions → R-3 violation; stop and rebuild.

**Stop condition.** ROS 2 discovery not working at all. **Do not publish
robot commands at any point in this stage.**

**Files to save.** All four `tee` outputs.

---

## Stage 3 — Mid-360 driver only

**Purpose.** The first real sensor data. Nothing else runs.

**Preconditions.** Stages 1–2 passed. Nothing else using the LiDAR network.
Perception stack **not** running.

**Commands:**

```bash
# shell A — driver only, no DLIO, no perception
ros2 launch g1_perception_bringup source_hw.launch.py \
    driver:=on lio:=off 2>&1 | tee "$SESSION/stage3_driver.log"

# shell B — RECORD FIRST, look second
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 3 60 \
    /livox/lidar /livox/imu

# shell B — then the probe
ros2 run g1_perception_bringup hw_source_probe.py --ros-args \
    -p duration:=60.0 -p json:="$SESSION/stage3_source_probe.json" \
    2>&1 | tee "$SESSION/stage3_source_probe.txt"
```

**Expected output** (from the probe, all in one report):

```
  topic                        /livox/lidar
  count                        ~600
  rate_hz                      ~10.0
  stamp_regressions            0
  clock_domain                 host-clock (median offset … ms, stable)
                               ── or ── sensor-clock … (RECORD WHICH)
  ...
  points_per_frame_median      (Mid-360 ≈ 20 000 at 10 Hz — Q-3)
  finite_fraction_min          1.0
PROBLEMS: none detected by this probe
```

**Success criterion (all of):**

- `/livox/lidar` at 10 ± 0.5 Hz, `/livox/imu` present at ~200 Hz;
- `frame_id` **`mid360_link`** on **both** topics (`livox_frame` on the IMU
  means patch 0005 did not reach the build in use);
- 7 fields, `point_step` 26, no layout deviation reported;
- `stamp_regressions` = 0 on both;
- `finite_fraction_min` ≈ 1.0;
- IMU median |accel| ≈ 9.81, median |gyro| ≈ 0 while stationary;
- `clock_domain` **recorded** — this is the §14.3 answer.

These are expectations, not acceptance by themselves. A clean probe is the
absence of the failures the probe can see.

**Failure symptoms.**

| Symptom | Meaning |
|---|---|
| **no `/livox/*` topic at all** after 15 s | SDK bind failure, not a dead LiDAR. The driver does not exit and ignores SIGINT/SIGTERM. `pkill -9 -f livo[x]`, fix the IPs, re-run the preflight. Two attempts, then abort the stage |
| topic present, 0 Hz | different failure — device state or firmware. Save the driver stdout, move on |
| `frame_id` wrong | `livox_driver.yaml` did not reach the node — stale install prefix |
| `stamp_regressions` > 0 | **stop**: tf2 and every message filter treat a backwards stamp as a time jump |
| large/drifting `clock_domain` offset | LiDAR is on its own or a PTP clock while ROS is on the host clock. Record it; it changes what §9.3 must resolve |
| `arrival_gaps` | packet loss — check `ip -s link show <iface>` |
| points/frame far below expectation | Q-3; record, do not tune |

**Stop condition.** Wrong frame IDs, backwards stamps, excessive message age,
unstable rate, unexpected point layout, implausible IMU, or significant packet
loss.

**Files to save.** `stage3_driver.log`, the raw bag (**record before you look
— this bag is the input to several offline analyses**), `stage3_source_probe.{txt,json}`,
`env_stage3.txt`, `baginfo_stage3.txt`, `md5_stage3.txt`.

---

## Stage 4 — static TF and physical extrinsic verification

**Purpose.** Establish that the published sensor pose matches the physical
mount. Everything downstream is measured in a frame this stage defines.

**Preconditions.** Stage 3 passed. Robot standing static on a level floor,
foot positions marked with tape so the pose is reproducible.

**Commands:**

```bash
# shell A — robot description only
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=false

# shell B
ros2 run tf2_ros tf2_echo base_link torso_link    | tee "$SESSION/stage4_tf_torso.txt"
ros2 run tf2_ros tf2_echo torso_link mid360_link  | tee "$SESSION/stage4_tf_mid360.txt"
ros2 run tf2_ros tf2_echo base_link mid360_link   | tee "$SESSION/stage4_tf_full.txt"
ros2 run tf2_tools view_frames                    # writes frames.pdf
```

**Physically measure**, with a tape measure, from the pelvis origin to the
sensor: **x, y, z, roll, pitch, yaw**. Record in `$SESSION/stage4_measured.md`
with a sketch.

**Expected output.** `base_link → mid360_link` ≈ `(-0.0037, 0.00003, 0.4724)`
with roll = π (the mount is **upside down**, H-1/H-2), pitch 0.000892.

**Cross-check from data** (using the stage-3 bag, offline): fit a plane to the
ground returns in `mid360_link`. With the upside-down mount the ground sits at
**positive** sensor-frame z and the plane normal is ≈ **+z**. If it comes out
the other way, the mount is not H-1 and every downstream number is suspect.

**Success criterion.** Tape measurement agrees with the published TF within
your measurement uncertainty (state it — a tape measure on a robot is ±5 mm at
best), **and** the floor plane has the expected sign.

**Failure symptom.** A z offset ≈ 0.47 m in the wrong direction, or a plane
normal pointing the wrong way, means the roll = π assumption is wrong for this
unit.

**If the mount differs — the correction procedure, in this order:**

1. edit **`g1_description/urdf/g1_mid360.xacro`** — the single source of truth;
2. regenerate/update `config/dlio.yaml`'s `extrinsics/baselink2{lidar,imu}`
   from it (they are DERIVED, never hand-edited independently);
3. run the guard: `ros2 run g1_description t7_hw_extrinsic_guard.py` → PASS;
4. `colcon build --merge-install --packages-select g1_description g1_perception_bringup`;
5. `source install/setup.bash`; restart the launch;
6. `config_diff.py` → PASS.

**Do not hand-edit both sources.** DLIO has no TF listener: its extrinsics
parameters *are* its definition of `base_link`, so a stale copy silently
redefines the frame instead of producing a visible conflict. Also **do not**
put the extrinsic into `MID360_config.json`'s `extrinsic_parameter` — that
applies it twice.

**Stop condition.** A clearly incorrect extrinsic. Do not continue.

**Files to save.** the three `tf2_echo` dumps, `frames.pdf`,
`stage4_measured.md`, the plane-fit result, the sketch/photograph.

---

## Stage 5 — DLIO stationary initialisation

**Purpose.** Find out whether odometry exists and is stable before anything
depends on it.

**Preconditions.** Stage 4 passed. **The robot must remain completely still
for the whole stage, including the first 3 s** — DLIO calibrates IMU bias over
`odom/imu/calibration/time: 3.0` and a moving robot poisons it.

**Commands:**

```bash
# shell A
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio 2>&1 | tee "$SESSION/stage5_dlio.log"

# shell B — at least 10 minutes
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 5 600 \
    /livox/lidar /livox/imu /odom /tf /tf_static /diagnostics

# shell C — while it records
ros2 topic hz /odom
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
    -p duration:=120.0 -p json:="$SESSION/stage5_tf_probe.json" \
    2>&1 | tee "$SESSION/stage5_tf_probe.txt"
ros2 topic echo /diagnostics --once   | tee "$SESSION/stage5_diagnostics.txt"
top -b -n 3 | head -25                | tee "$SESSION/stage5_cpu.txt"
```

**Expected output.**

- `/odom` ≈ 100 Hz;
- TF `odom→base_link` at **scan rate ~10 Hz** — that is DLIO's design, not a
  fault (the broadcasts live in the per-scan thread, not the 100 Hz timer);
- the first ~30 clouds dropped with *"timestamp … earlier than all the data in
  the transform cache"* — the expected startup transient;
- diagnostics `perception/dlio` OK, `perception/tf` OK.

**Measure and record** (offline from the bag): `/odom` rate, TF rate,
stationary x/y/z drift, yaw drift, roll/pitch stability, CPU, memory,
timestamp age, calibration behaviour.

**Success criterion.** **Drift < 1 cm/min** over ≥ 10 minutes, computed as
‖p(t) − p(0)‖ / t from the recorded `/odom`; no pose jumps; roll/pitch stable;
TF present at the LiDAR stamps (`hw_tf_probe` success fraction ≥ 0.95).

**Do not assume meaningful odometry merely because `/odom` exists.** It exists
as soon as DLIO starts; the first stamps are literally 0.

**Failure symptoms.** No initialisation; violent pose jumps; upside-down
orientation; a large z offset; drift that grows; inconsistent timestamps; TF
discontinuities; an unreasonably low publish rate.

**Stop condition.** Position jumps > 1 m while stationary → record 2 more
minutes for offline diagnosis, then stop the stage.

**Files to save.** the ≥10 min bag, `stage5_dlio.log`,
`stage5_tf_probe.{txt,json}`, `stage5_diagnostics.txt`, `stage5_cpu.txt`.

---

## Stage 6 — externally moved sensor validation

**Purpose.** Check the **signs** of odometry before anything moves under its
own power. A stationary test cannot detect an inverted axis.

**Preconditions.** Stage 5 passed. The robot can be safely moved by hand, or
carried, or moved on a stand. **The robot does not move under its own power.**

**Commands:**

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 6 180 \
    /livox/lidar /livox/imu /odom /tf /tf_static
# then, announcing each into the log as it happens:
#   1. slow translation forward ~1 m, pause
#   2. slow translation left ~1 m, pause
#   3. slow yaw rotation +90 deg, pause
#   4. return to the starting pose
ros2 run tf2_ros tf2_echo odom base_link      # watch live while moving
```

**Expected output.** Moving the robot **forward** increases `odom` **x**;
**left** increases **y**; **up** increases **z**; a **counter-clockwise seen
from above** rotation increases yaw. This is the ROS convention (REP-103,
x-forward y-left z-up, right-handed).

**Success criterion.** All four signs correct, and returning to the start
returns the pose approximately to the origin (loop error is a measurement, not
a gate).

**Failure symptom.** Any inverted sign — most likely an extrinsic rotation
error from stage 4, not a DLIO bug.

**Stop condition.** Inverted axes or signs. Do not proceed; go back to stage 4.

**Files to save.** the bag, a written log of what was done at what time, the
loop-closure error.

---

## Stage 7 — raw self-hit capture

**Purpose.** Find out where the robot's own body appears in the sensor frame,
on real hardware. The current CropBox is a **simulation-derived interim**.

**Preconditions.** Stages 1–3 passed. **Perception stack NOT running** — the
whole point is unfiltered clouds. Clear ≥ 1.5 m radius around the robot so
that everything within 0.8 m is the robot itself.

**Commands:**

```bash
# driver only — no CropBox in the graph at all
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off

# one bag per configuration, ≥30 s each, announced into the log
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 7 30 /livox/lidar /tf_static
```

Configurations to capture, at minimum:

1. powered but not standing, **where safe**;
2. Passive (damping);
3. FixStand;
4. arms in the nominal pose;
5. representative arm positions (slow full-range sweep of each arm);
6. representative torso pitch (waist yaw/roll/pitch through range);
7. wrists brought near the LiDAR — the case simulation flagged.

**Offline analysis (after the session, not during):**

```bash
ros2 run g1_perception_bringup selfhit_analysis.py "$SESSION/stage7_<cfg>" \
    --radius 0.8 --json "$SESSION/stage7_selfhit.json"
```

**Expected output.** Near-range returns clustered where arms and torso are;
nothing else inside 0.8 m.

**Success criterion.** A written report identifying returns from: head shell,
torso, shoulders, wrists, thighs, knees, feet, cables and brackets — with, for
each, the point locations in `mid360_link`, distance from the robot,
persistence across frames, and **which mechanism would remove it**: geometry
masking, CropBox, `range_min`, or the height filter.

**Do not immediately enlarge the CropBox.** Quantify first. The decisive
cross-check the analysis prints: **if the 99.9th percentile |x| or |y|
approaches `range_min` (0.30 m), the arms are reaching into the detection band
and the answer is a shaped or pose-aware mask, not a bigger box.** A bigger box
buys self-filtering by making the robot blind at close range.

**Failure symptom.** Returns at radii that no box can exclude without eating
the near field.

**Stop condition.** None — this stage only records.

**Files to save.** one bag per configuration with the configuration named in
the filename, `stage7_selfhit.json`, the written report, photographs of each
pose.

---

## Stage 8 — CropBox-only validation

**Purpose.** Show the self-filter removes the robot **and keeps real
near-field obstacles**.

**Preconditions.** Stage 7 captured. A soft external object available.

**Commands:**

```bash
# source + TF + the perception container (CropBox is its first stage)
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio

# compare in and out
ros2 topic hz /livox/lidar /points_self_filtered
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 8 60 \
    /livox/lidar /points_self_filtered /tf /tf_static /diagnostics
```

Place a soft object at **0.4, 0.6, 0.8, 1.0 and 1.5 m** from the robot,
30 s each, announced into the log.

**Expected output.** `/points_self_filtered` at the same 10 Hz with fewer
points; diagnostics `perception/self_hit` OK.

**Success criterion — BOTH halves, and the second is the one that gets
forgotten:**

1. robot-body returns are removed;
2. the external object is **still present** at every distance ≥ `range_min`.

**Failure symptom.** A CropBox that removes all near-field data. That is not a
pass; it is blindness that looks like cleanliness.

**Stop condition.** If (1) and (2) cannot both hold with a box, stop and
record that a shaped/pose-aware mask is required — a design change, not a
tuning change.

**Files to save.** the bag, point-count-in/out per frame, the per-distance
object visibility table, `stage8_diagnostics.txt`.

---

## Stage 9 — height band and LaserScan validation

**Purpose.** Establish what the 2-D projection actually sees on a real floor.

**Preconditions.** Stage 8 passed. Flat floor.

**The semantics, stated exactly** (they are easy to get wrong):

- **CropBox bounds are in `mid360_link`** (the cloud frame; no TF involved).
- **`min_height` / `max_height` are applied after transforming to
  `base_footprint`.**
- **`range_min` is horizontal distance in `base_footprint`.**
- **There is no Patchwork++ and no ground segmentation.** `min_height: 0.15`
  is the only floor rejection, it is a height band, and it is valid on flat
  ground only.

**Commands:**

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 9 60 \
    /livox/lidar /points_self_filtered /scan /tf /tf_static /diagnostics
```

Scenes, ≥30 s each: empty flat floor; a known low obstacle; a medium-height
cylinder; a table leg; a table edge/overhang if relevant; the robot's own legs
and feet; moderate body pitch; a small floor slope **if safe**.

**Measure:** occupied scan-bin fraction, floor-return fraction, self-return
fraction, detection loss by object height, flicker across frames.

**Success criterion.** On an empty flat floor the scan is essentially empty
inside the room; objects taller than the band's lower edge appear at the right
range; the near-field ring is not dominated by floor returns (diagnostics
`perception/floor_artifact` OK).

**Failure symptoms.** A persistent ring of near-field returns (floor entering
the band, or body pitch tilting the band into the floor); objects vanishing as
the body pitches.

**Stop condition.** None, but **do not test rough terrain as a supported
feature.** It is not one, and a rough-terrain capture presented as a result
would be a false claim.

**Files to save.** the bag, the per-scene metric table, RViz screenshots.

---

## Stage 10 — obstacle extractor validation

**Purpose.** First real detector error numbers, against surveyed geometry.

**Preconditions.** Stage 9 passed. Robot static in the taped pose.
**Surveyed circular fixtures**: ≥ 3 cylinders, at least one with r ≥ 0.30 m,
and **every prop r ≤ 0.52 m** — above that the circle-fit gate drops them at
range (measured: r = 0.55 m detects at 2 m and is dropped at 3 and 4 m).

Survey convention: measure to the **prop face**, centre = face distance + r
along the bearing. Record in `$SESSION/t4_layout.yaml`:

```yaml
match_radius: 0.5
targets:
  - {name: cyl_1m,     x:  0.813, y:  0.813, r: 0.15}
  - {name: cyl_2m,     x: -1.520, y:  1.520, r: 0.15}
  - {name: blocker_3m, x:  2.333, y: -2.333, r: 0.30}
```

**For each fixture record:** true centre, true radius, distance, visibility
(full arc / partly occluded), height, material.

**Commands:**

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 10 60 \
    /livox/lidar /scan /raw_obstacles /tracked_obstacles /obstacles_safe \
    /odom /tf /tf_static /diagnostics
```

Repeat at 1, 2 and 3 m (or place all three at once and get one bag).

**Offline — the same harness as the simulation gate, pointed at the hardware
bag:**

```bash
T4_BAG="$SESSION/stage10_<t>" T4_LAYOUT="$SESSION/t4_layout.yaml" \
T4_USE_SIM_TIME=false \
  launch_test src/g1_perception/g1_perception_bringup/test/test_detection_static.launch_test.py
```

**Measure:** detection probability, centre error, fitted-radius error, false
positives, missed detections, merged circles, split circles, effect of partial
visibility.

**Success criterion (the simulation gates, restated for hardware — treat them
as targets, and report the numbers whether or not they are met):** centre
error ≤ 0.10 m, `true_radius` error ≤ 0.05 m on `/tracked_obstacles`
pre-inflation, detection latency ≤ 2 frames.

**Also record explicitly:** does the simulation finding "bias grows with
radius" (+33 mm radius, 83 mm centre at r = 0.30) reproduce? The
`fixed_inflation = 0.051` calibration rests entirely on it.

**Failure symptom.** No circles detected at 1 m → check `/scan` first (stage 9
territory).

**Stop condition.** None; on failure record 60 s of `/scan` + `/raw_obstacles`
for offline extractor work and end the stage.

**Do not modify the safety inflation to hide detector errors.** Detector
calibration and safety inflation are separate tasks with separate evidence.

**Files to save.** the bags, `t4_layout.yaml`, the harness output, per-target
error tables, photographs of the surveyed layout.

---

## Stage 11 — tracker validation

**Purpose.** Track stability, velocity accuracy, and whether the published
covariance means anything.

**Preconditions.** Stage 10 done.

**Scenarios**, ≥60 s each: static obstacle while the robot (or sensor) moves;
moving obstacle while the robot is still; two obstacles crossing; temporary
occlusion; reappearance after occlusion.

**Commands:**

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 11 60 \
    /scan /raw_obstacles /tracked_obstacles /obstacles_safe /odom /tf /tf_static
# during replay, offline:
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/phase4_obstacles_dump.py \
    120 "$SESSION/stage11.jsonl"
```

**Measure:** track-ID stability, velocity RMSE, association failures, ID
switches, track creation delay, track deletion delay, covariance output,
coasting behaviour, merged-track behaviour.

**Success criterion.** IDs survive a brief occlusion; velocity error is
characterised (there is no prior hardware number to meet).

**`measurement_variance: 1.0` must be treated as uncalibrated.** Derive the
real value from this session's data — preferably from the detector's own
scatter about each target's mean (which absorbs the systematic circle-fit
bias that `fixed_inflation` already covers and must not be double-counted):

```bash
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/measure_measurement_variance.py \
    "$SESSION/stage11.jsonl"
```

**Do not enable `use_covariance` until covariance consistency is assessed.**
`calibrate_k_sigma.py` refuses to emit a `k_sigma` while σ p50 > 0.25 m,
precisely so that a wrong `R` cannot be silently absorbed into it. Set `R`
first, then re-verify the containment numbers — `R` changes how much the
tracker trusts each measurement, so it changes the tracked positions the whole
safety chain consumes.

**Stop condition.** None; this is measurement.

**Files to save.** the bags, `stage11.jsonl`, the variance and (if it is
emitted at all) `k_sigma` outputs, the per-scenario table.

---

## Stage 12 — full perception-only pipeline

**Purpose.** The endpoint of this phase: the whole chain running on real data,
with nothing connected to the robot.

**Preconditions.** Stages 1–11 done, or their failures recorded.

**Commands:**

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true record:=on \
    bag_path:="$SESSION/stage12_full" 2>&1 | tee "$SESSION/stage12.log"

# in another shell, while it runs
ros2 topic hz /livox/lidar /points_self_filtered /scan \
              /raw_obstacles /tracked_obstacles /obstacles_safe
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
    -p duration:=120.0 -p json:="$SESSION/stage12_tf.json"
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/phase4_latency_probe.py 60 \
    | tee "$SESSION/stage12_latency.txt"
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/proc_cpu.py \
    | tee "$SESSION/stage12_cpu.txt"
ros2 topic echo /diagnostics | head -120 | tee "$SESSION/stage12_diagnostics.txt"
```

**Verify all six topics are live:** `/livox/lidar`, `/points_self_filtered`,
`/scan`, `/raw_obstacles`, `/tracked_obstacles`, `/obstacles_safe`.

**Prove no command is being published** (do this, do not assume it):

```bash
ros2 topic list | grep -E 'cmd|lowcmd|wireless|sport' \
    | tee "$SESSION/stage12_no_command_topics.txt"     # expect: nothing from us
ros2 node list  | tee "$SESSION/stage12_nodes.txt"     # expect: no g1_ctrl, no deploy
ros2 topic info /obstacles_safe --verbose \
    | tee "$SESSION/stage12_safe_subscribers.txt"      # expect: RViz relay only
```

**Measure:** end-to-end rate; cloud→scan, cloud→raw, cloud→tracked,
cloud→safe latency; CPU per process; memory; TF miss rate; message age; packet
loss; obstacle count; false positives; floor artefacts; self-hit artefacts.

**Success criterion (all of):**

- stable real sensor input;
- stable odometry;
- timestamp-consistent TF (`hw_tf_probe` success ≥ 0.95 at cloud stamps);
- **no persistent robot-body phantom obstacle**;
- **no persistent floor obstacle ring**;
- known fixtures detected in the correct `odom` coordinates;
- static world obstacles stay approximately stationary in `odom` while the
  sensor or robot moves;
- **no actuation** — the three checks above return nothing;
- reference budgets for context (dev-machine numbers, not gates on unknown
  hardware): perception container < 1 core, cloud→`/obstacles_safe`
  p95 ≤ 60 ms. If perception ran on a workstation rather than the onboard PC,
  **say so** and carry the on-target benchmark as a named follow-up.

**Failure symptoms.** A phantom obstacle that follows the robot (self-hit
leaking past CropBox); a ring of obstacles at fixed radius (floor entering the
height band); obstacles that drift in `odom` while the sensor moves (odometry
or extrinsic error, not a detector error).

**Stop condition.** Any evidence that a command topic exists → stop
immediately and find out what started it.

**Files to save.** the bag **plus its `.session.json` and `configs/`**, all
`tee` outputs above, RViz screenshots, the latency and CPU dumps.

---

## Stage 13 — controlled G1 posture changes, no walking

**Purpose.** See what the perception stack does when the robot actually moves
its body. **No translation is commanded.**

**Preconditions.** Stages 0–12 passed. E-stop operator in position. Robot
supported. Everyone informed.

**Procedure.** Enter a known safe standing state using the **standard Unitree
procedure** (hold the robot; `L2+up` → FixStand; `R2+A` → RLBase; *then* lower
it — do not lower onto FixStand's PD hold pose). Then, with perception
recording:

- body sway;
- small pitch;
- small yaw;
- arm motion;
- posture transitions.

**Commands:** as stage 12, plus a bag per motion type via `hw_record.sh`.

**Measure whether:** self-hits reappear; floor artefacts increase; TF remains
available at cloud stamps; DLIO remains stable; obstacle tracks jump;
`base_footprint` remains valid.

**Success criterion.** Perception and odometry remain stable through every
motion; any degradation is characterised rather than merely observed.

**Failure symptoms.** DLIO diverging under body motion; TF lookups failing as
the TF rate drops; tracks jumping when the body pitches (the fixed
`base_link→torso_link` approximation is a real hardware error source — the
waist joints move, the published transform does not).

**Stop condition.** **Stop immediately** if perception or odometry becomes
unstable. Return to Passive.

**Files to save.** one bag per motion type, the diagnostics stream, the
before/after comparison.

---

## Stage 14 — OPTIONAL externally commanded very-low-speed motion

**Blocked by default.** This stage runs only if the operator, the lab and the
robot-safety procedure **explicitly** allow it.

**Purpose.** See odometry, self-hits, floor artefacts, TF timing, obstacle
stability and CPU under gait — as observations, not as a gate.

**Preconditions.** Stage 13 passed. E-stop operator in position. Clear space.
The existing Unitree control interface commands a **very low, known** velocity.
**Perception remains read-only.**

**Hard constraints:**

- **`/obstacles_safe` is not connected to the controller.** Nothing subscribes
  it but the visualisation relay.
- **No DPCBF runs.** Do not claim DPCBF hardware operation on the basis of
  this stage — no filtered command exists.
- The perception launch is unchanged; the command comes from the robot's own
  interface, entirely separately.

**Measure:** odometry during gait; self-hit behaviour during arm and leg
motion; floor artefacts; TF timing; obstacle stability in `odom`; CPU and
latency under locomotion.

**Stop condition.** Any instability, any unexpected motion, any perception
degradation that would matter if it were in a control loop.

**Files to save.** the bag, the commanded velocity profile, the same metric
set as stage 12 with `robot_motion_occurred: true` in the metadata.

---

## Session artefacts

For every session produce `evidence/hardware/<date>/<session_name>/`
containing:

```
preflight.txt              stage0.md … stage14.md      command log
env_stage*.txt             (environment dump per capture)
<bag>/                     <bag>.session.json          configs/
baginfo_stage*.txt         md5_stage*.txt
stage*_source_probe.json   stage*_tf_probe.json
stage*_diagnostics.txt     stage*_cpu.txt   stage*_latency.txt
frames.pdf                 screenshots/*.png
t4_layout.yaml             measured fixture geometry
operator_notes.md          known_failures.md
```

Plus the machine-readable summary — written by
`hw_session_metadata.py`, completed offline from the probe JSONs:

```json
{
  "hardware": {
    "g1_variant": "",
    "target_arch": "",
    "lidar_serial": "",
    "lidar_firmware": ""
  },
  "network": {
    "ros_interface": "",
    "livox_interface": "",
    "ros_domain_id": 0
  },
  "results": {
    "lidar_rate_hz": null,
    "imu_rate_hz": null,
    "odom_rate_hz": null,
    "tf_lookup_success": null,
    "cloud_to_safe_p95_ms": null
  },
  "actuation_enabled": false
}
```

The record must also state, explicitly: **whether robot motion occurred** and
**whether any command publisher was active** (`session.robot_motion_occurred`,
`session.command_publisher_active` — both default false, and the recording
tooling reports them as part of the metadata).

At the end of the session:

- [ ] every bag `md5sum`'d, sized, listed with its recipe and purpose;
- [ ] session log written: date, operator, preflight answers, topology
      decision, per-stage outcome **with numbers**, anything aborted and why;
- [ ] props and survey measurements photographed or sketched;
- [ ] **nothing tuned on the robot** — every retune is offline, from a bag,
      with the measurement recorded.

---

## Shutdown

```bash
# 1. stop recording (Ctrl-C in the recording shell), let rosbag finish
# 2. stop the perception launch (Ctrl-C in shell A)
# 3. the driver may not die — it ignores SIGINT/SIGTERM on some paths
pkill -9 -f 'livo[x]'
pkill -9 -f 'component_containe[r]'     # the bracket is REQUIRED: an
                                        # unbracketed pattern matches the
                                        # killing shell's own command line
pkill -9 -f 'dlio_odom_nod[e]'
ros2 node list                          # expect empty
# 4. return the controller to Passive by the robot's own procedure
# 5. power down per stage 0
```

---

## Quick answers a first-time operator needs

| Question | Answer |
|---|---|
| Which computer runs perception? | The decision recorded in preflight §3.4. Not decided by this repository |
| Which network interface? | Preflight §3.2 table; pinned via `CYCLONEDDS_URI` |
| G1 and Mid-360 IP addresses? | Preflight §1/§2; the shipped values are **placeholders** |
| How is CycloneDDS configured? | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID` matching SDK2, `CYCLONEDDS_URI` pinning one NIC |
| How is the workspace built on the target? | `tools/build_target.sh` (`--check` first). aarch64 is **unproven** |
| How is the Mid-360 launched alone? | `source_hw.launch.py driver:=on lio:=off` (stage 3) |
| How are LiDAR and IMU validated? | `hw_source_probe.py` (stage 3) |
| How is the mount extrinsic checked? | `tf2_echo` + tape measure + floor-plane sign (stage 4) |
| How is DLIO initialised? | Robot still for ≥3 s, `lio:=dlio` (stage 5) |
| How is TF checked at LiDAR timestamps? | `hw_tf_probe.py` — stamped lookups, never latest |
| How is CropBox behaviour inspected? | Stage 8: `/livox/lidar` vs `/points_self_filtered`, with an external object |
| How are floor returns identified? | Stage 9: near-field `/scan` bin fraction; diagnostics `perception/floor_artifact` |
| How is the full stack launched without actuation? | `g1_perception_hardware_only.launch.py` |
| How is a bag recorded with metadata? | `record:=on`, or `hw_record.sh` |
| Which parameters are uncalibrated? | Preflight §6 |
| What aborts the experiment? | Each stage's stop condition; §11.5 of the phase report |
| How is everything shut down? | The shutdown block above |
| Why no DPCBF walking yet? | Preflight §5 and §9 below |

---

## §9 — What the later DPCBF hardware phase still needs

**Design notes only. None of this is implemented, and none of it is in scope
for this session.**

### 9.1 Hardware `RobotState` source

DPCBF's `Filter()` needs:

```
RobotState { x; y; phi; sagittal_velocity; lateral_velocity; }
```

Candidate sources: DLIO `/odom` pose; DLIO `/odom.twist`; differentiated
pose; Unitree's body-velocity estimator; the body IMU; a fused estimator.

The design must resolve, with measurements: which **frame** each is in; the
**timestamp** each carries and whether they share a clock; **latency**;
**velocity noise** (differentiating a 10 Hz-TF pose is not the same as reading
a twist); **yaw consistency** between DLIO and the robot's own estimate; and
**behaviour on loss of odometry** — which must be a defined degradation, not
an undefined one.

### 9.2 Hardware command seam

```
joystick / autonomy command
    → desired sagittal / lateral / yaw
    → DPCBF Filter()
    → filtered command
    → G1 controller
```

Before anything is connected, **prove no raw-command bypass exists** — that
every path from input to controller passes through the filter. Then specify:
the controller input API; command rate; which thread; fail-safe behaviour;
solver-failure behaviour; no-data behaviour; stale-data behaviour; command
timeout; FSM button handling; emergency stop; and a **dry-run mode**.

### 9.3 Hardware time domain

Determine whether these share a clock: LiDAR packets, IMU packets, DLIO
odometry, ROS system time, ROS steady time, the Unitree controller's time, and
the DPCBF query time.

**The staleness ladder cannot be reused safely until `frame.stamp` and
`t_query` are in a validated common time domain.** Stage 3's `clock_domain`
measurement is the first input to this.

### 9.4 Hardware shadow mode

There is no MuJoCo ground truth on hardware, so the sim's oracle/shadow ladder
does not transfer. Define a **hardware dry-run**: compute the DPCBF output,
log the raw and filtered commands, and **never send the filtered output to the
robot**. External motion capture may optionally provide evaluation ground
truth but must not be assumed available.
