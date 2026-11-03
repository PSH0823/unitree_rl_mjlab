# G1 Field Manual — 2026-08-13: Tuning perception for a FAST-WALKING human

**Target**: the 2026-08-13 session. Yesterday (08-12) the pipeline ran end to
end, but **a person walking fast is not turned into a stable circle**. Slow
walking works; fast walking loses the obstacle, or the circle flickers, jumps,
or lags behind the person.

**This document is a parameter-tuning manual.** It explains, for every knob in
the chain, *what it physically does*, *which symptom it fixes*, *what it breaks
if you push it too far*, and *how to verify the change on screen in 30 seconds*.

**The instrument you tune with is the plot that was written yesterday** —
`dpcbf_scan_view` (commits `d049fb7`, `f9da9c7`). It draws the `/scan` the
circles were fitted **from**, in the robot's own frame, on axes that never
move. That is the only view in which "the fit is bad" can be separated from
"there was nothing to fit."

> Everything about the **network link** (Fast DDS, `ROS_DOMAIN_ID`, multicast,
> peers mode, troubleshooting) stays in
> [`g1_first_day_field_runbook.md`](g1_first_day_field_runbook.md). This
> document repeats only the minimum needed to get the terminals up, and
> **does not touch `~/.bashrc` on either machine** — you source the
> environment by hand in every terminal (§1.3).

---

## Contents

- [0. Why a fast walker is lost — the failure chain](#0-why-a-fast-walker-is-lost--the-failure-chain)
- [1. Terminals and environment (no .bashrc)](#1-terminals-and-environment-no-bashrc)
- [2. Comp2 preparation](#2-comp2-preparation-g1-onboard-foxy)
- [3. Comp3 preparation](#3-comp3-preparation-laptop-humble)
- [4. Link check — the short version](#4-link-check--the-short-version)
- [5. The three windows you tune with](#5-the-three-windows-you-tune-with)
- [6. Run 0 — the baseline measurement](#6-run-0--the-baseline-measurement-do-this-first)
- [7. THE TUNING CHAPTER](#7-the-tuning-chapter)
- [8. Decision tree — which stage to touch](#8-decision-tree--which-stage-to-touch)
- [9. Copy-paste run sheet](#9-copy-paste-run-sheet)
- [10. Troubleshooting](#10-troubleshooting)
- [11. What to record](#11-what-to-record)

---

## 0. Why a fast walker is lost — the failure chain

The pipeline is a chain. A fast walker can be lost at **six** different places,
and the fix is different at each one. This is the whole reason §8's decision
tree exists.

```
/livox/lidar  ──CropBox──▶ /points_self_filtered ──p2l──▶ /scan
      10 Hz                                        (height band!)
                 ──extractor──▶ /raw_obstacles ──tracker──▶ /tracked_obstacles
                   (grouping)                     (association + KF)
                                       ──safety filter──▶ /obstacles_safe
                                          (gating + inflation)
```

| # | Where | What fast motion does | Knob (§7 stage) |
|---|---|---|---|
| ① | Livox frame accumulation | One `/livox/lidar` frame is **100 ms of accumulation**. At 1.5 m/s the person is smeared **0.15 m** along the direction of travel. The fitted circle grows and its centre trails behind the truth by roughly half the smear | `publish_freq` (**B**) |
| ② | Height band (`min/max_height`) | Yesterday's band sits at roughly **knee-to-hip height**. That is exactly where the **legs swing**: they separate up to ~0.7 m at mid-stride, alternately occlude each other, and move at up to **twice** the body speed. The extractor sees 1 or 2 clusters that appear and disappear every frame | `min_height`/`max_height` (**A**) |
| ③ | Point starvation | A thin band × a sparse non-repetitive Livox pattern often leaves **fewer than 5 returns** on a person at 3 m. `min_group_points: 5` then drops them **silently** | `min_group_points`, band thickness (**A/C**) |
| ④ | Grouping / splitting | A smeared, two-legged cluster exceeds `max_group_distance` internally → split into fragments, each below `min_group_points` → discarded | `max_group_distance`, `max_split_distance`, `max_merge_*` (**C**) |
| ⑤ | Association | A **newly born** track has zero velocity, so the gate is applied to the raw two-frame displacement: 0.15 m at 1.5 m/s, 0.25 m at 2.5 m/s, against a gate of **0.30 m** — plus a radius-mismatch penalty. Miss it, and the track is never promoted: nothing on `/tracked_obstacles`, therefore nothing on `/obstacles_safe` | `min_correspondence_cost` (**D**) |
| ⑥ | KF lag | `measurement_variance: 1.0` (a **1-metre** 1-σ measurement — the file itself flags this value as inherited and wrong) gives a steady-state gain of roughly **K ≈ 0.25**, i.e. ~0.4 s to absorb a step. A crossing lasts ~2 s, so **the entire crossing is filter transient** | `measurement_variance`, `process_rate_variance` (**D**) |
| ⑦ | Safety gating | `max_age: 0.30` drops the **whole message** if the extractor misses 3 frames; `v_max_obstacle: 1.5` clamps a fast walker's speed | `max_age`, `v_max_obstacle` (**E**) |

> **The single highest-value change today is ②** — move the projection band off
> the swinging legs and onto the torso. Do §7-A first, and re-measure before
> touching anything else.

---

## 1. Terminals and environment (no .bashrc)

### 1.1 Terminals for this session

| | Machine | What |
|---|---|---|
| **T1** | Comp3 → SSH to Comp2 | the perception stack (start / stop / restart every tuning iteration) |
| **T2** | Comp3 → SSH to Comp2 | rate checks + `hw_obstacle_watch.py` (numbers) |
| **T3** | Comp3 → SSH to Comp2 | bag recording |
| **T4** | Comp3 → SSH to Comp2 | **the tuning terminal**: edit YAML → `colcon build` → `config_diff` |
| **T5** | Comp3 (itself) | **`dpcbf_scan_view`** — the primary instrument |
| **T6** | Comp3 (itself) | **RViz2** — raw point cloud + 2-D laser scan |
| **T7** | Comp3 (itself) | link check / `dpcbf_plot_client` (optional, odom-frame view) |

T1–T4 are on Comp2, T5–T7 on Comp3. New terminal: `Ctrl`+`Alt`+`T`; new tab:
`Ctrl`+`Shift`+`T`; switch: `Alt`+`1`, `Alt`+`2`, …

**Rule: type `hostname` as the first command in every terminal.** Half of a bad
field day is a command run on the wrong machine.

### 1.2 Environment files (write once, per machine)

**Comp3 (laptop, user `dyros`)** — the block you already have:

```bash
cd ~
cat > ~/.g1_net_env <<'EOF'
export ROS_DOMAIN_ID=7                 # ★ must be IDENTICAL on Comp2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=/home/dyros/unitree_rl_mjlab/ros2      # ★ different from Comp2
#export G1_PEER_IP=192.168.123.164     # Comp2's IP. Only for peers mode
EOF
```

**Comp2 (G1 onboard, user `unitree`)** — same file, **different `G1_WS`**:

```bash
# confirm the real path FIRST, do not guess:
cd /home/unitree/dyros_ws/sanghyuk_ws/unitree_rl_mjlab/ros2 && pwd
# if that fails:
find /home -maxdepth 6 -name deps.repos -path "*/ros2/*" 2>/dev/null
```

```bash
cd ~
cat > ~/.g1_net_env <<'EOF'
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=/home/unitree/dyros_ws/sanghyuk_ws/unitree_rl_mjlab/ros2   # ★ the path you just confirmed
#export G1_PEER_IP=<Comp3's IP>
EOF
```

| Variable | Comp2 | Comp3 | |
|---|---|---|---|
| `ROS_DOMAIN_ID` | `7` | `7` | **must match** |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | `rmw_fastrtps_cpp` | **must match** |
| `ROS_LOCALHOST_ONLY` | `0` | `0` | both 0 |
| `G1_WS` | `/home/unitree/dyros_ws/sanghyuk_ws/…/ros2` | `/home/dyros/unitree_rl_mjlab/ros2` | **different** |
| `G1_PEER_IP` | Comp**3**'s IP | Comp**2**'s IP | **opposite**, peers mode only |

### 1.3 ★ The block you paste into **every** terminal

Nothing is added to `~/.bashrc` on either machine, so a new terminal has
**no** ROS environment until you paste this. Forgetting it is the #1 cause of
"the topic list is empty."

**On Comp2 (T1–T4), 4 lines:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

**On Comp3 (T5–T7), 4 lines:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Verify in one line (works on both):

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY | tr '\n' ' '; echo; ls "$G1_WS/deps.repos"
```

Expected: `7 rmw_fastrtps_cpp 0` and the `deps.repos` path. If `ls` says
*No such file or directory*, `G1_WS` is wrong — fix it before doing anything
else, or `cd "$G1_WS"` fails silently and `source install/setup.bash` runs in
the wrong directory ("I built it but the package is missing").

> **SSH into Comp2** (from a Comp3 terminal):
> ```bash
> ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4 unitree@<Comp2 IP>
> ```
> Turn off the laptop's screen blanking and automatic suspend, and do not
> close the lid: a sleeping laptop kills all four SSH sessions at once.

---

## 2. Comp2 preparation (G1 onboard, Foxy)

**T1**, after the 4-line block:

```bash
cd "$G1_WS"
git pull
source /opt/ros/foxy/setup.bash
colcon build --packages-select g1_perception_bringup
```

That rebuild takes a few seconds and is what copies `config/*.yaml` into
`install/share/`. **`ros2 launch` reads the installed copy, never the file you
edited** — this is the whole point of §7.1.

If the workspace has never been built on this machine, use the 15-package
build from the Fast DDS manual §2-4.

Preflight (must PASS before the first run of the day):

```bash
./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh
echo "EXIT=$?"
```

---

## 3. Comp3 preparation (laptop, Humble)

**T5**, after the 4-line block:

```bash
cd "$G1_WS"
git pull
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select \
    obstacle_detector dpcbf_viz_msgs dpcbf_plot_client
```

`git pull` matters today: `dpcbf_scan_view` and its `roll = pi` fix
(`f9da9c7`) landed yesterday evening. Without the fix **every circle is
mirrored about the robot's forward axis** while still looking plausible.

Verify:

```bash
source install/setup.bash
ros2 pkg executables dpcbf_plot_client
```

Expected — three entries, the third is today's instrument:

```
dpcbf_plot_client dpcbf_plot_client
dpcbf_plot_client synthetic_dpcbf_publisher
dpcbf_plot_client dpcbf_scan_view
```

apt packages, if this laptop is fresh:

```bash
sudo apt-get install -y python3-matplotlib python3-pyqtgraph python3-pyqt5 \
                        python3-pyqt5.qtopengl ros-humble-laser-geometry
```

---

## 4. Link check — the short version

Full procedure: Fast DDS manual §4. Minimum before you start tuning, with the
stack already running on Comp2 — in **T7 on Comp3**:

```bash
ros2 daemon stop
ros2 topic hz /scan --no-daemon                # ~10
ros2 topic hz /obstacles_safe --no-daemon      # ~10
```

If those two arrive on the laptop, everything in §5 will work. If they do not,
**stop and fix the link** (Fast DDS manual §6-1) — a tuning session run
through a broken link produces conclusions about the network, not about the
parameters.

---

## 5. The three windows you tune with

### 5.1 `dpcbf_scan_view` — the primary instrument (T5, Comp3)

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dpcbf_plot_client dpcbf_scan_view
```

With options (this is the form to use today):

```bash
ros2 run dpcbf_plot_client dpcbf_scan_view --ros-args \
    -p range:=4.0 -p scan_history:=5
```

| Parameter | Default | Meaning / when to change |
|---|---|---|
| `range` | `5.0` | half-width of the fixed axes [m]. Set it to the crossing distance + 1 m so the person fills the window |
| `scan_history` | `1` | overdraws the last N scans, older ones fainter. **`5` is the setting for today**: it makes the person's motion visible as a trail of dots, and separates *scan* noise from *fit* noise at a glance |
| `target_frame` | `base_link` | frame everything is drawn in. `odom` compares against the world view; `''` means "whatever frame `/scan` arrives in" (which today is `mid360_link`, and **that frame is upside down**, so the picture comes out mirrored — do not use it) |
| `obstacle_topics` | raw, tracked, safe | the three pipeline stages, drawn in different styles |
| `scan_topic` | `/scan` | — |
| `stale_after_s` | `1.0` | banner turns red past this age |

**How to read it** — this is the entire diagnostic:

| On screen | Source | Meaning |
|---|---|---|
| bright blue dots | `/scan`, newest frame | **what the extractor actually got.** If the person is not here, no parameter downstream can help |
| faint blue dots | previous N scans | motion trail; jitter here is *sensor/projection* noise |
| **grey dotted circle** | `/raw_obstacles` | the extractor's per-frame fit |
| **orange circle + uid + r** | `/tracked_obstacles` | the tracker's KF output; the label is `uid r=<true_radius>` |
| **red circle** | `/obstacles_safe` | after gating + inflation — what DPCBF would consume |
| dashed line from a circle | velocity | where the centre reaches in **1 second** |
| green dot + line at origin | robot | fixed at (0,0), forward = +x, left = +y |
| banner top-left | ages | green = fresh, red = `STALE`/`NO DATA` |

**The four colours are the failure chain made visible.** Dots but no grey
circle → extractor (§7-C). Grey but no orange → tracker (§7-D). Orange but no
red → safety filter (§7-E). No dots at all → projection (§7-A).

> The velocity dash is also your speed read-out: its length **in metres** is the
> estimated speed in m/s. A person walking at 1.5 m/s should draw a dash about
> 1.5 m long, in the direction they are walking. A dash that jitters in
> direction every frame means the tracker is re-creating the track, not
> tracking it.

### 5.2 `hw_obstacle_watch.py` — the numbers (T2, Comp2)

The console table version of the same thing, printed in **base_link**, with
both `radius` and `true_radius`:

```bash
ros2 run g1_perception_bringup hw_obstacle_watch.py
ros2 run g1_perception_bringup hw_obstacle_watch.py --ros-args \
    -p rate:=2.0 -p json:=$SESSION/tuning_watch.jsonl
```

Use it when you need to *write a number down* — the on-screen circle tells you
whether it works, this tells you by how much. The `json:` parameter appends
one JSON line per frame, which is the evidence for "run 3 was better than run 2."

### 5.3 RViz2 — raw point cloud and 2-D laser scan (T6, Comp3)

**This is the view that answers "is the person even in the height band?"**

The repo already ships a robot-frame layout with exactly the two displays you
want. It is a plain file in the source tree, so **Comp3 does not need
`g1_perception_bringup` built** to use it:

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d "$G1_WS/src/g1_perception/g1_perception_bringup/rviz/perception_robot_frame.rviz"
```

| Display in that layout | Topic | Note |
|---|---|---|
| `LivoxCloud` | `/livox/lidar` | **raw 3-D cloud**. Starts *disabled* in the committed layout — tick its checkbox |
| `Scan (extractor input)` | `/scan` | the 2-D scan, i.e. the height band after projection |
| `Odom` | `/odom` | |
| `RawObstacles` / `TrackedObstacles` / `SafeObstacles` | marker relays | **will stay empty** — those relays only run under `viz.launch.py`, which is not part of the hardware stack. Use `dpcbf_scan_view` for circles |

Fixed Frame is `base_link`.

**Building it by hand instead** (if you prefer, or the config will not load):

```bash
rviz2
```
1. **Global Options → Fixed Frame**: `base_link` (or `mid360_link` — see below)
2. **Add → By topic → `/livox/lidar` → PointCloud2**
   - **Topic → Reliability Policy: `Best Effort`** ← do this. The Livox driver
     publishes *Reliable, depth 256*; a Reliable subscriber over Wi-Fi makes
     the middleware retransmit a 0.5 MB message and can starve `/odom`
   - Size (m): `0.02`, Style: `Points`
   - **Color Transformer: `AxisColor`, Axis: `Z`** — this is what lets you
     read heights straight off the picture
3. **Add → By topic → `/scan` → LaserScan**
   - Reliability Policy: `Best Effort`, Style `Points`, Size `0.05`, colour red
4. **Add → `/points_self_filtered` → PointCloud2** if you want to see what the
   CropBox self-filter removed

> ⚠ **Bandwidth.** `/livox/lidar` is roughly **5 MB/s** (≈20 000 points ×
> 26 bytes × 10 Hz). Over Wi-Fi that competes with everything else on the link.
> Use it in short bursts to answer a specific question, then untick it.
> `/scan` is ~4 kB per frame and costs nothing — leave that one on.

> **To measure the height band directly**: set Fixed Frame to `mid360_link`,
> enable `AxisColor` on Z, and use **Measure** (toolbar) to click from the
> floor to a point on the person. Remember the sign: in `mid360_link`
> **positive z is *below* the sensor** (the Mid-360 is mounted upside-down,
> `roll = π`). The floor shows up at `z ≈ +H`, where `H` is the sensor's height
> above the floor — that is the fastest way to get `H` for §7-A.

**On Comp2 instead** (if the laptop cannot show it): `ssh -X unitree@<Comp2 IP>`,
paste the 4-line block, then add `use_rviz:=true` to the launch in §9-1.

---

## 6. Run 0 — the baseline measurement (do this first)

**Do not change a parameter before you have Run 0.** Without it you cannot tell
which of today's edits helped.

Setup: open floor, robot standing (or on its stand), one person crossing left
to right at a fixed distance of **2.5–3 m**, then back.

| Pass | Speed | Roughly |
|---|---|---|
| 1 | slow | 0.5 m/s — a stroll |
| 2 | normal | 1.2 m/s |
| 3 | **fast** | 1.8–2.0 m/s — a brisk walk, the failing case |
| 4 | **stop-and-go** | walk fast, stop dead for 2 s, walk again |
| 5 | **toward the robot** | walk straight at the robot from 4 m to 1.5 m |

For each pass, look at T5 and record **one letter**:

| Letter | What you saw |
|---|---|
| **N** | no blue dots on the person — nothing to fit |
| **S** | dots, but fewer than ~5, or split into two clumps (legs) |
| **R** | grey circle appears, but flickers on and off |
| **T** | grey stable, but **no orange** (track never promoted) |
| **L** | orange present but **lags behind the dots** along the direction of travel |
| **X** | orange fine, **no red** (`/obstacles_safe` empty) |
| **OK** | all three circles stay on the person for the whole crossing |

That letter takes you straight to a stage in §8. Take a screenshot of each
pass (`PrtSc`) into `$SESSION` — five screenshots is your before/after.

**Also record a bag for pass 3 and 5** (§9-3). A bag that contains
`/livox/lidar` lets you re-tune the **entire** downstream chain offline,
tonight, without the person: replay it with `driver:=off lio:=off` and every
parameter in §7-A/C/D/E can be swept from a chair. This is worth the disk
space (~5 MB/s → ~150 MB for a 30 s pass).

---

## 7. THE TUNING CHAPTER

### 7.1 ★ How the edit loop actually works (read this or lose the day)

**Three facts that will otherwise cost you hours:**

1. **`ros2 param set` does not work on the extractor or the tracker.** In this
   fork the parameter-update service is commented out
   ([obstacle_extractor.cpp:50](../src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L50),
   [obstacle_tracker.cpp:53](../src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L53));
   the values are read **once**, at node construction. `ros2 param set
   /obstacle_extractor min_group_points 3` will return `Set parameter
   successful` and **change nothing**. Do not trust it.
2. **`ros2 launch` reads the *installed* YAML**, not the one you edit. Editing
   `src/.../config/*.yaml` and relaunching runs the **old numbers**, with
   nothing printed anywhere to say so.
3. All five perception nodes are components in **one container**, so any change
   means restarting the whole stack — including DLIO. **Keep the robot still
   for 3 s after every relaunch** (IMU calibration, `odom/imu/calibration/time:
   3.0`).

**The loop, in T4 on Comp2:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash

nano src/g1_perception/g1_perception_bringup/config/<file>.yaml   # edit

colcon build --packages-select g1_perception_bringup              # ~5 s: copies config into install/
ros2 run g1_perception_bringup config_diff.py                     # every line must say IDENTICAL
```

Then in **T1**: `Ctrl-C`, relaunch (§9-1), stand still 3 s, and re-run the pass
you are testing. **One stage per iteration.** Two changes at once and you learn
nothing from the result.

> Optional speed-up for the day: rebuild the workspace once with
> `colcon build --symlink-install --packages-select g1_perception_bringup`.
> The installed config then becomes a symlink to the source file, and the
> `colcon build` step disappears from the loop — you still restart the stack.
> `config_diff.py` keeps passing (identical bytes).

### 7.2 Proposed starting set for today

Apply **Stage A first, alone**, and re-measure. Apply the rest only if the
letter from §6 says so.

| Stage | File | Parameter | 08-12 | Proposed | Why |
|---|---|---|---|---|---|
| **A** | `pointcloud_to_laserscan.yaml` | `min_height` | `0.2` | `-0.15` | raise the band's top above the sensor plane |
| **A** | " | `max_height` | `0.7` | `0.65` | keep the floor out; band now covers waist→chest instead of knees |
| **C** | `obstacle_detector.yaml` | `min_group_points` | `5` | `3` | a person at 3 m in a thin band often gives 3–4 returns |
| **C** | " | `max_group_distance` | `0.10` | `0.15` | tolerate the 0.15 m motion smear inside one cluster |
| **C** | " | `distance_proportion` | `0.01745` | `0.03` | range-scaled tolerance, for the far end of the crossing |
| **C** | " | `max_split_distance` | `0.20` | `0.25` | stop splitting one smeared person into two |
| **C** | " | `max_merge_separation` | `0.20` | `0.35` | merge two legs into one obstacle |
| **C** | " | `max_merge_spread` | `0.20` | `0.30` | same |
| **D** | " | `min_correspondence_cost` | `0.3` | `0.5` | 2.0 m/s × 0.1 s = 0.20 m + radius penalty must fit inside the gate |
| **D** | " | `measurement_variance` | `1.0` | `0.04` | 1 σ = 0.2 m instead of 1 m — the KF stops lagging |
| **D** | " | `process_rate_variance` | `0.03` | `0.10` | let the velocity state change as fast as a person can |
| **E** | `safety_obstacle_filter.yaml` | `max_age` | `0.30` | `0.50` | survive a 3-frame detection gap |
| **E** | " | `v_max_obstacle` | `1.5` | `2.5` | stop clamping a fast walker's speed |
| **B** | `livox_driver.yaml` | `publish_freq` | `10.0` | `20.0` | **last resort**, see §7-B for the three companion edits |

---

### Stage A — the projection band (`pointcloud_to_laserscan.yaml`)

**This is the one that matters most.**

#### What these parameters actually do

`pointcloud_to_laserscan` takes the 3-D cloud, **keeps only the points whose z
lies between `min_height` and `max_height`**, and flattens that horizontal slab
into a 2-D `LaserScan`. Everything downstream sees only that slab. A person
outside the slab does not exist to this pipeline.

**The sign trap.** With `target_frame: ''` (yesterday's setting) no transform
is applied, so `min_height`/`max_height` are measured **in `mid360_link`** —
and that frame carries `roll = π`, because the Mid-360 is mounted upside-down
([`g1_mid360.xacro`](../src/g1_perception/g1_description/urdf/g1_mid360.xacro)).
Therefore:

> **positive `z` in `mid360_link` = *below* the sensor.**

So the mapping to real heights above the floor, with `H` = the sensor's height
above the floor, is:

```
height above floor  =  H − z_mid360

band [min_height, max_height]  →  floor band [ H − max_height ,  H − min_height ]
```

**Measure `H` before you edit anything.** Tape measure from the floor to the
centre of the Mid-360, in the pose the robot will hold, or read it off RViz
(§5.3: the floor appears at `z_mid360 ≈ +H`). Everything below assumes
**H ≈ 1.20 m** — substitute your measured value.

| Setting | Floor band with H = 1.20 | What it hits on a person |
|---|---|---|
| 08-12: `0.20 … 0.70` | **0.50 – 1.00 m** | knees, thighs, hips — **the swinging legs** |
| Proposed: `-0.15 … 0.65` | **0.55 – 1.35 m** | thighs → waist → lower chest — one solid trunk |
| torso-only: `-0.15 … 0.35` | 0.85 – 1.35 m | trunk only; fewest points, cleanest circle |

**Why moving up fixes fast walking.** The legs are the fastest and least
circle-like part of a walker: at mid-stride they are two separate 0.15 m-wide
objects up to 0.7 m apart, moving at up to twice the body's speed, and each
alternately occludes the other. The torso is one 0.3–0.4 m object that moves at
exactly the walking speed and never splits. **The circle model in this pipeline
describes a torso; it does not describe a pair of legs.**

#### The two limits you must not cross

| Limit | Rule | If you break it |
|---|---|---|
| **Sensor FOV above the horizon** | Mounted upside-down, the Mid-360 covers ~**7° above** its own horizontal plane and ~52° below. A point `d` metres away can be at most `0.12 · d` above the sensor | `min_height` more negative than `−0.12 · d_min` gives you an empty band at close range. At 1.5 m: `min_height ≥ −0.18` |
| **The floor** | The floor is at `z_mid360 ≈ +H` | `max_height` close to `H` fills the scan with a **ring of floor returns**, the extractor fits circles to the floor, and everything else drowns. Keep `max_height ≤ H − 0.4` |

#### Band thickness is itself a knob

Thicker band = more points per person = more robust to the sparse Livox
pattern, but also more of the room (chairs, tables, a wall at torso height) in
the scan. For an open-floor crossing, **thicker is better**. If the room is
cluttered at torso height, narrow it and compensate in Stage C
(`min_group_points`).

> **Also worth knowing**: the band is defined relative to the **sensor**, which
> rides on the torso. When the robot walks, torso pitch of ±5° moves the far
> end of the band by ±0.35 m at 4 m. This is another argument for a thick band
> once the robot itself starts moving.

#### Alternative: put the band in a world frame

```yaml
target_frame: base_footprint   # then min/max_height are heights above the FLOOR, sign normal
min_height: 0.70
max_height: 1.50
```

This is the pre-08-12 setting and it is the *right* long-term answer — heights
mean what they say, and torso pitch no longer moves the band. It costs a TF
lookup per cloud (`odom → base_footprint`, i.e. it puts **DLIO in the
projection path**), which is exactly why it was switched off yesterday for the
hanging test. **If DLIO is healthy today, try this — it removes the whole sign
trap.** Verify with `ros2 topic hz /scan`: if `/scan` drops out while
`/points_self_filtered` keeps running, TF is the problem, and you go back to
`target_frame: ''`.

#### Verify Stage A in 30 seconds

1. T6 (RViz): the person is visible in `/livox/lidar` **and** now appears in
   `/scan` as a small arc.
2. T5 (`dpcbf_scan_view`): the person is a compact clump of **5+ blue dots**
   that stays one clump through the whole crossing (no splitting into two).
3. `ros2 topic hz /scan` still ~10 Hz.

---

### Stage B — sensor rate (`livox_driver.yaml`) — *last resort*

#### What it does

`publish_freq: 10.0` means the driver accumulates **100 ms** of the Mid-360's
non-repetitive pattern into each `/livox/lidar` message. That accumulation is
the source of the motion smear in failure ①: a person at 1.5 m/s is smeared
0.15 m within one frame, which inflates the fitted radius and drags the centre.

At 20 Hz the smear halves — but so does the **number of points per frame**,
which makes failure ③ worse. This is a genuine trade, which is why it is last.

#### If you try it, all four edits are required

```yaml
# livox_driver.yaml
publish_freq: 20.0

# pointcloud_to_laserscan.yaml
scan_time: 0.05          # was 0.1 — this is metadata; consumers use it

# obstacle_detector.yaml (both nodes)
sensor_rate: 20.0        # association cost model + fade-counter sizing
loop_rate: 20.0
```

`tracking_duration` stays `1.0` (it is in seconds; the fade window is computed
as `rate × duration`, so it re-sizes itself to 20 ticks).

#### Verify

```bash
ros2 topic hz /livox/lidar    # 20
ros2 topic hz /scan           # 20
ros2 topic hz /obstacles_safe # 20
top -b -n1 | head -15         # Comp2 CPU — the container roughly doubles its load
```

Then re-check §6 pass 3. **If `min_group_points` starts dropping the person
again, lower it to 3 (or 2) — that is the expected side effect.** Bag size and
Wi-Fi load also double.

---

### Stage C — extractor grouping (`obstacle_detector.yaml` → `obstacle_extractor`)

The extractor walks the scan point by point and does three things in order:
**group → split → merge → fit a circle**.

| Parameter | 08-12 | What it *does* | Symptom it fixes | Push too far and |
|---|---|---|---|---|
| `min_group_points` | `5` | a group with fewer points is **discarded** before any fit | **the person vanishes at range** (the classic silent drop) | 2 or 3 stray returns become a phantom obstacle; noise starts producing circles |
| `max_group_distance` | `0.10` | two consecutive scan points join the same group if their gap `< max_group_distance + range · distance_proportion`. This is the **cluster-continuity threshold** | the smeared/two-legged person is torn into fragments that then fall under `min_group_points` | the person **merges with the wall/table behind them**, and the fit blows past `max_circle_radius` (0.60) → dropped entirely |
| `distance_proportion` | `0.01745` (=1°) | the **range-scaled** part of the same threshold: at 4 m it adds `4 × 0.01745 = 0.07 m` | fragmenting that happens **only at the far end** of the crossing | same as above, but it kicks in only far away — which is the safer place to be generous |
| `max_split_distance` | `0.20` | after grouping, a group is split at its point of maximum deviation from the chord if that deviation exceeds this | one person being split into two circles | a genuine two-object scene (person + pillar) stays fused |
| `max_merge_separation` | `0.20` | two segments closer than this are candidates to merge | **the two legs staying two obstacles** | a person walking past a pillar merges with it |
| `max_merge_spread` | `0.20` | ...and they merge only if all four endpoints lie within this of the merged line | same | same |
| `max_circle_radius` | `0.60` | **hard drop**: a fit whose radius exceeds this is thrown away and counted | — | do **not** raise it casually; it is the sensing limit §9.6 relies on, and it is deliberately mirrored in `safety_obstacle_filter.yaml`. **If you change it, change both files.** |
| `radius_enlargement` | `0.17` | margin added to the fitted radius (`radius = true_radius + this`), compensating the short-arc bias of a partly visible object | circles that look too small on screen | over-inflated obstacles; DPCBF becomes needlessly conservative |
| `circles_from_visibles` | `true` | fit the circle from the visible arc only | — | — |
| `frame_id` / `transform_coordinates` | `odom` / `true` | the extractor publishes in **odom**, using a full TF from the scan frame | — | **do not change.** `dpcbf_scan_view` and `hw_obstacle_watch.py` both assume it |

**Grouping arithmetic, so you can pick numbers instead of guessing.** At range
`r`, two neighbouring returns belong to the same object if their gap is under

```
gap_max = max_group_distance + r · distance_proportion
```

| r | 08-12 (`0.10`, `0.01745`) | proposed (`0.15`, `0.03`) |
|---|---|---|
| 1 m | 0.12 m | 0.18 m |
| 3 m | 0.15 m | 0.24 m |
| 5 m | 0.19 m | 0.30 m |

A person's torso is 0.30–0.40 m wide, so the proposed row keeps a torso in one
piece out to 5 m even when the Livox pattern leaves holes in it.

#### Verify Stage C

T5: a **grey dotted circle** now appears on the person every frame (not every
other frame), with `r =` between about **0.15 and 0.30**. If `r` jumps above
0.5, you have merged the person with something else — back `max_merge_separation`
off. If the grey circle disappears while the dots are still there, the drop is
`min_group_points` or `max_circle_radius`.

---

### Stage D — the tracker (`obstacle_detector.yaml` → `obstacle_tracker`)

The tracker does two separable jobs, and they fail differently.

#### D-1. Association — "is this the same person as last frame?"

Every frame it builds a cost between each new detection and each existing
track:

```
cost = sqrt( Δx² + Δy² + (radius_residual_weight · Δr)² )
```

and accepts the match only if `cost < min_correspondence_cost`. Existing
tracks are **predicted forward first**, so for a mature track `Δ` is the
prediction error, not the full displacement. **A newly born track has zero
velocity**, so for the first two frames `Δ` *is* the full displacement.

| Speed | Displacement per 0.1 s frame | Against the `0.30` gate |
|---|---|---|
| 0.5 m/s | 0.05 m | comfortable |
| 1.2 m/s | 0.12 m | fine |
| 1.8 m/s | 0.18 m | + a 0.1 m radius wobble → cost 0.21 — tight |
| 2.5 m/s | 0.25 m | + any radius wobble → **over the gate, track never born** |

| Parameter | 08-12 | What it does | Raise it when | Cost of raising |
|---|---|---|---|---|
| `min_correspondence_cost` | `0.3` | the association gate, in metres | fast targets never get promoted to `/tracked_obstacles` (symptom **T**) | two nearby people swap identities, or a person "jumps" onto a static prop |
| `radius_residual_weight` | `0.3` | how much a radius mismatch is punished, relative to distance | a person whose fitted radius wobbles frame to frame is failing to associate | lower it and two different-sized objects associate too easily |
| `std_correspondence_dev` | `0.15` | measurement spread assumed inside the association distribution | — | rarely the right knob; change the gate instead |
| `sensor_rate` | `10.0` | **must match the real `/scan` rate.** Used in the association model and to size the fade window | you changed Stage B | a wrong value silently distorts association and how long tracks coast |
| `tracking_duration` | `1.0` | seconds a track survives with no matching detection before it is deleted | the person is intermittently occluded | a ghost circle coasts on after the person leaves |

#### D-2. The Kalman filter — "where exactly, and how fast?"

State per axis: `[position, velocity]`, constant-velocity model.

| Parameter | 08-12 | Role |
|---|---|---|
| `measurement_variance` | `1.0` | **R** — assumed variance of the measured centre, in **m²**. `1.0` asserts a **1-metre** 1-σ LiDAR measurement |
| `process_variance` | `0.0001` | **Q(0,0)** — how much the position is allowed to move unmodelled |
| `process_rate_variance` | `0.03` | **Q(1,1)** — how much the **velocity** is allowed to change per step, i.e. how much acceleration you permit |

**Why `measurement_variance: 1.0` is the lag you see on screen.** The gain is
`K = P/(P+R)`. The YAML's own comment records that with `R = 1.0` a track's
σ relaxes to about 0.58 m, i.e. `P ≈ 0.34`, giving

```
K ≈ 0.34 / (0.34 + 1.0) ≈ 0.25
```

Each update moves the estimate only a quarter of the way to the measurement —
about **0.4 s to absorb a step**. A 3 m crossing at 1.8 m/s lasts under 2 s, so
**the whole event is filter transient**: the orange circle chases the blue dots
and never catches up, and the velocity dash points the right way but is short.

The file already flags this value as inherited and wrong (measured scatter in
noiseless sim was 1.8e-06 m², which is not a shipping value either). For a real
Mid-360 + projection + circle fit on a human torso, a centre scatter of
**0.1–0.2 m** is the honest range:

```yaml
measurement_variance: 0.04    # sigma = 0.2 m — conservative first try
# measurement_variance: 0.01  # sigma = 0.1 m — if 0.04 still lags
```

`process_rate_variance: 0.03` corresponds to a velocity change of ~0.17 m/s per
step. A person starting, stopping or turning does more than that; `0.10`
(~0.32 m/s per step) tracks the stop-and-go pass much better.

> **Tuning R and Q against each other:** lower `measurement_variance` **or**
> higher `process_rate_variance` both increase the gain. Change **one**. Too
> much and the circle jitters frame to frame and the velocity dash flails;
> that jitter is the signal you have gone too far.

#### Verify Stage D

T5, during the fast pass:
- an **orange** circle exists at all (that is association, D-1);
- its centre sits **on** the bright blue dots, not behind them along the
  direction of travel (that is the KF, D-2);
- the `uid` label **stays the same number** for the whole crossing. A uid that
  increments every frame or two means the track is being re-born — go back to
  `min_correspondence_cost`;
- the dashed velocity line is about as long, in metres, as the person's speed
  in m/s, and points where they are going.

---

### Stage E — safety gating (`safety_obstacle_filter.yaml`)

This node turns `/tracked_obstacles` into `/obstacles_safe` — what a controller
would actually consume. It **drops**, **clamps** and **inflates**.

| Parameter | 08-12 | What it does | For a fast walker |
|---|---|---|---|
| `max_age` | `0.30` s | if the message stamp is older than this, **every circle in it is dropped** and the output is empty | 3 missed frames at 10 Hz and `/obstacles_safe` goes blank mid-crossing. **`0.50` while tuning** |
| `min_radius` | `0.20` m | **floor** on the reported radius (not a drop) — a small fit is reported as 0.20 | fine as is; a torso fit is 0.15–0.25 |
| `max_circle_radius` | `0.60` m | **hard drop** above this radius | keep **equal to the extractor's value** — the sensing limit is one number and splitting it across two files is a known past bug |
| `fixed_inflation` | `0.051` m | constant safety margin added to every radius (Phase-4 calibrated) | leave alone unless you are re-deriving it |
| `latency_horizon` | `0.12` s | radius is additionally inflated by `speed × this` — pre-paying for pipeline latency | at 2 m/s that is 0.24 m of extra radius, which is correct behaviour, not a bug |
| `v_max_obstacle` | `1.5` m/s | **clamps** the speed (direction preserved) before extrapolating | a 1.8–2.0 m/s walker is clamped → the red circle under-inflates and the velocity read-out is wrong. **`2.5` for this experiment** |
| `use_covariance` / `k_sigma` | `false` / `2.748` | the σ-inflation path. **Leave `false`** — `k_sigma` is an uncalibrated placeholder and enabling it is a joint recalibration with `fixed_inflation` | do not touch today |

The final radius is:

```
safe.radius = max(true_radius, min_radius) + fixed_inflation + |v_clamped| · latency_horizon
```

which is why the red circle is legitimately **larger** than the orange one, and
grows with speed.

#### Verify Stage E

T5: a **red** circle exists wherever there is an orange one, is slightly
bigger, and grows visibly as the person speeds up. If red disappears while
orange stays, it is `max_age` (the whole message is being gated) or
`max_circle_radius` (this one circle got too big).

---

## 8. Decision tree — which stage to touch

Start from the letter you wrote down in §6.

```
Look at dpcbf_scan_view during the FAST pass.

  No blue dots on the person? ────────────────▶ N → Stage A (height band)
       │                                            check RViz: visible in /livox/lidar
       │                                            but missing from /scan = band is wrong
       ▼
  Dots present but < 5, or two clumps? ───────▶ S → Stage A (thicker band, move up)
       │                                            then C (min_group_points 3)
       ▼
  Dots fine, grey circle flickers? ───────────▶ R → Stage C (grouping / split / merge)
       │                                            watch r: >0.5 = merged with background
       ▼
  Grey stable, no orange? ────────────────────▶ T → Stage D-1 (min_correspondence_cost)
       │                                            check: does uid keep changing?
       ▼
  Orange lags behind the dots? ───────────────▶ L → Stage D-2 (measurement_variance)
       │                                            then process_rate_variance
       ▼
  Orange fine, no red? ───────────────────────▶ X → Stage E (max_age, max_circle_radius)
       │
       ▼
  All three follow the person ────────────────▶ OK. Push the speed up and repeat.
                                                  Then try Stage B (20 Hz) for margin.
```

**Rules for the day:**

1. **One stage per iteration.** A restart is 30 s; a confounded result costs an
   hour.
2. **Re-run the *slow* pass after every change.** Several of these knobs (the
   merge distances, the association gate, a low `min_group_points`) buy fast
   performance by paying in false positives, and the slow pass is where you see
   that bill.
3. **Screenshot before and after**, into `$SESSION`. `PrtSc` is fine.
4. **Write the change into the YAML in `src/` and commit at the end of the
   day** — not just into `install/`.

---

## 9. Copy-paste run sheet

### 9-1. Comp2 / T1 — the stack

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash
export SESSION=$G1_WS/evidence/hardware/$(date +%F)/tuning && mkdir -p "$SESSION"

./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh || echo "STOP"

# ★ keep the robot still for 3 s after this line (DLIO IMU calibration)
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

Add `use_rviz:=true` if you want RViz on Comp2 instead of Comp3.

Replaying a bag instead of the live sensor (offline tuning):

```bash
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=off lio:=off
# in another terminal:
ros2 bag play <bag>
```

### 9-2. Comp2 / T2 — checks

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash
ros2 daemon stop

# Foxy takes one topic at a time — Ctrl-C between lines.
ros2 topic hz /livox/lidar          # 10 (20 after Stage B)
ros2 topic hz /scan                 # 10   ← Stage A lives or dies here
ros2 topic hz /raw_obstacles        # 10
ros2 topic hz /tracked_obstacles    # 10
ros2 topic hz /obstacles_safe       # 10

ros2 run g1_perception_bringup hw_obstacle_watch.py
```

### 9-3. Comp2 / T3 — bag (record pass 3 and pass 5 at minimum)

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash
export SESSION=$G1_WS/evidence/hardware/$(date +%F)/tuning

# Foxy syntax. Includes /livox/lidar so the whole chain can be re-tuned offline.
ros2 bag record -o "$SESSION/fast_$(date +%H%M%S)" \
    /livox/lidar /livox/imu /odom /tf /tf_static \
    /points_self_filtered /scan /raw_obstacles \
    /tracked_obstacles /obstacles_safe /diagnostics
```

### 9-4. Comp2 / T4 — the tuning loop

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash

nano src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml
colcon build --packages-select g1_perception_bringup
ros2 run g1_perception_bringup config_diff.py     # all IDENTICAL
# → then Ctrl-C and relaunch T1
```

### 9-5. Comp3 / T5 — `dpcbf_scan_view`

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

ros2 run dpcbf_plot_client dpcbf_scan_view --ros-args \
    -p range:=4.0 -p scan_history:=5
```

### 9-6. Comp3 / T6 — RViz2

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

rviz2 -d "$G1_WS/src/g1_perception/g1_perception_bringup/rviz/perception_robot_frame.rviz"
```

### 9-7. Comp3 / T7 — odom-frame view (optional)

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
```

`dpcbf/plot: NO DATA (no publisher)` in the banner is **correct** — there is no
DPCBF control seam on the robot. Left panel is the live view; the five
right-hand time series stay empty.

### 9-8. Shutdown order

```
① Comp3 T5/T6/T7 — close the GUIs   (no effect on Comp2)
② Comp2 T3       — bag Ctrl-C        (wait for rosbag to finalise)
③ Comp2 T2       — watch Ctrl-C
④ Comp2 T1       — stack Ctrl-C
```

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| **A parameter change did nothing** | ⓐ you used `ros2 param set` — it does not work here (§7.1); ⓑ you did not `colcon build --packages-select g1_perception_bringup`; ⓒ you did not restart the stack. Run `config_diff.py`: any `DIFFERENT` line is the answer |
| `/scan` is empty but `/points_self_filtered` is fine | the height band excludes everything (Stage A), **or** — if you set `target_frame: base_footprint` — the TF lookup is failing, i.e. DLIO is not publishing `odom → base_link` |
| `/scan` is full of a circular ring of points | `max_height` reached the floor. Lower it (§7-A limits) |
| Everything is **mirrored** left/right in the plot | old `dpcbf_plot_client` on Comp3 — `git pull` and rebuild (§3, the `f9da9c7` fix), or you set `target_frame:=''` in `dpcbf_scan_view` while `/scan` is in `mid360_link` |
| Circles appear where nothing is | Stage C went too far: `min_group_points` too low, or the merge distances fused the person with a wall. Re-run the slow pass |
| `uid` increments every frame | association is failing → `min_correspondence_cost` (§7-D-1) |
| Orange circle trails the dots | KF gain too low → `measurement_variance` (§7-D-2) |
| `/obstacles_safe` blank while `/tracked_obstacles` is fine | `max_age` or `max_circle_radius` in the safety filter (§7-E) |
| RViz stutters, `/odom` goes stale on the laptop | `/livox/lidar` is saturating the Wi-Fi. Untick it, or set its Reliability Policy to Best Effort (§5.3) |
| `qt.qpa.plugin: Could not load the Qt platform plugin "xcb"` | you launched a GUI inside an SSH session. Run it in a **local** Comp3 terminal (`hostname` to check) |
| Laptop shows no topics at all | link problem, not a tuning problem → Fast DDS manual §6-1. `ros2 daemon stop` first |
| DLIO drifts / `/odom` jumps after a restart | the robot moved during the 3 s IMU calibration. Restart the stack and stand still |

---

## 11. What to record

For each iteration, one line in a session log:

```
run  stage  changed                          slow  normal  fast  stopgo  toward  note
0    -      baseline (08-12 config)          OK    OK      S     N       R       legs only
1    A      min_height -0.15 / max 0.65      OK    OK      R     R       OK      torso now solid
2    C      min_group_points 3, merge 0.35   OK    OK      T     R       OK      grey stable
3    D      min_corr_cost 0.5, meas_var 0.04 OK    OK      OK    OK      OK      ← keep
```

Put into `$SESSION`:
- the screenshots (before/after, per pass),
- the bags from §9-3,
- `ros2 run g1_perception_bringup config_diff.py --json $SESSION/configs.json`
  (checksums of the exact config the bag was recorded with),
- `hw_obstacle_watch.py -p json:=$SESSION/watch_runN.jsonl` for the runs you
  want numbers from.

**At the end of the day**, commit the winning YAMLs from `src/` — with the
measured sensor height `H` and the fast-walk speed written into the comment,
so the next session knows what the numbers were fitted to.

---

## Related documents

- [`g1_first_day_field_runbook.md`](g1_first_day_field_runbook.md) — field
  network, staged bring-up and stop conditions
- [`operator_runbook.md`](operator_runbook.md) — simulation and development
  operations
- [`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) — verified hardware
  path and remaining measurement gates
