# Operator runbook — DPCBF perception on the dev machine

**What this is.** Every procedure needed to *run* what Phases 0–5A and the two
interim blocks built: bring the stack up, see it, walk the robot, reproduce any
number in §21, and diagnose it when nothing arrives. It is the dev-machine
counterpart to `phase5b_checklists.md` (the robot session) and follows the same
shape: **preconditions → exact command → expected observable → what failure
looks like**.

**What this is not.** It is not the architecture (that is
`../DPCBF_Perception_Subsystem_ROS2_Architecture.md`, the source of truth) and
not the workspace provenance — which externals are pinned, which patches exist
and why, what was built from source because `sudo` was unavailable. That stays
in [`../README.md`](../README.md). Nothing is duplicated between the two: if a
fact about *how to run something* used to live in the README, it moved here.

**Machine assumed.** The dev machine as of 2026-08-02: Ubuntu 22.04.5, ROS 2
Humble, 32 cores, broken NVIDIA driver (software GL everywhere), no gamepad,
`sudo` available only for the two apt packages already installed
(`rosbag2-storage-mcap`, `foxglove-bridge`). Branch `dpcbf_perception_ros2`.

---

## Verification status

Everything below was executed **in the order it is printed**, on this machine,
in one session (2026-08-02), and the "expected" text is the output that
actually came back. Where that is not true it says so inline, in bold, with the
reason. Do not assume an unmarked block is aspirational — assume a marked one
is.

| Marker | Meaning |
|---|---|
| *(unmarked)* | run this session; the observable is real output |
| **NOT RUN** | could not be executed here; the reason is stated at the block |
| **NOT VERIFIED** | the code path exists and is written up, but its stated observable was not seen |

Costs are wall-clock on this machine and are given so you can pick work by
budget, not so you can benchmark: `~1 s`, `~1 min`, `~5 min`, `~10 min+`.

**Coverage: 28 of the 35 runnable blocks were executed.** §4.6 (the live 2-D
overlay) was added in a later session and executed then — its live walking run,
bag-replay degradation, validation gate and CPU figures are all real output,
with one piece marked **NOT VERIFIED** inline (the T6 staleness *appearance*,
because the drill below was still not run). The seven that were
not, each marked at its own block: the from-scratch build (§2.2) and the
`simulate`/`deploy` builds (§2.4) — the workspace was already built and a cold
rebuild was out of budget; the one-line `bringup.launch.py` (§3.3) — every block
here drives the four launches separately; `foxglove_bridge` (§4.5) — no browser
client to connect with; `fsm_button_probe` standalone (§6.3) — the walking run
it gates succeeded, so there was no failure to point it at; the T6 staleness
drill (§6.6); and `hw_source_stub.py` by hand (§8.2) — the two launch tests that
drive it passed inside the suite. One block, the OpenCV visualizer (§4.4), is
partially verified and says so.

---

## 1. I want to…

| … | Section | Cost |
|---|---|---|
| get a shell that can talk to this stack at all | [2.1](#21-the-environment-block) | ~5 s |
| build from a fresh clone | [2.2](#22-fresh-clone--built-workspace) | ~40 min |
| rebuild after editing one package | [2.3](#23-incremental-builds) | seconds |
| build the simulator / the deploy FSM | [2.4](#24-simulate-and-deploy) | ~2–10 min |
| run the whole test suite | [5.1](#51-the-whole-suite) | ~5 min |
| run one gate | [5.2](#52-one-gate-at-a-time) | ~0.1 s – 5 min |
| **see the pipeline work with no simulator** (cheapest useful thing) | [3.1](#31-bag-replay--the-cheapest-full-pipeline) | ~45 s |
| run the live simulated stack | [3.2](#32-live-sim-stack) | ~2 min |
| use the one-line bringup instead of four launches | [3.3](#33-the-one-line-bringup) | ~2 min |
| **see it in RViz — including headless, with a screenshot** | [4.1](#41-rviz2), [4.2](#42-rviz-headless-with-a-real-screenshot) | ~1 min |
| see the DPCBF filter's own view (constraints, selected obstacles) | [4.4](#44-the-opencv-dpcbf_visualizer) | free |
| **make the robot walk and watch it** | [6](#6-walking) | ~1.5 min |
| get §17.3 collision rate and min clearance | [6.4](#64-the-walking-ab) | ~3 min/arm |
| reproduce T4 / T5 / T8 | [5.2](#52-one-gate-at-a-time) | ~1–5 min |
| prove the oracle-equivalence claim (T1) | [5.2](#52-one-gate-at-a-time) | ~4 s |
| run the offline A/B and containment sweep | [7.1](#71-offline-ab--containment-one-command) | ~4 min |
| characterise the circle-fit bias / find the radius cut | [7.2](#72-circle-fit-bias-sweep) | ~15 s |
| derive `measurement_variance` / `k_sigma` from a capture | [7.3](#73-the-two-calibrators) | seconds |
| see what the hardware path does, without hardware | [8](#8-hardware-path-without-hardware) | ~1 min |
| regenerate a fixture bag after a message change | [9.1](#91-fixture-bags) | ~2 min each |
| regenerate the evidence images | [9.2](#92-evidence-images) | ~1.5 min |
| **find out why I am seeing no data** | [10](#10-nothing-is-arriving--diagnostic-order) | ~2 min |

---

## 2. Getting to a working workspace

### 2.1 The environment block

**Every** shell that touches this workspace starts with this. It is six lines
and each one has a silent failure mode — that is why they are all here rather
than in your `.bashrc`, where you would stop seeing them.

```bash
export PATH=/usr/bin:$PATH; hash -r          # system python 3.10 BEFORE conda's 3.12
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # D2 — mandatory
export ROS_DOMAIN_ID=0                       # matches simulate/config.yaml & SDK2
# ONLY for sessions with a live simulator (see the note below):
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

**How to tell each one is wrong.** All five failures are silent; none of them
produces an error message on the line that caused it.

| Symptom | Cause | Check |
|---|---|---|
| `ModuleNotFoundError: rclpy`, or an ament_python node dies on import | conda python 3.12 shadowing 3.10 | `which python3` → must be `/usr/bin/python3` |
| `ros2 topic list` shows nothing at all | ROS not sourced | `echo $ROS_DISTRO` → `humble` |
| topics **listed** but every `echo`/`hz` hangs | RMW mismatch, or the `CYCLONEDDS_URI` mismatch below | `ros2 topic info -v <topic>` — publisher present, no data |
| two of everything, `/clock` at 2× rate | a stray simulator from a previous run | §3.4 |
| a launch test reports impossible counts | someone else's test on your domain | §5.3 |

> **The `CYCLONEDDS_URI` trap (Phase 2).** `simulate` derives a **`lo`-pinned**
> URI internally from `simulate/config.yaml` when the variable is unset, so its
> topics live on the loopback interface only. A process started *without* the
> same URI binds the default NIC, discovers the topic **names**, and receives
> **no data** — indistinguishable from a dead publisher in `ros2 topic list`.
> Export it for any session with a live simulator. Bag-replay-only sessions
> work either way (every endpoint then shares the default interface), which is
> exactly why Phases 0–2 never hit it.

### 2.2 Fresh clone → built workspace

**Preconditions:** the repo cloned, `vcstool` and `colcon` on PATH, network
access for the pinned checkouts.

```bash
cd ~/unitree_rl_mjlab_/ros2
./setup_external.sh                    # vcs import (pinned SHAs) + patches 0001–0009
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`setup_external.sh` is idempotent: each patch is `git apply --check`ed first and
prints `… already applied` rather than failing. **A patch that reports neither
"applied" nor "already applied" but an error is a real problem** — the pin moved
or the checkout is dirty; `git -C src/external/<repo> status` before anything
else.

`--merge-install` is **not optional** — `unitree_sdk2` and the ROS packages must
share one prefix so exactly one `libddsc.so.0` exists at runtime (R-3), and
colcon refuses to mix layouts afterwards (see §5.1 for the way this bites).

**NOT RUN this session** — the workspace was already built, and a from-scratch
rebuild was out of budget. The ~40 min figure is the phase-log estimate, not a
measurement taken here. What *was* measured: a no-op rebuild of one package
completes in **0.18 s**, and the externals tree is intact at its pins.

### 2.3 Incremental builds

```bash
colcon build --merge-install --packages-select g1_perception_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```
Finished <<< g1_perception_bringup [0.04s]
Summary: 1 package finished [0.18s]
```

Config files (`config/*.yaml`, `rviz/*.rviz`, `launch/*.py`) are **installed**,
not read from the source tree — editing one and re-launching without rebuilding
silently runs the old file. Rebuild the package after any config edit.

### 2.4 `simulate` and `deploy`

The simulator is plain CMake, not colcon, and is built **twice** on this machine
— once at baseline (`build/`, no ROS) and once with the ROS 2 module
(`build_ros2/`). Everything in this runbook uses `build_ros2/`.

```bash
cd ~/unitree_rl_mjlab_/simulate && mkdir -p build_ros2 && cd build_ros2
cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=ON \
      -DCMAKE_PREFIX_PATH=$PWD/../../ros2/install
make -j$(nproc)
```

Produces `unitree_mujoco` plus three instruments this runbook uses:
`t1_replay` (§5.2), `ab_eval` (§7.1), `fsm_button_probe` (§6.3).

**NOT RUN this session** — `build_ros2/` was already present and all four
binaries were exercised from it, so the tree is known good; the cmake/make lines
are transcribed, not re-executed.

`deploy`'s FSM controller (`g1_ctrl`) is needed only for walking (§6); it is
already built at `deploy/robots/g1/build/g1_ctrl`. **`deploy/` source is not
modified by this project** — the button probe *includes* its DSL header and
never edits it.

---

## 3. Running the stack

### 3.1 Bag replay — the cheapest full pipeline

No simulator, no sidecar, no GL, no `CYCLONEDDS_URI`. A recorded bag carries
`/livox/lidar`, the TF tree **and** `/clock`, so replaying it drives the whole
container with zero helper nodes. **This is the first thing to run when
something looks broken**, because it removes the simulator from the question.

```bash
cd ~/unitree_rl_mjlab_/ros2
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true &
sleep 5
python3 src/g1_perception/g1_perception_bringup/test/phase2_probe.py --duration 34 &
sleep 2
ros2 bag play test_fixtures/s1_surveyed
```

Cost: **~45 s** end to end (the bag is 30 s).

```json
"t9":    { "checked": 289, "misses": 0 },
"rates": { "livox_lidar": {"frames":290,"hz":10.0,"drop_fraction":0.0},
           "points_self_filtered": {"frames":290,"hz":10.0,"drop_fraction":0.0},
           "scan": {"frames":290,"hz":10.0,"drop_fraction":0.0} },
"latency_ms": { "cloud_to_filtered": {"p50":0.23,"p95":0.36},
                "cloud_to_scan":     {"p50":0.60,"p95":1.04} },
"cropbox_removed_points": { "median": 381, "max": 608 },
"scan_occupied_bin_fraction": { "mean": 0.0811 }
```

For the **full** chain to `/obstacles_safe` (the §17.2 latency budget) swap the
probe for `phase4_latency_probe.py 34 /tmp/lat.txt`:

```
cloud->scan:    n=290 p50=0.49ms p95=0.95ms p99=1.01ms max=1.13ms
cloud->tracked: n=290 p50=0.67ms p95=1.16ms p99=1.25ms max=1.30ms
cloud->safe:    n=290 p50=0.76ms p95=1.28ms p99=1.37ms max=1.41ms
container CPU: 2.9% of one core over 34s
frames: cloud=291 scan=290 tracked=290 safe=290
```

**Failure looks like:** `frames: cloud=N scan=0`. Two causes, in order of
likelihood — (1) **`pointcloud_to_laserscan` subscribes lazily**: it only
subscribes to `cloud_in` while `/scan` has at least one subscriber, so with no
probe and no RViz attached the chain sits idle and looks dead; (2) a stale
container from the previous run captured the LoadNode RPC (§3.4).

**Which bag to replay:**

| fixture | what it is | why you'd replay it |
|---|---|---|
| `s1_surveyed` | 3 surveyed cylinders r=0.15 at 1/2/3 m, static, 30 s | T4, T8, the default "does it work" |
| `s2_cross_05`, `s2_cross_08` | one cylinder crossing at 0.5 / 0.8 m/s | T5 |
| `s3_swarm` | 20-obstacle seeded swarm | the density limit; containment tail |
| `s4_occlusion` | blocker + occluded crosser inside `p_max` | track death and re-acquisition |
| `s1_static_reference` | Phase-1 90-obstacle field, robot suspended | the projection-chain regression |

All are gitignored; regenerate per §9.1.

### 3.2 Live sim stack

Four processes. The simulator needs *a* display for its GLFW context — it does
**not** need yours: a private `Xvfb` works and keeps the run headless and out of
your desktop.

**Precondition — the shadow run-tree.** The simulator resolves `config.yaml`
relative to its own executable and `dpcbf/config/dpcbf_config.yaml` one level
above that, so it cannot be run in-place with modified settings without editing
tracked files. Every live session in this project therefore runs from a
throwaway copy. **This step is assumed by `phase4_live_session.sh` and was
documented nowhere until now:**

```bash
REPO=~/unitree_rl_mjlab_; SHADOW=/tmp/sim_shadow
rm -rf $SHADOW && mkdir -p $SHADOW/simulate/build $SHADOW/dpcbf/config
ln -sfn $REPO/src $SHADOW/src                      # scene assets
cp $REPO/simulate/build_ros2/unitree_mujoco $SHADOW/simulate/build/
cp $REPO/dpcbf/config/dpcbf_config.yaml    $SHADOW/dpcbf/config/
sed -e 's/^use_joystick: .*/use_joystick: 0/' \
    -e 's/^enable_elastic_band: .*/enable_elastic_band: 1/' \
    $REPO/simulate/config.yaml > $SHADOW/simulate/config.yaml
```

`use_joystick: 0` is mandatory here: there is no `/dev/input/js0` on this
machine and the stock config has no CLI override, so the tracked config exits at
startup. **Never edit the tracked `simulate/config.yaml`** — that is what the
shadow tree is for.

Then:

```bash
Xvfb :77 -screen 0 1920x1080x24 -nolisten tcp &          # private display
cd $SHADOW/simulate/build
env LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa DISPLAY=:77 \
    UNITREE_DPCBF_MODE=oracle \
    UNITREE_MUJOCO_SCRIPTED_COMMANDS=$REPO/ros2/test_fixtures/t1_baseline/t1_command_profile.txt \
    ./unitree_mujoco &
sleep 14                                                  # model load + mirror dump + DDS up

cd ~/unitree_rl_mjlab_/ros2
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=true &
ros2 launch g1_perception_bringup source_sim.launch.py &
ros2 launch g1_perception_bringup perception.launch.py use_sim_time:=true &
```

Cost: **~26 s to first `/obstacles_safe`**, then it runs indefinitely.

Expected, measured over 45 s with the 90-obstacle field moving:

```
cloud->scan:    n=450 p50=0.66ms p95=1.17ms p99=2.12ms max=3.37ms
cloud->tracked: n=450 p50=1.26ms p95=1.91ms p99=2.73ms max=3.69ms
cloud->safe:    n=450 p50=1.48ms p95=2.18ms p99=3.02ms max=3.93ms
container CPU: 4.6% of one core over 45s
sidecar CPU: 30.3%
frames: cloud=451 scan=450 tracked=450 safe=450
```
```json
"t9": { "checked": 449, "misses": 0 },
"rates": { "livox_lidar": {"frames":451,"span_s":45.0,"hz":10.0,"drop_fraction":0.0} }
```

The sidecar at ~30 % of a core is the raycast (24 000 rays at 10 Hz); the
container at ~4.6 % is the entire perception chain. The two are the §17.4
budget.

**The 14 s wait matters.** `simulate` dumps the *compiled* mirror model to
`/tmp/unitree_mujoco_mirror_model.xml` during load, and the sidecar loads **that
file, never the raw scene** — the 90 obstacle mocap bodies are added to the spec
at runtime and are absent from `scene_g1.xml`. Start the sidecar first and it
dies on a missing file, or worse, loads a stale one from a previous scene.

**Sim time.** Everything downstream runs `use_sim_time:=true` against the
simulator's `/clock`. The one deliberate exception is the DPCBF adapter inside
`simulate`, which runs `use_sim_time=false` — it must not consume the `/clock`
its own process publishes; its safety ages are sim-time `d->time` against
sim-time header stamps.

### 3.3 The one-line bringup

`bringup.launch.py` is the switchboard over the four launches above:

```bash
ros2 launch g1_perception_bringup bringup.launch.py \
      source:=sim|hw  mode:=oracle|shadow|estimated \
      ground_seg:=off|patchwork  viz:=off|rviz  record:=off|on
```

**NOT RUN this session** — every live block below and above was driven by the
four launches separately (§3.2), because that is what every harness in the tree
does and what you want while debugging. The composition is read from
`bringup.launch.py`, not measured.

It still expects `simulate` to be running for `source:=sim` (it starts the
sidecar, not the simulator). `use_sim_time` defaults from `source` — true for
sim, false for hw — and that single `PythonExpression` is the **only** sim/hw
conditional outside the two `source_*` files; `perception.launch.py` has none by
design (D4).

Start the pieces separately (§3.2) whenever you need to restart one stage
without the others — in practice that is most debugging.

### 3.4 Teardown — read this before your second run

Stopping a launch with **SIGTERM orphans the composable container**. The stale
`/perception_container` then wins the race for the *next* launch's LoadNode RPC
and the new container comes up empty: no error, no components, no data. Always:

```bash
pkill -INT -f 'rclcpp_components/component_containe[r]'; sleep 2
pkill -KILL -f 'rclcpp_components/component_containe[r]'
pkill -f 'sim_mjlidar_bridg[e]'
pkill -INT -f 'unitree_mujoc[o]'; sleep 1; pkill -KILL -f 'unitree_mujoc[o]'
pkill -f 'robot_state_publishe[r]'; pkill -f 'base_footprint_publishe[r]'
```

Two bracket traps, both of which cost this project a debugging cycle each:

- **`pkill -f component_container` kills the shell running it.** The shell's own
  command line contains the pattern, so `pkill` matches itself and the rest of
  your script never executes. It presents as a silent hang. Always bracket a
  character: `'component_containe[r]'`.
- **`pkill -f simulate/build/unitree_mujoco` matches nothing.** The process
  cmdline is `./unitree_mujoco`, so a path-qualified pattern misses it and a
  stray 1 kHz simulator keeps publishing. The tell is doubled `/clock` and
  `/dpcbf/status`. Use `'unitree_mujoc[o]'`.

---

## 4. Seeing it

### 4.1 RViz2

```bash
ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true
```

Starts **four** marker relays plus RViz on the committed layout. On this machine
RViz needs the software-GL variables like everything else:

```bash
env LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
    ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true
```

| Display | Topic | Colour | What it answers |
|---|---|---|---|
| LivoxCloud | `/livox/lidar` | red points | is the sensor producing anything |
| GTObstacles | `/sim/gt_obstacles` via relay | green, uid labels | where the obstacles really are (sim only) |
| RawObstacles | `/raw_obstacles` via relay | grey, 0.8 m tall | what the **extractor** found, per frame |
| TrackedObstacles | `/tracked_obstacles` via relay | orange, 1.2 m, uid labels | what the **tracker** believes, with identity |
| SafeObstacles | `/obstacles_safe` via relay | red, 1.5 m, translucent | **what DPCBF is actually told** |
| DPCBFOverlay | `/dpcbf_overlay/markers` | multi | **§4.6** — estimated-vs-GT error and the DPCBF constraint geometry |
| Odom | `/odom` | arrow trail | is odometry moving |
| Scan | `/scan` | — | off by default; enable to debug the projection band |

The relays are cylinders of *different heights on purpose* so all four layers
are legible stacked on the same obstacle.

**The layout now opens in top-down 2-D** (`TopDownOrtho`, target `base_link`),
because that is the plane the filter reasons in; the previous `Orbit` camera is
kept as a saved view named `Orbit3D`. Three displays changed default state at
the same time, all for legibility in 2-D and all still one tick away:
`LivoxCloud` and `RawObstacles` now start **off** (the cloud covers every other
layer from above), and `Odom` keeps 12 arrows rather than 100 (a 100-arrow
trail fans across exactly the area the §4.6 layers occupy).

**Reading a correct picture** — this is what a working stack looks like, and
each item is a different failure if absent:

- red points form **arcs**, not a full disc: the LiDAR sees surfaces, and there
  is a **shadow behind every obstacle**. No shadows ⇒ suspect the raycast, not
  the detector.
- orange circles sit **on** the green circles, slightly larger and pulled a few
  cm *towards* the robot (the circle fit's `−0.278·r` centre bias and `+0.084·r`
  radius bias — both are expected and are what `fixed_inflation` covers).
- red safe circles are concentric with the orange ones and larger by the
  inflation.
- uid labels are **stable** as an obstacle moves. A uid that increments every
  frame is a tracking failure, not a rendering one.
- near the robot, a ring of returns at ~0.3 m is the **wrist self-hit** the
  Phase-3 CropBox interim removes; if you see circles fitted to it, the CropBox
  parameters are not the shipped ones.

**SafeObstacles was added to the layout in this session.** `viz.launch.py` has
published that relay since Phase 4, but `perception.rviz` had no display bound
to it — so the one stream with safety meaning was the one you could not see. It
is now the fifth display.

### 4.2 RViz headless, with a real screenshot

The interactive-RViz screenshot has been a carried follow-up since Phase 1
("GNOME blocks unattended screenshots"). It is not blocked: run RViz against a
**private Xvfb** and grab that display. No compositor, no desktop, nothing on
your screen.

```bash
Xvfb :77 -screen 0 1920x1080x24 -nolisten tcp &
sleep 3
env DISPLAY=:77 LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
    ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true &
sleep 20                                     # RViz start + display discovery
# … drive data at it (bag replay §3.1 or live sim §3.2) …
python3 -c "from PIL import ImageGrab; ImageGrab.grab(xdisplay=':77').save('/tmp/rviz.png')"
```

Cost: **~45 s** including a bag replay. RViz renders at **31 fps** under
software GL on Xvfb — interactive enough to be worth using, and the status bar
reads `RViz is ready.` when it is.

Evidence produced this way:
[`../evidence/runbook/rviz_bag_replay.png`](../evidence/runbook/rviz_bag_replay.png)
— the `s1_surveyed` replay with all five layers on: cloud arcs with shadows,
three tracked cylinders on their GT circles, uid labels 0/1/2, safe inflation
around each.

> `PIL.ImageGrab.grab(xdisplay=…)` works on Pillow 9.0.1 (the system one). It
> grabs the **whole display**, so give RViz its own — pointing it at `:1` would
> capture the operator's desktop.

### 4.3 Offscreen overlay (no RViz at all)

When what you want is the *measurement* rather than a 3-D view —
tracked-versus-GT per obstacle, through a whole walking run — use the matplotlib
Agg renderer instead. It needs no display and shows strictly more than a
screenshot: per-obstacle error rather than a percentile.

```bash
ros2/src/g1_perception/g1_perception_bringup/test/walk_overlay_run.sh \
      /tmp/walk_overlay.png W1 75
```

Cost: **1 min 32 s** (it brings up the whole walking stack itself, §6). Output:
five panels — four time slices plus the base trajectory.

```
W1: S1-like: sparse static field
  count=6 radius=[0.2, 0.3] speed=[0.0, 0.0] arena=[8.0, 8.0] seed=20260802
wrote /tmp/rb_walk_overlay.png from 2747 frames (50.0..80.5 s)
```

Reproduced this session:
[`../evidence/runbook/walk_overlay_reproduced.png`](../evidence/runbook/walk_overlay_reproduced.png).
Red solid = tracked, blue dashed = GT, grey = `/scan`, green = robot and its
`p_max` horizon. **A late panel showing `0 tracked, 6 GT` is not a failure** —
it means the robot walked out of the field, which the trajectory panel will
confirm.

### 4.4 The OpenCV `dpcbf_visualizer`

The DPCBF library ships its own top-down visualizer showing what the *filter*
sees — the obstacles it selected, the active constraints, the velocity
boundaries. It is sim-only, in-process, has never been documented, and is
**already on**: `dpcbf/config/dpcbf_config.yaml` has `visualization.enabled:
true`, so `simulate` opens a 1500×900 OpenCV window on whatever `$DISPLAY` it
was given, at 30 Hz.

Use it instead of RViz when the question is *"why did the filter do that"*
rather than *"what does perception see"* — it is the only view of the QP's
inputs.

To turn it off (or resize it), edit the copy in your **shadow run-tree**
(§3.2) — `dpcbf/` is frozen (D3) and must not be edited in the repo:

```bash
sed -i 's/^    enabled: true/    enabled: false/' $SHADOW/dpcbf/config/dpcbf_config.yaml
```

**Partially verified.** The window was observed on a headless Xvfb display
during a live oracle-mode run, rendering its
`Waiting for MuJoCo state...` placeholder frame — the frame it draws before the
1 kHz seam first calls `Update()`. I could not confirm that it *refreshes* on
Xvfb (the seam was demonstrably running — the simulator log carries OSQP
messages from the same loop), and I did not open it on the operator's desktop to
check. Treat it as a real-display tool; on Xvfb, trust the Filter-I/O capture
(§6.5) instead.

### 4.5 Foxglove and MCAP

Both apt packages are now installed (`ros-humble-foxglove-bridge`,
`ros-humble-rosbag2-storage-mcap`) — the operator ask that blocked this since
Phase 0 is closed.

```bash
ros2 run foxglove_bridge foxglove_bridge          # then connect to ws://localhost:8765
```

**NOT RUN this session** (no browser client here to connect with — the bridge
starting proves nothing on its own).

`record.launch.py` still writes **sqlite3**: switching its storage to MCAP is an
ordinary follow-up, not a blocker. Until then, record MCAP by hand when you want
it:

```bash
ros2 bag record -s mcap -o /tmp/mybag --include-unpublished-topics \
    /livox/lidar /livox/imu /odom /tf /tf_static /scan \
    /raw_obstacles /tracked_obstacles /obstacles_safe /sim/gt_obstacles \
    /dpcbf/status /clock
```

### 4.6 The live 2-D overlay — estimated vs GT, and the DPCBF constraint

`dpcbf_overlay` is the sixth marker layer. It answers two questions at once:
*how wrong is perception, per obstacle, right now* — and *what is the filter
actually constraining*.

**Read this first: the "parabola" is not in the world.** The DPCBF barrier is

```
h = x̃ + A·( λ·ỹ² + k_μ·d_safe )      A = √(s²−1)/r_safe,  λ = k_λ·d_safe/v_safe
```

so `h = 0` is `x̃ = vertex_x − curvature·ỹ²`, and **`(x̃, ỹ)` is the relative
velocity `v_obs − v_robot` rotated into the obstacle's line of sight**
([`dpcbf_safety_filter.cpp:404`](../../dpcbf/src/dpcbf_safety_filter.cpp#L404),
coefficients named at
[`:590`](../../dpcbf/src/dpcbf_safety_filter.cpp#L590), and stated outright at
[`dpcbf_safety_filter.h:47`](../../dpcbf/include/dpcbf/dpcbf_safety_filter.h#L47)).
Both axes are **m/s**. Drawing that curve on the `odom` grid would be a
category error, so it is drawn in its own TF frame, `dpcbf_velocity_plane`,
parked beside the robot, and every card says "axes are m/s, NOT metres".

> The OpenCV `dpcbf_visualizer` (§4.4) *does* draw a world-frame curve. It is
> the same velocity-space curve multiplied by `velocity_arrow_seconds = 1.0`
> and anchored at the **robot**, i.e. metres = (m/s)×(1 s) — a one-second
> lookahead diagram, not an envelope around the obstacle. Its own config
> bounds it in m/s (`world_parabola_lateral_limit`) on a pane scaled in m.
> Do not read it as workspace geometry, and do not port it as one.

**Preconditions.** A live run — a bag replay has the filter out of the loop, so
`/dpcbf/status` never appears and the constraint layer is drawing against
nothing. The overlay needs the **same** `dpcbf_config.yaml` the simulator
loaded; when you run from a shadow tree (§3.2) pass that copy, not the repo's.

```bash
export DISPLAY=:77                      # private Xvfb, §4.2
ros2/src/g1_perception/g1_perception_bringup/test/walk_overlay_rviz_run.sh \
      /tmp/wov W4 estimated
```

One command: shadow tree, `g1_ctrl` first, staggered chords, band at `34,6`,
perception, RViz with `overlay:=on`, a 1 kHz `UNITREE_DPCBF_FILTER_LOG`
capture, the 10 Hz overlay JSONL, two screenshots and per-process CPU.
Cost: **2 min 22 s**. To add it to a stack you already have up:

```bash
ros2 launch g1_perception_bringup viz.launch.py use_sim_time:=true \
      overlay:=on dpcbf_config:=$SHADOW/dpcbf/config/dpcbf_config.yaml \
      overlay_log:=/tmp/overlay.jsonl        # log optional; needed for the join
```

**`perception.rviz` and `viz.launch.py` are installed, not read from source
(§2.3). Rebuild `g1_perception_bringup` after editing either.** This cost a
live run in the session that wrote this section: the layout and the launch file
were both edited, neither was rebuilt, and the run came up with the old four
relays and no overlay node — silently, because a launch argument that selects
nothing looks exactly like a launch argument that is off.

**The layer stack.** Under `TopDownOrtho` z is not height, it is **draw order** —
that is the mechanism for the deliberate overlap.

| Layer (marker ns) | Form | z | Reads as |
|---|---|---|---|
| `gt` | thin unfilled ring, green | 0.02 | the reference never occludes what is measured |
| `tracked` | translucent filled disc, orange | 0.04 | the estimate *covers* the truth; offset visible through it |
| `safe` | outline only, red | 0.06 | an inflation of tracked, not a measurement |
| `error` | GT-centre→tracked-centre segment + `mm` text | 0.08 | the number, per obstacle |
| `unpaired_gt` | cyan ring + `MISS` | 0.03 | a missed detection, not a rendering gap |
| `unpaired_tracked` | magenta ring + `FP?` | 0.07 | a false positive |
| `ecbf_barrier` | ring at `r_rob+r_obs`, orange (red if breached) | 0.09 | the **eCBF** family's zero set — real world geometry |
| `selected` | ring at `r_safe`, blue, `#n h=…`, spoke to robot | 0.10 | which obstacles become QP rows, in row order |
| `pmax`, `status` | horizon ring, heading, banner | — | scope and state |
| `vel_*` | the h=0 parabolas | *(other frame)* | **m/s**, in `dpcbf_velocity_plane` |

Positions are **never magnified** — the expected bias is a few cm (§4.1) and
falsifying it produces a picture nobody can quote. `error_magnify` exists, and
whenever it is not 1.0 the factor is printed in an always-visible marker.

**Reading a correct picture**, in the register of §4.1:

- orange discs sit **on** the green rings, pulled a few cm toward the robot;
  the `mm` label is the number to quote, and it agrees with
  `tracked_to_gt_p50_mm` from the same run because the pairing rule is
  `walk_ab_probe.py`'s verbatim (scope `p_max`, tracked→nearest GT, 0.5 m cap).
- blue `selected` rings are **not** simply the nearest obstacles. The shipped
  config has `obstacle_priority: 1`, so the order is **closing alignment
  first**, distance only as tie-break — a far obstacle closing head-on
  outranks a near one drifting away. Expect the ranks to reshuffle as the
  robot turns.
- the p_max gate is **centre-to-centre**; an obstacle whose surface is inside
  3.0 m but whose centre is outside is *not* constrained, and correctly not
  ringed.
- cyan `MISS` rings clustered behind another obstacle are occlusion, not
  tracker failure — check for the LiDAR shadow before blaming the tracker.
- the velocity cards' shaded half is `h < 0`. The dot is the current `(x̃, ỹ)`;
  when it sits inside the shading that constraint is violated **now**.
- **both constraint families are drawn** because the shipped config runs
  `ecbf_enabled: true` *and* `slack_enabled: true`: the parabola (DPCBF) and
  the `r_rob+r_obs` circle (eCBF). Showing only the parabola would understate
  what shapes the command; both are soft, and the card says so.

**What failure looks like.**

| Symptom | Almost certainly |
|---|---|
| no `dpcbf_overlay` in `viz.log`, four relays only | `g1_perception_bringup` not rebuilt after editing `viz.launch.py` |
| node exits at startup with a `dpcbf_config` message | the parameter is required *by design* — deriving the geometry from ROS defaults would let it drift from the filter silently |
| `GT UNAVAILABLE` banner, no green/cyan/`mm` layers | expected on hardware and on bag replays without `/sim/gt_obstacles`; the estimated layers are still live |
| `mode=? dpcbf=? ?s` | no `/dpcbf/status` — a replay, or the seam is not running (§10) |
| `sel 0/N` with obstacles visibly close | they are outside `p_max` centre-to-centre, or `/obstacles_safe` is stale — check the banner's age |
| the whole view is red points | `LivoxCloud` got re-enabled; it covers everything in 2-D and ships off |

**Staleness is on the banner and in the colours.** During the T6 drill (§6.6)
the retained obstacle set inflates and never empties; the `tracked` discs shift
to purple and the banner gains `*** stop: SET RETAINED ***`, so a frozen set
cannot be read as a live one.

> **NOT VERIFIED.** The colour-shift and the STOP banner are implemented and
> driven by `/dpcbf/status`'s level, and the banner was observed rendering its
> `fresh` form live and its `mode=? dpcbf=?` form on a replay — but the T6
> drill itself was not run this session, so the DEGRADE/STOP *appearance* is
> written, not seen. Running §6.6 with `overlay:=on` closes both this and the
> drill's own unexecuted status in one go.

#### The validation gate

The recomputation is proven against the **frozen library**, not trusted:

```bash
cd simulate/build_ros2 && ctest -R dpcbf_boundary_recomputation --output-on-failure
```

`boundary_check math` replays every recorded `Filter()` call, runs the real
`DpcbfSafetyFilter` as the oracle and `dpcbf_boundary.h` (the header the
overlay itself uses) on identical inputs, and compares the selected set, its
order, and every coefficient. **Result: 38 402 ticks, 214 085 selected-obstacle
rows, every delta identically 0 — bit-exact, not "within tolerance."** The
stated tolerance is `1e-12`, and it is a round-off budget, not a tuned number:
both sides do the same operations in the same order on the same doubles.
Cost **3.1 s**; it SKIPs (77) without the capture fixture.

That gate covers the *math*. It cannot cover the *inputs*, so the second mode
prices those separately — and this is the number that constrains how far you
can trust the constraint layer live:

```bash
simulate/build_ros2/boundary_check join /tmp/wov/capture.bin \
      /tmp/wov/overlay.jsonl $SHADOW/dpcbf/config/dpcbf_config.yaml
```

**Join rule: nearest-preceding capture tick** (last 1 kHz record with
`t ≤ t_safe`), max gap 0.15 s, no interpolation — the capture's obstacle set is
piecewise-constant between perception frames, so interpolating would invent
states the filter never saw. Keyed on the sim-time stamp of the
`/obstacles_safe` frame the overlay consumed, not on wall time.

Measured on a W4 walking run (35.8 m, no fall,
[`../evidence/overlay/boundary_join_w4.txt`](../evidence/overlay/boundary_join_w4.txt)):

```
overlay records 1186, joined 1186, dropped 0 (gap) + 0 (no preceding tick)
selected set identical on 751/1186 ticks (63.3%)
set disagreement (Jaccard): mean 0.0610
|d body speed| (differentiated vs exact): mean 0.0482  max 0.5434 m/s
|d vertex_x|: mean 0.1909  max 1.177 m/s
|d curvature|: mean 0.2335  max 6.912 s/m
```

**Why it is not exact, and why no tuning fixes it.** The seam gets the 1 kHz
instantaneous pelvis twist; the overlay must *differentiate* `/odom` pose at
100 Hz, because **`sim_mjlidar_bridge` publishes no twist** — `MjState` carries
no `qvel` by design ([`bridge_node.py:9`](../src/g1_perception/sim_mjlidar_bridge/sim_mjlidar_bridge/bridge_node.py#L9)).
Sweeping the smoothing constant offline against the capture's exact velocities
over a walking window: mean `|dv|` **0.032** m/s at `τ=0`, 0.032 at 0.02, 0.049
at 0.05, **0.077 at 0.15** — smoothing *costs* accuracy here, because `/odom`
pose is exact and lag, not noise, dominates. The shipped `vel_tau_s` is
**0.02 s**: `τ=0`'s mean with a better p95 (0.055 vs 0.094 m/s).

Set membership is close (Jaccard 0.061 ⇒ ~94 % overlap); it is the **ordering**
that diverges, because ranking by closing alignment reshuffles on millimetre-
per-second differences. So: the workspace layers are authoritative — they are
read straight off the topics — and **the `selected` and `vel_*` layers are
indicative**, which they say on screen rather than only here.

> A run that **fell** produces wildly different numbers (mean `|dv|` 0.554 m/s,
> peaks past 8 m/s) because a thrashing pelvis is not a walking one. Always
> check `fell_at_s` before quoting a join. W4 fell at band release in 3 of 6
> attempts this session; W1 walked every time.

#### Cost

Measured over 45 s of a walking run, `% of one core`
([`proc_cpu.py`](../src/g1_perception/g1_perception_bringup/test/proc_cpu.py)):

| Process | W1 | W4 (90 obstacles) | §17.4 budget |
|---|---|---|---|
| `dpcbf_overlay` | **1.44 %** | **2.02 %** | new |
| `component_container` | 4.68 % | 5.31 % | 4.6 % |
| `sim_mjlidar_bridge` | 27.75 % | 35.92 % | 30.3 % |
| `rviz2` | 119 % | 191 % | — (viewer, software GL) |

The container and sidecar sit on their budget, so the overlay moves neither —
it is a separate process outside the perception container and subscribes only.
**RViz under software GL is by far the most expensive thing on the machine**;
that is the pre-existing cost of looking, not of this layer.

**Evidence:**
[`../evidence/overlay/rviz_dpcbf_overlay_w4.png`](../evidence/overlay/rviz_dpcbf_overlay_w4.png)
— W4 walking, five blue `selected` rings, cyan `MISS` badges, orange discs on
green rings, and three velocity cards with the shaded `h < 0` half.

### 4.7 Watching it live on the real desktop

§4.6's `walk_overlay_rviz_run.sh` is an **evidence harness**: it starts its own
`Xvfb :77`, forces `DISPLAY` at every child, and tears the stack down the moment
`walk_ab_probe.py`'s timer expires. Both behaviours are wrong when you want to
*watch*. Use the interactive launcher instead:

```bash
ros2/src/g1_perception/g1_perception_bringup/test/run_live_w4_view.sh W4 estimated
```

No Xvfb; everything inherits `DISPLAY` (default `:1`, refusing `:77`/`:1001`/
`:1002` outright). It brings up g1_ctrl → simulator → *waits for the mirror
dump* → description → sidecar → perception → RViz, prints the sim-time
timeline, then blocks on `wait` until **Ctrl+C**.

Two differences from the batch path, both deliberate:

- **It re-enables the OpenCV `dpcbf_visualizer`.** `walk_scenarios.py` flips
  `visualization.enabled` to `false` when it writes the shadow config ("no
  place in a headless batch run"), which silently suppresses the filter's own
  velocity-boundary window — the one view of the QP's inputs (§4.4).
- **It opens RViz on a copy of the layout with `LivoxCloud` enabled**, via the
  `rviz_config` argument added to `viz.launch.py`, so the raw cloud is visible
  without editing the tracked file and rebuilding.

Verified on this machine: FSM `Passive → FixStand → Velocity`, `/livox/lidar`
10.07 Hz, `/tracked_obstacles` 10.05, `/obstacles_safe` 10.05, `/dpcbf/status`
10.00, `/dpcbf_overlay/markers` 10.00, and all three windows mapped on `:1`
(`unitree_mujoco`, `rviz2`, `DPCBF Top-Down and Velocity Boundaries`).

> The OpenCV window opens **320×240** and is `WINDOW_NORMAL`, so drag it larger
> — `visualization.window_width/height` in the config is not honoured until the
> first frame sizes it. It is nested under the simulator's window group, so
> `xwininfo -root -children` will not list it; use `xwininfo -root -tree`.

---

## 5. Tests and gates

### 5.1 The whole suite

```bash
cd ~/unitree_rl_mjlab_/ros2
colcon test --merge-install --packages-select \
    dpcbf_ros_adapter safety_obstacle_filter g1_perception_utils \
    g1_description g1_perception_bringup sim_mjlidar_bridge
for p in dpcbf_ros_adapter safety_obstacle_filter g1_perception_utils \
         g1_description g1_perception_bringup sim_mjlidar_bridge; do
  echo "--- $p"; colcon test-result --test-result-base build/$p | tail -1
done
```

Cost: **4 min 54 s**, essentially all of it `g1_perception_bringup`'s launch
tests — the other five packages run in parallel and finish in under 4 s
combined (`dpcbf_ros_adapter` 3.4 s, the rest ≤0.6 s each).

```
--- dpcbf_ros_adapter        Summary: 26 tests, 0 errors, 0 failures, 0 skipped
--- safety_obstacle_filter   Summary: 16 tests, 0 errors, 0 failures, 0 skipped
--- g1_perception_utils      Summary:  5 tests, 0 errors, 0 failures, 0 skipped
--- g1_description           Summary:  2 tests, 0 errors, 0 failures, 0 skipped
--- g1_perception_bringup    Summary: 41 tests, 0 errors, 0 failures, 2 skipped
--- sim_mjlidar_bridge       Summary:  7 tests, 0 errors, 0 failures, 0 skipped
```

**97 tests** (89 before §4.6; `dpcbf_ros_adapter` gained 8 for the overlay's
recomputed constraint geometry). Two things about that number:

- **`--merge-install` is required on `colcon test` too.** Without it you get
  `ERROR:colcon:colcon test: The install directory 'install' was created with
  the layout 'merged'` — and, because `colcon test-result` reads whatever XML is
  already on disk, the follow-up command then prints the **previous** run's
  green summary. The failure is loud but its consequence is silent. (This
  runbook's first executed command found exactly this: the README's documented
  `colcon test` line does not work in this workspace.)
- **Scope the query to the project packages.** `colcon test-result --all` from
  the workspace root reports something like *1466 tests, 815 failures* — every
  one a lint test in an **external** package (`rmw_cyclonedds_cpp`'s `xmllint`
  cannot fetch its XSD without network, and so on). Those have never been in
  this project's count.

**The two expected skips**, so nobody chases them:

| Test | Skips because |
|---|---|
| `test_bringup_sim.launch_test.py` | no live simulator is running (it deliberately sits on domain 0 to talk to one) |
| `hw_config_check` | `MID360_config.json` still carries the upstream placeholder IPs (Q-1); it exits 2, which CTest maps to SKIP |

### 5.2 One gate at a time

| Gate | What it proves | Command | Cost / result here |
|---|---|---|---|
| **T1** oracle equivalence | the seam refactor did not change one bit of `Filter()` I/O across 38 402 recorded calls | `cd simulate/build_ros2 && ctest -R t1 --output-on-failure` | **3.79 s — Passed** |
| **boundary** recomputation | the overlay's constraint geometry is identical to the frozen filter's own `selected_obstacles` — set, order and every coefficient | `cd simulate/build_ros2 && ctest -R dpcbf_boundary --output-on-failure` | **3.07 s — Passed**, 38 402 ticks / 214 085 rows, all deltas exactly 0 (§4.6) |
| **T2** wall occlusion | zero through-wall rays (the H-3 risk the whole sim rests on) | `python3 src/g1_perception/sim_mjlidar_bridge/test_gates/t2_wall_occlusion.py` | **0.11 s — PASS**, 17 608 expected hits, 0 through, max range err 0.00 mm |
| **T3** pattern envelope | `mid360.npy` stays inside the datasheet FOV | `python3 …/test_gates/t3_pattern_envelope.py` | **0.09 s — PASS**, elevation −7.2123…+52.1640°, 99.782 % strict, 0 hard outliers |
| **T4** static accuracy | centre ≤0.10 m, radius ≤0.05 m, ≤2-frame latency | `launch_test src/…/test/test_detection_static.launch_test.py` | in the suite; needs `s1_surveyed` |
| **T5** dynamic tracking | velocity RMSE <0.1 m/s, ≤1 ID swap / 10 s | `launch_test …/test_tracking_dynamic_{05,08}.launch_test.py` | in the suite; needs `s2_cross_*` |
| **T7 / T7-hw** extrinsic guards | MJCF site ≡ xacro; DLIO's extrinsics re-derived from the xacro | `colcon test --merge-install --packages-select g1_description` | in the suite, 2 tests |
| **T8** replay determinism | two replays give bit-identical `/raw_obstacles` **and** `/tracked_obstacles` | `python3 src/…/test/test_t8_replay_determinism.py` | in the suite (CTest `t8_replay_determinism`), ~2 replays |
| **T9** TF availability | `odom→base_footprint` resolves within 50 ms of every cloud | asserted inside the replay/wall tests; also printed by `phase2_probe.py` | 289/0 and 449/0 misses this session |
| **T10** DDS coexistence | one process links SDK2 **and** rclcpp without a domain-creation fight | `install/t10_dds_coexistence/lib/t10_dds_coexistence/t10_smoke` | §5.4 |
| hardware cloud contract | the driver's real 26-byte record traverses the unmodified stack | `launch_test …/test_hw_source_contract.launch_test.py` | in the suite |
| DLIO wiring | subscriptions, QoS, `/odom` rate, frame parentage | `launch_test …/test_dlio_wiring.launch_test.py` | in the suite |

To run a single launch test outside colcon (useful when you want its stdout):

```bash
launch_test src/g1_perception/g1_perception_bringup/test/test_detection_static.launch_test.py
```

**T4 against a hardware bag** is the *same file*, pointed at a session:

```bash
T4_BAG=/path/to/hw_bag T4_LAYOUT=/path/to/t4_layout.yaml T4_USE_SIM_TIME=false \
    launch_test src/g1_perception/g1_perception_bringup/test/test_detection_static.launch_test.py
```

### 5.3 Why the tests do not collide

Each launch test pins itself to a private `ROS_DOMAIN_ID` at import time, before
`rclpy.init` and before any launch action exists (`test/isolate_domain.py`;
registry 41–49). Without it, concurrent tests see each other's topics and report
arithmetically impossible counts — a 278-cloud replay probe reporting 597
clouds. The assignment is **unconditional** because the documented workspace
default exports `ROS_DOMAIN_ID=0`, so an "only if unset" rule would never fire.

To point one test at a **live simulator** instead, force its domain:

```bash
PERCEPTION_TEST_DOMAIN=0 launch_test src/…/test/test_bringup_sim.launch_test.py
```

That is also the only way to make `test_bringup_sim` do anything other than skip.

### 5.4 T10, and what a DDS failure looks like

```bash
install/lib/t10_dds_coexistence/t10_smoke
```

Cost: **10.4 s — PASS.**

```
[t10] init order: ROS2 -> SDK2
[t10] rmw implementation: rmw_cyclonedds_cpp | domain 0 | interface
[t10] loaded DDS/rmw libraries:
  … /ros2/install/lib/librmw_cyclonedds_cpp.so
  … /ros2/install/lib/libddscxx.so
  … /ros2/install/lib/libddsc.so.0.10.2          <- exactly one, from the merged prefix
[t10] sdk2 self-loopback rx: 498
[t10] sdk2 rt/lowstate  rx: 0 (informational)     <- 0 because no simulator was running
[t10] ros2 /t10_ping    rx: 498
[t10] PASS
```

(Note the install path: `install/lib/…`, not the
`install/t10_dds_coexistence/lib/…` the README carried — `--merge-install`
flattens it.)

It exercises the load-bearing init order: **rclcpp initialises first, then
SDK2's `ChannelFactory::Init(domain, "")` joins the existing domain with an
EMPTY interface.** Both stacks hard-fail if asked to *create* a domain that
exists — `--sdk2-first` reproduces that failure deliberately, and is worth
running once so the error message is familiar:
`rmw_create_node: failed to create domain, error Precondition Not Met`.

With a live simulator on the same domain it also counts `rt/lowstate` (≈930 Hz)
and asserts exactly one `libddsc.so.0.10.2` in `/proc/self/maps`.

---

## 6. Walking

This is the section that did not exist anywhere. The startup sequence is four
constraints, each of which was learned by losing a run to it, and none of which
is guessable from the topic graph.

### 6.1 The four constraints

1. **`g1_ctrl` starts BEFORE the simulator.** It blocks in
   `wait_for_connection()` until `rt/lowstate` appears, so starting it first puts
   `State_Passive` live at sim t≈0, deterministically ahead of the profile's
   chords. Started *after*, it races them, and losing that race presents as
   "the robot never stood up" with nothing in any log.
2. **Chords must be STAGGERED.** The FSM's transitions are DSL strings in
   `deploy/robots/g1/config/config.yaml` (`LT + up.on_pressed`,
   `RT + A.on_pressed`). The receiving side re-filters the L2/R2 **bit** through
   an `Axis` with `smooth = 0.03` against `threshold = 0.5`, so `LT.pressed`
   needs ~23 ticks to go true, while `up.on_pressed` is true for exactly **one**
   tick — the first. Pressed together the AND is never simultaneously true and
   the FSM silently stays in Passive. Hold the axis half ≥25 ms (use 0.3–0.5 s)
   **before** adding the button. Setting `smooth = 1.0` on the *sender* does not
   help: it is the receiver's filter that lags.
3. **The robot must be held up until the policy is running.** With
   `enable_elastic_band: 1`, `UNITREE_MUJOCO_BAND_LENGTH=0.572` carries the
   33.34 kg robot at its 0.793 m spawn height, and
   `UNITREE_MUJOCO_BAND_RELEASE=<t0>[,<ramp>]` lowers it away over `ramp`
   sim-seconds from `t0`. **Lowering it onto FixStand's PD hold pose topples the
   robot every time** (tilt 0.02 → 0.33 rad within 3 s). Lower it *after*
   `R2+A`, onto an actively balancing policy.
4. **`UNITREE_MUJOCO_BAND_RELEASE=34,6`, not the harness default `24,4`.** The
   default is what the recorded A/B matrix was taken with, and is kept for
   comparability; `34,6` gives the policy 12 s of settled standing first and
   converts a class of bring-up falls into walking.

### 6.2 The profile

`config/walk_profile.txt`, sim-time breakpoints, piecewise-constant hold.
Columns `t lx ly rx buttons`; the fifth column is a comma-separated key list
held to the next breakpoint (`-` or absent for none). Keys:
`L2 R2 L1 R1 A B X Y up down left right start select` plus the LT/RT/LB/RB
aliases. **An unknown key aborts at load** rather than silently doing nothing —
a typo'd key is exactly how a scripted FSM run becomes an unexplained failure.

```
# t      lx     ly     rx     buttons
0.0      0.0    0.0    0.0    -
15.0     0.0    0.0    0.0    L2       <- axis half first
15.5     0.0    0.0    0.0    L2,up    <- then the button edge  -> FixStand
16.0     0.0    0.0    0.0    -
21.0     0.0    0.0    0.0    R2
21.5     0.0    0.0    0.0    R2,A     -> RLBase (Velocity)
22.0     0.0    0.0    0.0    -
40.0     0.0    0.20   0.35   -        <- walk (after the band is down)
```

`ly` → forward, `−lx` → lateral, `−rx` → yaw. Chord keys deliberately bypass
`axis_filter_`: DPCBF gates locomotion commands, not mode changes.

### 6.3 Proving the chords arrive — `fsm_button_probe`

```bash
simulate/build_ros2/fsm_button_probe            # against a live rt/lowstate
```

It runs the FSM's **own** `LowState::update()` decode and its **own** compiled
transition DSL (from `deploy`'s config) at 1 kHz, and exits non-zero if a
configured transition never fires — so it is a gate, not only a debugger.

**NOT RUN this session** — the walking run it gates (§6.4) transitioned
correctly on the first attempt, so there was no failure to point it at. It is
built and present at `simulate/build_ros2/fsm_button_probe`.

Use it the moment a run does not stand up. Topic-level inspection cannot answer
this question: in the failure it was built for, the keys **arrived perfectly**
(601 pressed ticks each, axis peak 0.999999) and all seven transition predicates
fired **zero** times. *Arrival is not satisfaction.*

### 6.4 The walking A/B

One command brings up `g1_ctrl` + simulator + description + sidecar +
perception, in the right order, with the band procedure, once per DPCBF mode:

```bash
export DISPLAY=:77                    # a private Xvfb is fine; see §4.2
export WALK_BAND_RELEASE=34,6         # preferred over the 24,4 default
export WALK_SECS=110 WALK_SETTLE=40   # defaults
export WALK_MODES="oracle estimated"  # default; set to one arm to re-run just it
ros2/src/g1_perception/g1_perception_bringup/test/walk_ab_run.sh /tmp/walk_ab W1 W2 W3 W4
```

Fields `W1..W4` are **live** seeded arenas from `DynamicObstacleManager`
(`walk_scenarios.py`), *not* the S1–S4 fixture bags — a bag replay has the
simulator out of the loop and cannot produce a collision rate at all. W4 is the
Phase-4 90-obstacle field verbatim, seed 42.

Verified this session, one arm, `W1`, `WALK_SECS=70`: **1 min 29 s wall clock.**

```
[info] FSM: Start Passive
[info] FSM: Change state from Passive to FixStand
[info] FSM: Change state from FixStand to Velocity

window_s                 35.22        fell_at_s                None
path_m                   15.139       mean_speed_mps           0.43
pelvis_z_min             0.777        tilt_max_rad             0.044
gt_in_scope_mean         2.75         margin_violation_events  0
clearance_min_m          0.0188       clearance_p50_m          1.0675
tracked_to_gt_p50_mm     74.2         tracked_to_gt_p90_mm     156.5
```

### 6.5 What good looks like, and what the failures look like

**Walking** — against a 0.793 m standing height: pelvis **0.77–0.79 m**,
`tilt_max_rad` **< 0.05**, `fell_at_s` **None**, mean speed **0.4–0.5 m/s**,
`bad_orientation` never firing (it drops the FSM back to Passive past 1 rad).
Standing, twitching, or being dragged is not walking; the probe reports all of
these distinctly.

| Symptom | Almost certainly |
|---|---|
| no `Change state` lines in `*.g1_ctrl.log` | chords not staggered (§6.1-2), or `g1_ctrl` started after the simulator (§6.1-1) — run `fsm_button_probe` |
| `Passive → FixStand` but never `→ Velocity` | the second chord; same two causes |
| falls 2–3 s after the band ramp ends, before the first walking command | band-transfer transient, **a harness failure not a product one** — use `34,6` |
| falls *during* the command window with clearance crossing zero first | a real collision; check the clearance trace's zero crossing against the pitch/height columns |
| walks fine, `0 tracked` late in the run | the robot left the arena — read the trajectory panel (§4.3), not the tracker |

To capture every `Filter()` call for offline analysis (the T1/T6 instrument):

```bash
UNITREE_DPCBF_FILTER_LOG=/tmp/capture.bin   # env on the simulator
python3 src/…/test/phase4_capture_stats.py stats /tmp/capture.bin
python3 src/…/test/phase4_capture_stats.py t6    /tmp/capture.bin   # staleness timeline
```

### 6.6 The staleness drill (T6)

```bash
src/g1_perception/g1_perception_bringup/test/phase4_live_session.sh estimated 90 /tmp/t6 t6
python3 src/g1_perception/g1_perception_bringup/test/phase4_capture_stats.py t6 /tmp/t6/capture_estimated.bin
```

**NOT RUN this session** (the walking and A/B blocks consumed the live-session
budget; the numbers below are the Phase-4 record, not a measurement taken here).

It SIGKILLs the perception container mid-run and restarts it 5 s later. Expected
from the recorded drill: DEGRADE at age **0.300 s**, STOP at **0.600 s**, the
retained obstacle set inflated and **never emptied**, recovery to FRESH ~0.03 s
after the restarted container's first frame.

**Preconditions this script does not create for you:** the shadow run-tree at
`$SHADOW` (default `/tmp/sim_shadow_phase4`) — build it per §3.2 first. It also
hardcodes `DISPLAY=:1`, so it will open the simulator on the operator's desktop
rather than an Xvfb; export nothing and expect a window, or edit the line.

---

## 7. Measurement harnesses

### 7.1 Offline A/B + containment, one command

```bash
src/g1_perception/g1_perception_bringup/test/phase4_ab_run.sh /tmp/ab
```

Per scenario: fixture bag → container replay → gt/tracked/safe/raw JSONL dump →
oracle & estimated binary streams → `ab_eval` (two `DpcbfSafetyFilter` arms at
1 kHz) → metrics, plus the §9.6 containment sweep on the same dumps. It needs
the five fixture bags and `simulate/build_ros2/ab_eval`.

Cost: **3 min 48 s.** It reproduced every recorded §17.3 ratio exactly:

```
s1_static:    perf ratio est/orc=0.9565      s2_cross_05: 1.0000
s2_cross_08:  0.9978                          s3_swarm:    0.8265
s4_occlusion: 0.9456                          WORST: 0.8265 (gate >= 0.95)

F=  0mm pooled=75.772%  s1=100.000%  s2_05=100.000%  s2_08=100.000%
                        s3_swarm=70.895%  s4_occlusion=13.242%
F= 50mm pooled=92.900%  s3_swarm=86.410%  s4_occlusion=94.977%
calibrated fixed_inflation (>= 99.9% pooled AND per-scenario): 379.6 mm
```

Read that last line the way §21 does: 379.6 mm is **not** a value to ship. It
is the pooled 99.9th percentile of a distribution with two different
populations in it — a calibratable steady-state bias (which the shipped
`fixed_inflation = 0.051` covers) and occlusion-coast/merged-arc transients
(which no fixed term can). S3 and S4 are where the second population lives.

This is a **robot pinned at (0,0,0)**: it scores command tracking and
containment, never collision rate. For collision rate use §6.4.

Its by-products are the inputs to §7.3: `/tmp/ab/<scenario>.jsonl` carry
`/sim/gt_obstacles`, `/raw_obstacles` (the KF's actual measurements),
`/tracked_obstacles` and `/obstacles_safe` with per-circle covariance.

### 7.2 Circle-fit bias sweep

```bash
python3 src/g1_perception/g1_perception_bringup/test/circle_fit_sweep.py
```

Publishes analytic LaserScan rays at a known cylinder straight into the **real**
extractor and reads `/raw_obstacles` back, so ground truth is exact and every
millimetre of difference is the fit. **It needs a running extractor** — start
`perception.launch.py` first (`use_sim_time:=false`; the sweep stamps with wall
time).

A 10-row sweep costs **~15 s**. Measured this session:

```
r=0.15 d=2.0 arc=1.00: r_meas=0.1638  e_r= +13.8mm  e_c=  -38.8mm  F_req= -11.1mm
r=0.30 d=2.0 arc=1.00: r_meas=0.3273  e_r= +27.3mm  e_c=  -84.3mm  F_req= +57.0mm
r=0.52 d=3.0 arc=1.00: r_meas=0.5624  e_r= +42.4mm  e_c= -152.7mm  F_req=+110.4mm
r=0.55 d=2.0 arc=1.00: r_meas=0.5796  e_r= +29.6mm  e_c= -183.6mm  F_req=+154.1mm
r=0.55 d=3.0 arc=1.0 : NO DETECTION
r=0.60 d=2.0 arc=1.0 : NO DETECTION
```

**The G2 sensing limit is range-dependent, and 0.55 m is only the 2 m figure.**
The chord — hence the fit — grows with range, so the same prop passes the 0.60 m
gate close in and fails it further out: r = 0.55 m is seen at 2 m (fit 0.5796)
and dropped at 3 m and 4 m, while r = 0.52 m is seen at 2, 3 and 4 m. Use
**r ≤ 0.52 m** for props that must be visible across the working range; the 5B
checklist has been corrected to match. Raw rows:
[`../evidence/runbook/circle_fit_limit_by_range.csv`](../evidence/runbook/circle_fit_limit_by_range.csv).

> **It reports `NO DETECTION` for every row when no extractor is running**,
> which is indistinguishable from a real total drop-out — and briefly looked
> like patch 0009 had broken everything. Check the container is up before
> believing a wall of zeroes.

### 7.3 The two calibrators

Both consume a `phase4_obstacles_dump.py` JSONL, so a robot session is a data
drop rather than a development task.

```bash
python3 src/…/test/phase4_obstacles_dump.py 120 /tmp/s1.jsonl     # while the stack runs
python3 src/…/test/measure_measurement_variance.py /tmp/s1.jsonl  # -> R, in m^2
python3 src/…/test/calibrate_k_sigma.py s1=/tmp/s1.jsonl --fixed 0.051
```

Both run in seconds on an existing dump. Measured on the fresh `s1_static`
replay from §7.1:

```
 target     n    mean x    mean y  sd x mm  sd y mm  sd r mm
      0   300     2.196    -2.203      1.4      0.5      1.2
      1   300     0.787     0.782      0.1      0.1      0.1
      2   300    -1.486     1.502      0.5      2.9      2.5
pooled position variance  1.775e-06 m^2  (1-sigma 1.3 mm)
recommended obstacle_tracker measurement_variance: 1.77e-06    # m^2
Shipped value 1.0 asserts a 1 m 1-sigma measurement, i.e. it is 563431x too large.
```

**Do not ship that recommendation either.** The simulated Mid-360 is an
analytic raycast with no range-noise model, so 1.3 mm is a discretisation
artefact; setting R to it would tell the tracker to believe every measurement
absolutely. Both endpoints are wrong, which is why what ships is machinery and
a tripwire rather than a number.

```
POOLED pairs=1700  sigma p50 = 583.6 mm
*** REFUSING TO CALIBRATE k_sigma ***
    sigma p50 584 mm exceeds the plausibility bound 250 mm …
```

**`calibrate_k_sigma.py` refuses** to emit a number while pooled σ p50
exceeds 0.25 m, printing why instead of an authoritative-looking value. That is
correct and is the point: `measurement_variance` is still the inherited
`1.0 m²`, which asserts a one-metre 1σ measurement, so σ is ~0.58 m and any
`k_sigma` fitted against it silently absorbs the error. Set R from **hardware**
first (5B block 1).

### 7.4 The §17.4 numbers

`phase4_latency_probe.py <secs> <out>` gives per-hop latency and container CPU
(§3.1, §3.2). `phase2_probe.py --duration <secs>` adds rates, drop fractions,
T9, CropBox accounting and `/scan` bin occupancy; `--gt-json` adds the per-target
occupancy table that the Phase-5 rosette comparison (Q-3) is scored against.
`phase4_status_dump.py <secs> <out.jsonl>` records `/dpcbf/status` — the
adapter's mode, staleness state and `GetObstacles` latency histogram.

---

## 8. Hardware path, without hardware

> **Going to the robot?** This section is the *dev-machine* view. The robot
> session has dedicated field documents, and they are the ones to read:
>
> | Document | For |
> |---|---|
> | [`g1_first_day_field_runbook.md`](g1_first_day_field_runbook.md) | preflight, network setup, staged field session and stop conditions |
> | [`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) | what the hardware path actually contains, edge by edge, with every claim labelled verified-from-source / verified-by-test / **not measured** / blocked |
> | [`phase5b_checklists.md`](phase5b_checklists.md) | the block-structured capture plan the stages above expand on; still the authority on props, survey convention and per-block bags |
>
> The Phase-5C toolkit, all non-actuating, all runnable as
> `ros2 run g1_perception_bringup <name>`:
>
> | Tool | Does |
> |---|---|
> | `g1_hw_preflight.sh` | read-only preflight: arch, ROS/DDS environment, executables, installed files, **source-vs-installed drift**, Mid-360 config, placeholder-IP hard stop, routes, the extrinsic guard, disk. Exit 0/1/2 |
> | `hw_source_probe.py` | `/livox/{lidar,imu}` only: rate, gaps, frame_id, point layout, **stamp monotonicity**, **host-vs-device clock**, geometry, QoS match. JSON + text |
> | `hw_tf_probe.py` | TF availability **at each LiDAR message stamp** (never latest) for the four §8.2 pairs, with extrapolation classification |
> | `hw_diagnostics.py` | one `/diagnostics` array at 1 Hz covering the whole chain; ERROR on no-data rather than silence |
> | `hw_session_metadata.py` | provenance next to a bag; reports the operator fields still blank |
> | `hw_record.sh` | bag + environment dump + checksums + metadata in one command |
> | `config_diff.py` | which copy of every launch/config/rviz file is actually live |

### 8.1 Preflight

```bash
python3 src/g1_perception/g1_perception_bringup/test/hw_config_check.py
```

```
local IPv4 addresses: ['100.98.37.113', '127.0.0.1', '127.0.1.1', '192.168.0.65']
FAIL: host_net_info IP 192.168.1.5 is not assigned to any local interface — the SDK will bind nothing

PLACEHOLDER CONFIG: the IPs are upstream sample values, not this robot (Q-1).
Fill them in from the LiDAR serial sticker and the onboard PC network before 5B block 1.
```

Exit **2**. On the dev machine that is the *correct* answer and is why the CTest
skips; on the robot it is a hard fail. It also checks that ports are free, that
LiDAR and host are on the same /24, that `xfer_format` is 0 (the only
PointCloud2 format the ROS 2 build implements) and that `extrinsic_parameter` is
identity — H-1 lives in the xacro only, and setting it here too would apply the
extrinsic twice.

### 8.2 The bench stub

```bash
ros2 launch g1_perception_bringup source_hw.launch.py driver:=off lio:=dlio
python3 src/g1_perception/g1_perception_bringup/test/hw_source_stub.py <bag>
```

`hw_source_stub.py` is a **driver output emulator**, not a LiDAR simulator: it
replays a sim fixture's geometry re-wrapped in the driver's exact wire format
(7 fields, `point_step` 26 under `#pragma pack(1)`, RELIABLE depth 256,
wall-clock stamps) plus a synthetic IMU. That is what makes the cloud contract,
the QoS wire-check and the TF tree shape testable on a bench. It is exercised by
two launch tests in the suite (§5.1) — running it by hand is for when you want
to watch.

**NOT RUN by hand this session**; both launch tests that drive it passed inside
the suite, so the stub works, but the two-command invocation above is
transcribed from them rather than executed standalone.

### 8.3 The driver with no device

```bash
ros2 launch g1_perception_bringup source_hw.launch.py lio:=off
```

Verified this session on the dev machine (no LiDAR attached):

```
[livox_ros_driver2_node-1] bind failed
[livox_ros_driver2_node-1] Failed to init livox lidar sdk.
[livox_ros_driver2_node-1] [ERROR] [livox_lidar_publisher]: Init lds lidar fail!
$ ros2 topic list | grep -i livox
(no /livox/* topic)
```

Note carefully what it does **not** do: it does not exit, it creates **no
`/livox/*` topic at all** (the publisher is lazy), and it **ignores SIGINT and
SIGTERM** — after SIGINT to `ros2 launch` the node was still running and had to
be `pkill -9 -f livox`ed. On robot day, "no `/livox/*` topic after 15 s" is the
diagnostic for an SDK bind failure, not for a dead LiDAR.

### 8.4 Building for the robot

`tools/build_target.sh` is the on-target build (Orin, Pi, container). `--check`
runs the preflight only. It encodes five environment faults with the error each
prevents and ends with a triage list keyed by error message. Expect **10 of 18
packages** on a fresh aarch64 environment: an unsolved upstream `ament_cmake`
cache-shadowing bug stops the rest, and
`tools/diagnose_ament_export_libraries.py` exists to recognise it —
**diagnostic only, no workaround has been shown correct.** Read both before the
session, not during it.

### 8.5 The perception-only hardware launch

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true record:=on
```

One entry point for the whole hardware chain — driver, DLIO, robot TF,
`base_footprint`, the perception container, diagnostics, optional RViz and an
optional metadata-stamped bag — **and nothing else**. It contains no `mode`
argument, no `ground_seg` argument and no `use_sim_time` argument, because
none of the three has a meaning in a session with no DPCBF and no `/clock`.

`test_hw_offline_gates.py` (CTest `hw_offline_gates`) walks the launch closure
by AST on every build and asserts that it constructs no node from `g1_ctrl`,
`deploy`, `unitree_*` or `dpcbf_ros_adapter`, and no process whose command
mentions a command topic. `dpcbf_ros_adapter` is a **static library, not a
node**, so there is no DPCBF seam to construct; `/obstacles_safe` is published
and consumed by nobody.

On the dev machine `driver:=off lio:=off` brings up the container half alone,
which is how the launch itself is exercised without a device.

**`ground_seg:=patchwork` now raises.** It was a silent no-op — nothing ever
read the argument, Patchwork++ is not imported/built/launched, and
`/points_no_ground` does not exist. `bringup.launch.py` keeps the argument
solely so an old command line gets an explanation instead of a shrug; the
hardware launch does not have it at all.

---

## 9. Regeneration

### 9.1 Fixture bags

**Any change to `obstacle_detector/msg/Obstacles` invalidates every bag that
carries it** — all six do, via `/sim/gt_obstacles`. Patch 0007 did exactly that
and T5 went red with *"no GT crosser samples in bag replay"*: a deserialisation
failure, not a product regression. Expect this again if the message changes.

Recipe, hashes and per-fixture parameters live in
[`../test_fixtures/README.md`](../test_fixtures/README.md); the short version is
`make_scenario_scene.py` to build the mirror + scenario JSONs, then per scenario:
record the bag **before** starting `scenario_state_source.py` so t=0 is in-bag,
with SIGINT + `pkill` discipline between runs (§3.4). Record the new md5 in that
README — the last regeneration recorded two of five, and the missing three had
to be recovered later.

`s1_static_reference` is the odd one out: it is not a scripted scenario but a
live capture of the 90-obstacle field with the robot suspended, so it needs the
§3.2 stack **minus the perception container** (the container is what replays it
later) and a plain `ros2 bag record`. Regenerated this session:

```
Duration: 28.885 s   Bag size: 98.1 MiB   /livox/lidar: 289
md5(db3): 3466e2cf54b1f3374497796c33fbcbbb
```

**Match the wall clock to the realtime factor, not to the target duration.**
With the container also running the sim runs at ≈0.44 RT and 30 s of wall clock
bought 13.1 s of bag; without it the factor is ≈1.0 and 29 s of wall clock gave
28.9 s. Verify afterwards by running the consumer:

```
$ launch_test src/…/test/test_projection_replay.launch_test.py
/scan: 289 frames over 28.8 s sim time -> 10.00 Hz, drop 0.0%
T9 (replay): 288 clouds, 0 misses          OK   (36.8 s)
```

### 9.2 Evidence images

| Image | Command | Cost |
|---|---|---|
| `evidence/runbook/rviz_bag_replay.png` | §4.2 | ~45 s |
| `evidence/walking/walk_overlay.png` | `walk_overlay_run.sh <out> W3 110` | ~2 min |
| `evidence/runbook/walk_overlay_reproduced.png` | `walk_overlay_run.sh <out> W1 75` | 1 min 32 s |

### 9.3 The mirror model

`/tmp/unitree_mujoco_mirror_model.xml` is re-dumped by `simulate` on every model
load; there is nothing to regenerate by hand. Override the path with
`UNITREE_MUJOCO_MIRROR_XML` when running two scenes at once. If the sidecar
reports a shape mismatch, you are pointing it at a **stale** dump from a
different scene — delete it and restart the simulator.

---

## 10. Nothing is arriving — diagnostic order

Work down this list. Each step is cheap and each eliminates a whole class.

1. **Is anything published at all?** `ros2 topic list | sort`. Nothing ⇒ the
   environment block (§2.1). Topic names but no data ⇒ go to 2.
2. **`CYCLONEDDS_URI` mismatch** (live-sim sessions only). The single most
   common cause of "connected but silent" on this machine. §2.1.
3. **A stale container.** `pgrep -af 'component_containe[r]'` — more than one, or
   one you did not start, means the new launch loaded nothing. §3.4.
4. **A stray simulator.** `pgrep -af 'unitree_mujoc[o]'`. Doubled `/clock` is the
   tell.
5. **Lazy subscription.** No `/scan` subscriber ⇒ `pointcloud_to_laserscan` never
   subscribes to `cloud_in` and the whole downstream chain is idle. Attach a
   probe or RViz. §3.1.
6. **QoS mismatch.** `ros2 topic info -v <topic>` and compare reliability. Every
   subscriber in this stack is best-effort by design; a RELIABLE-only publisher
   (upstream DLIO, the hardware driver) matches nothing until patched.
7. **TF.** `ros2 run tf2_ros tf2_echo odom base_footprint`. If it does not
   resolve, `pointcloud_to_laserscan` produces nothing and the failure *looks
   like* a broken cloud path.
   > A `static_transform_publisher` is **not** a usable stand-in for odometry:
   > `base_footprint_publisher` deduplicates by stamp, so a static
   > `odom→base_link` makes it emit `base_footprint` exactly once at t=0 and
   > every later lookup extrapolates into the future. Use real odometry — the
   > sidecar, DLIO, or `hw_source_stub.py`'s `odom_hz` mode.
8. **Sim time.** A node left at `use_sim_time:=true` with no `/clock` never
   advances its clock and never fires a timer. This is DLIO's upstream default
   and is overridden in our config.
9. **The seam.** If perception is fine but DPCBF does nothing: with
   `use_joystick: 0` and no `UNITREE_MUJOCO_SCRIPTED_COMMANDS`, the stock bridge
   wires **no** joystick, `lowstate->joystick` stays null, and the 1 kHz
   `axis_filter`/`Filter()` seam never runs at all. It was dead through Phases
   0–3 and no topic-level probe could see it. Confirm with
   `UNITREE_DPCBF_FILTER_LOG` (§6.5).

---

## 11. Reference

### 11.1 Topics you will actually look at

| Topic | Rate | Frame | Produced by |
|---|---|---|---|
| `/livox/lidar` | 10 Hz | `mid360_link` | sidecar (sim) / `livox_ros_driver2` (hw) |
| `/points_self_filtered` | 10 Hz | `mid360_link` | CropBox |
| `/scan` | 10 Hz | `base_footprint` | `pointcloud_to_laserscan` |
| `/raw_obstacles` | 10 Hz | `odom` | `obstacle_extractor` |
| `/tracked_obstacles` | 10 Hz | `odom` | `obstacle_tracker` |
| `/obstacles_safe` | 10 Hz | `odom` | `safety_obstacle_filter` |
| `/sim/gt_obstacles` | 50 Hz | `odom` | `simulate` (sim only) |
| `/sim/mj_state` | 100 Hz | — | `simulate` (sim only) |
| `/odom` | 100 Hz sim / 100 Hz hw | `odom`→`base_link` | sidecar / DLIO |
| TF `odom→base_link` | 100 Hz sim / **~10 Hz hw** | — | sidecar / DLIO (per-scan thread) |
| `/dpcbf/status` | 10 Hz | — | the adapter (mode, staleness, latency histogram) |
| `/clock` | ≥100 Hz | — | `simulate` (sim only) |

`/dpcbf_overlay/markers` — the §4.6 overlay, one MarkerArray with namespaced
groups (`gt`, `tracked`, `safe`, `error`, `unpaired_*`, `selected`,
`ecbf_barrier`, `pmax`, `status`, `vel_*`). Only published when
`viz.launch.py` runs with `overlay:=on`.

### 11.2 Environment variables

| Variable | Where | Effect |
|---|---|---|
| `UNITREE_DPCBF_MODE` | simulate | `oracle` (default, D5) / `shadow` / `estimated` |
| `UNITREE_DPCBF_FILTER_LOG` | simulate | capture every `Filter()` call to a binary log |
| `UNITREE_MUJOCO_SCRIPTED_COMMANDS` | simulate | profile path; **without it the 1 kHz seam never runs** on this machine |
| `UNITREE_MUJOCO_BAND_LENGTH` | simulate | elastic-band rest length (0.572 holds the robot at spawn height) |
| `UNITREE_MUJOCO_BAND_RELEASE` | simulate | `<t0>[,<ramp>]` sim-time band lowering; inert when unset |
| `UNITREE_MUJOCO_MIRROR_XML` | simulate + sidecar | mirror-model dump path |
| `PERCEPTION_TEST_DOMAIN` | launch tests | force one test onto a chosen DDS domain (0 = live sim) |
| `WALK_SECS` / `WALK_SETTLE` / `WALK_MODES` / `WALK_BAND_RELEASE` / `WALK_TREE` | `walk_ab_run.sh` | run length, settle window, which arms, band schedule, scratch tree |
| `T4_BAG` / `T4_LAYOUT` / `T4_USE_SIM_TIME` | `test_detection_static` | point the T4 harness at a hardware session |
| `SHADOW` | `phase4_live_session.sh` | shadow run-tree location |

### 11.3 Trap index

Each trap is documented **where it bites**; this is only the index.

| Trap | Section |
|---|---|
| conda python 3.12 shadowing 3.10 | [2.1](#21-the-environment-block) |
| `CYCLONEDDS_URI` mismatch ⇒ names but no data | [2.1](#21-the-environment-block), [10](#10-nothing-is-arriving--diagnostic-order) |
| `colcon test` without `--merge-install`, and the stale green summary after it | [5.1](#51-the-whole-suite) |
| `colcon test-result --all` counting external lint failures | [5.1](#51-the-whole-suite) |
| config edits need a rebuild (configs are installed, not sourced) | [2.3](#23-incremental-builds) |
| `pkill -f component_container` killing its own shell | [3.4](#34-teardown--read-this-before-your-second-run) |
| path-qualified `pkill` missing `./unitree_mujoco` | [3.4](#34-teardown--read-this-before-your-second-run) |
| SIGTERM orphaning a container ⇒ next launch loads nothing | [3.4](#34-teardown--read-this-before-your-second-run) |
| `pointcloud_to_laserscan`'s lazy subscription | [3.1](#31-bag-replay--the-cheapest-full-pipeline) |
| sidecar must load the dumped mirror, not the raw scene | [3.2](#32-live-sim-stack) |
| `static_transform_publisher` unusable as odometry | [10](#10-nothing-is-arriving--diagnostic-order) |
| the driver ignoring SIGINT/SIGTERM with no device | [8.3](#83-the-driver-with-no-device) |
| `Obstacles` message changes invalidating every fixture bag | [9.1](#91-fixture-bags) |
| unstaggered FSM chords (arrival ≠ satisfaction) | [6.1](#61-the-four-constraints), [6.3](#63-proving-the-chords-arrive--fsm_button_probe) |
| lowering the band onto FixStand instead of the policy | [6.1](#61-the-four-constraints) |
| the 1 kHz seam silently not running | [10](#10-nothing-is-arriving--diagnostic-order) |
| `circle_fit_sweep.py`'s `NO DETECTION` wall with no extractor | [7.2](#72-circle-fit-bias-sweep) |
| launch files are installed too — an un-rebuilt `viz.launch.py` silently starts no overlay | [4.6](#46-the-live-2-d-overlay--estimated-vs-gt-and-the-dpcbf-constraint) |
| reading the DPCBF "parabola" as workspace geometry — it is m/s, per obstacle | [4.6](#46-the-live-2-d-overlay--estimated-vs-gt-and-the-dpcbf-constraint), [4.4](#44-the-opencv-dpcbf_visualizer) |
| `selected` ≠ nearest — the shipped `obstacle_priority: 1` orders by closing alignment | [4.6](#46-the-live-2-d-overlay--estimated-vs-gt-and-the-dpcbf-constraint) |
| quoting a `boundary_check join` from a run that fell (check `fell_at_s` first) | [4.6](#46-the-live-2-d-overlay--estimated-vs-gt-and-the-dpcbf-constraint) |
| a `-f`-style process match catching the measuring process itself (`proc_cpu.py`) | [4.6](#46-the-live-2-d-overlay--estimated-vs-gt-and-the-dpcbf-constraint), [3.4](#34-teardown--read-this-before-your-second-run) |
| `set -u` + `source /opt/ros/humble/setup.bash` aborting on `AMENT_TRACE_SETUP_FILES` | — every script here starts `set +u` |
| `launch_test` not putting the test's own directory on `sys.path` | — every test does `sys.path.insert` before its sibling import |
