# Phase 5B — robot-session checklists

**Rule for the session: capture, do not debug.** Every number below is
re-derivable offline from a bag. If a block does not produce its bag, the
block failed — move on and write it down; do not troubleshoot on robot time.
Blocks are ordered by dependency; a failed **abort criterion** ends that block,
not necessarily the session.

Robot is **stationary or externally supported throughout**. No DPCBF in the
loop, no walking, no deploy integration (all Phase 6). The operator owns
physical safety and is the only one who moves the robot.

Prerequisites for the whole session: the 5A gate is green, Q-1 is answered
and recorded, and `hw_config_check.py` exits 0 (not 2).

---

## Block 0 — before the robot is powered (no robot time)

| Item | Why |
|---|---|
| **Q-1 answers written down verbatim** — G1 variant; onboard PC (EDU Orin NX or other); is the Mid360 the factory head mount (H-1)?; does the onboard PC have sudo/apt and internet?; Ubuntu/ROS as shipped; how the dev machine reaches it (link, IPs) | §12 was written from vendor docs; nothing in it is verified |
| **Topology decision recorded**: perception on the Orin, or driver-on-robot + perception-on-dev-machine over ethernet | Decides whether the CPU-budget gate means anything this session (§17.4). If perception runs off-robot, the Orin benchmark becomes a **named follow-up**, not a silently skipped gate |
| Workspace built on whatever will run it — run **`ros2/tools/build_target.sh --check`** first, then without `--check` | The interim block built the workspace on emulated aarch64 and got **10 of 18 packages**; the script encodes every failure it hit and ends with a triage list keyed by error message. Assume the build is NOT a formality: allow a slot for it before the robot is powered, and note that installing the apt dependencies needs root on the target |
| Read **`ros2/tools/diagnose_ament_export_libraries.py`** before the session | A from-scratch build can die claiming a package cannot find its OWN library. It is an upstream ament bug, package-set dependent, and **unsolved** — the file explains how to recognise it and what to try (sequential executor; then that package alone with `--packages-select`). Budget for it; do not discover it on robot time |
| `CYCLONEDDS_URI` prepared naming the **robot's** NIC, not `lo` (§12.2) | Phase-2 trap: wrong interface ⇒ topic names visible, no data |
| Props: **≥3 cylinders, at least one r ≥ 0.30 m; keep every prop at r ≤ 0.52 m** | The interim sweep pinned the bias: centre error `−0.278·r`, radius error `+0.084·r`, both range-independent (evidence/interim/circle_fit_bias.md). It also found a hard cut, whose mechanism is now confirmed (gap G2): the extractor tested `fit + radius_enlargement(0.17) < max_circle_radius(0.60)`, i.e. a 0.43 m fit, i.e. **r < 0.397 m of true radius** — silently, and intermittently, because occlusion shortens the chord and brings the same prop back under the cut. **Fork patch 0009 fixes this**: the gate is on the fit, so the limit is now `0.60/1.084 ≈ 0.55 m` of true radius, and every drop is counted and throttle-logged by both the extractor and `safety_obstacle_filter`. **The r ≈ 0.32 m upper bound in the previous revision of this checklist is WITHDRAWN.** **REFINED 2026-08-02 (runbook block, `evidence/runbook/circle_fit_limit_by_range.csv`): the limit is RANGE-DEPENDENT and 0.55 m is only true at 2 m.** Re-measured on the same harness at full arc: r = 0.55 m detects at d = 2 m (fit 0.5796, just under the 0.60 gate) but is **dropped at d = 3 m and d = 4 m**; r = 0.52 m detects at 2, 3 and 4 m (fit 0.5624 / 0.5699). The chord — hence the fit — grows with range, so the same prop passes the gate close in and fails it further out, which is the same intermittency G2 described, in the range axis rather than the occlusion axis. Props are surveyed at 1–3 m, so **use r ≤ 0.52 m** and treat 0.55 m as the close-range-only figure. Keep `max_circle_radius` equal in `obstacle_detector.yaml` and `safety_obstacle_filter.yaml`; if a prop above that is ever wanted, raise both and say so in the session log |
| Tape measure, chalk/tape for floor marks, a flat wall segment ≥2 m wide | Blocks 2 and 5 are surveys |
| Disk: ≥20 GB free where bags are written | Raw 10 Hz clouds ≈ 3 MB/s (sim bags: 93 MB / 28 s) |
| `df -h`, `date`, `git rev-parse HEAD` captured into the session log | Bag provenance |

**Session bag root:** `test_fixtures/hw/<YYYY-MM-DD>/`. Every bag below is
`md5sum`'d and listed in `test_fixtures/README.md` at the end of the session.

---

## Block 1 — driver up

**Preconditions:** robot powered, standing or supported, nothing else using the
LiDAR network. Perception stack NOT running.

```bash
# 1. preflight (must exit 0; exit 2 means the config still has placeholder IPs)
ros2 run g1_perception_bringup hw_config_check.py    # or: python3 test/hw_config_check.py

# 2. driver alone
ros2 launch g1_perception_bringup source_hw.launch.py lio:=off

# 3. in a second shell — RECORD FIRST, look second
ros2 bag record -o test_fixtures/hw/<date>/b1_raw_driver \
    /livox/lidar /livox/imu

# 4. observables
ros2 topic hz /livox/lidar          # expect ~10 Hz
ros2 topic hz /livox/imu            # expect ~200 Hz (device fact, unverified)
ros2 topic echo --once /livox/lidar --field header
ros2 topic echo --once /livox/imu   --field header
```

**Expected observables**

- `/livox/lidar` at 10 ± 0.5 Hz; `frame_id` **`mid360_link`**; 7 fields
  (`x,y,z,intensity,tag,line,timestamp`), `point_step` 26.
- `/livox/imu` present, `frame_id` **`mid360_link`** (patch 0005 — if it says
  `livox_frame`, the patch did not get applied to the build in use).
- Timestamp mode: compare a cloud stamp to the host clock.
  `python3 -c "import time;print(time.time())"` next to
  `ros2 topic echo --once /livox/lidar --field header.stamp`. **Offset within a
  few ms ⇒ host-clock fallback** (`kTimestampTypeNoSync`); a large or drifting
  offset ⇒ the LiDAR is sync'd to its own/PTP clock. **Record which**; it
  decides whether DLIO's stamps and the driver's share a base (§14.3).

**Abort criteria**

- No `/livox/*` topic at all after 15 s ⇒ SDK init/bind failure. The driver
  **does not exit** on this — it logs `bind failed` / `Init lds lidar fail!`
  and keeps running, ignoring SIGINT and SIGTERM. `pkill -9 -f livox`, fix the
  IPs in `MID360_config.json`, rerun `hw_config_check.py`. Two attempts, then
  abort the block.
- Topic present but 0 Hz ⇒ different failure (device state / firmware); record
  the driver's stdout and move on.

**Bag:** `b1_raw_driver` — the first artefact of the session and the input to
Q-3 and Q-8 if nothing else works.

**Also derived from this bag, offline — the measurement variance for P-3.**
`measurement_variance` is `R(0,0)` and its unit is **m²** — the tracker's KF
has `C = [1 0]` and its measurement is a circle centre in metres, so σ was
always metres. What is wrong is the VALUE: the inherited `1.0` asserts a
**one-metre 1σ** measurement, P-2 seeds `P(0,0) = R` from it, and that is why
sim σ reads 583 mm and why `k_σ` silently absorbs the scale (gap G1; the
earlier "σ is not in metres" reading is corrected in the §21 entry). A
variance of 1.0 m² is a perfectly well-typed number, so nothing downstream
could ever have noticed — which is why there is now a tripwire in
`calibrate_k_sigma.py` (it REFUSES to emit a k_σ when σ p50 > 0.25 m) and a
throttled warning in `safety_obstacle_filter`.

Two ways to get the real number, in preference order:

1. **From the detector's own scatter (preferred — it is what R actually
   models).** `phase4_obstacles_dump.py` now records `/raw_obstacles`, the
   KF's actual measurements. On a static surveyed scene:
   ```bash
   /usr/bin/python3 test/measure_measurement_variance.py hw_b5.jsonl
   ```
   It clusters detections per physical target and reports the scatter about
   each target's mean, so the systematic circle-fit bias (which
   `fixed_inflation` already covers, and which must NOT be double-counted
   here) is absorbed by the mean.
2. **From raw range noise**, as a cross-check: point the LiDAR at the flat
   wall from block 2 and take the per-return residual to the fitted plane at
   1, 2 and 3 m. Expect this to be SMALLER than (1): a circle centre is a fit
   over many returns, but it also inherits projection and odometry jitter.

No extra robot time — both reuse `b1_raw_driver`, `b2_wall` and block 5's
capture. Set `measurement_variance` from (1) **before** anyone calibrates
`k_σ`, and re-run the S1/S2 containment check afterwards: R is not a display
parameter, it changes how much the tracker trusts each measurement and
therefore changes the tracked positions the whole safety chain consumes.

---

## Block 2 — extrinsic verification (H-1, Q-1)

**Preconditions:** block 1 green. Robot **standing static** on a level floor,
squared to a flat wall. Survey the wall: measure the horizontal distance from
the robot's `base_footprint` origin (the point on the floor under the pelvis)
to the wall face, and the wall's bearing. Mark the robot's foot positions with
tape so the pose is reproducible.

```bash
ros2 launch g1_perception_bringup bringup.launch.py source:=hw lio:=dlio
ros2 bag record -o test_fixtures/hw/<date>/b2_wall \
    /livox/lidar /livox/imu /odom /tf /tf_static /scan
```

**Expected observables**

- `/scan` ranges toward the surveyed wall within **±2 cm** of the tape measure,
  the same bar Phase 2 met in sim (measured −0.04/−0.07/−0.15 mm at 1/2/4 m).
- A floor-plane fit on `/livox/lidar` in `mid360_link` must show the
  **upside-down mount (H-2)**: the ground returns sit at positive sensor-frame
  z, and the plane normal is ≈ +z, not −z. If the plane comes out the other
  way, the mount is not H-1 and every downstream number is suspect.
- `ros2 run tf2_ros tf2_echo torso_link mid360_link` reproduces the xacro
  values (this only checks the published TF, not the physical mount).

**If the mount ≠ H-1:** re-measure it physically, update
`g1_description/urdf/g1_mid360.xacro` — the single source of truth — and let
T7 + T7-hw re-derive everything else (the MJCF side is a sim-fidelity
follow-up, not a 5B task). **Do not** patch the driver JSON's
`extrinsic_parameter` instead; that applies the extrinsic twice.

**Abort criteria:** wall error > 10 cm, or the floor plane is not flat
(RMS > 3 cm) ⇒ stop and re-survey; a bad survey invalidates block 5 too.

**Bag:** `b2_wall`.

---

## Block 3 — DLIO stationary (§18 gate)

**Preconditions:** block 2 green; robot **motionless** for the full window,
including the first 3 s (DLIO calibrates IMU bias over
`odom/imu/calibration/time: 3.0` and a moving robot poisons it).

```bash
ros2 launch g1_perception_bringup bringup.launch.py source:=hw lio:=dlio
ros2 bag record -o test_fixtures/hw/<date>/b3_dlio_static \
    /livox/lidar /livox/imu /odom /tf /tf_static      # >= 10 minutes
```

**Expected observables**

- `/odom` ~100 Hz; TF `odom→base_link` at **scan rate (~10 Hz)** — that is
  DLIO's design, not a fault (see the 5A seam audit).
- **GATE: drift < 1 cm/min** over ≥10 min, measured offline as
  ‖p(t) − p(0)‖ / t from the recorded `/odom`.
- First ~30 clouds dropped by `pointcloud_to_laserscan` with *"timestamp …
  earlier than all the data in the transform cache"* — expected startup
  transient before DLIO's first TF.

**Then, if the operator can carry/support the robot:** a short
externally-moved sequence (walk it around a loop back to the start) for
qualitative drift. Bag `b3_dlio_carried`. This is a measurement, not a gate.

**Q-4 bake-off (OPTIONAL, only if session time allows):** FAST-LIO needs
`xfer_format: 1` (CustomMsg), which the driver publishes **instead of**
PointCloud2 — so it is a *separate capture session* during which the
perception stack has no cloud. Budget ~20 min and treat it as a distinct
block, or skip and record it as carried.

**Abort criteria:** DLIO diverges (position jumps > 1 m while stationary) ⇒
record 2 min anyway for offline diagnosis, then stop the block.

---

## Block 4 — self-hit capture (CropBox retune data)

**Preconditions:** blocks 1–3 done. **Perception stack NOT running** — the
point is unfiltered clouds. Clear a ≥1.5 m radius around the robot so that
*everything* within 0.8 m of the sensor is the robot itself.

```bash
# driver only, no perception, no CropBox
ros2 launch g1_perception_bringup source_hw.launch.py lio:=off
ros2 bag record -o test_fixtures/hw/<date>/b4_selfhit /livox/lidar
```

Three sequences, ≥30 s each, announced into the log as they happen:

1. **static** — arms at the default pose, robot still;
2. **arm swing** — operator moves each arm slowly through its full range
   (this is the case H-4's head-shell mask never covered and the case the
   Phase-2/3 sim wrist finding predicts);
3. **torso motion** — waist yaw/roll/pitch through their range.

**Expected observables:** near-range returns clustered where the arms and
torso are; nothing else inside 0.8 m.

**Offline (after the session, not during):**

```bash
python3 test/selfhit_analysis.py test_fixtures/hw/<date>/b4_selfhit --radius 0.8 \
        --json evidence/phase5b/selfhit.json
```

It reports the p50/p99/p99.9/max envelope in the sensor frame and a proposed
box. **The cross-check it prints is the real decision:** if the 99.9th
percentile |x| or |y| approaches `range_min` (0.30 m), the arms are reaching
into the detection band and the answer is a shaped or pose-aware mask, not a
bigger box. The retune replaces the Phase-3 **sim-interim** values
(xy ±0.40, `max_z` 0.45) in `config/cropbox_self_filter.yaml`, with the
measurement recorded in the log (Appendix-A discipline).

**Abort criteria:** none — this block only records.

---

## Block 5 — T4-hardware (the phase's headline gate)

**Preconditions:** blocks 1–3 green. Robot static in the taped pose from
block 2. Props surveyed with the tape measure into a layout file; **at least
one prop with r ≥ 0.30 m**.

Survey convention (matches the sim fixtures): measure to the **prop face**,
then the centre is face-distance + r along the bearing. Record positions in
the `odom` frame — with the robot static at DLIO's origin, `odom` ≈
`base_footprint` rotated by the robot's yaw, so mark the robot's forward
direction on the floor and measure bearings from it.

`test_fixtures/hw/<date>/t4_layout.yaml`:

```yaml
match_radius: 0.5          # §17.2
targets:
  - {name: cyl_1m,     x:  0.813, y:  0.813, r: 0.15}
  - {name: cyl_2m,     x: -1.520, y:  1.520, r: 0.15}
  - {name: blocker_3m, x:  2.333, y: -2.333, r: 0.30}   # the r>=0.30 prop
```

```bash
ros2 launch g1_perception_bringup bringup.launch.py source:=hw lio:=dlio
ros2 bag record -o test_fixtures/hw/<date>/b5_t4 \
    /livox/lidar /livox/imu /odom /tf /tf_static /scan \
    /raw_obstacles /tracked_obstacles /obstacles_safe      # >= 60 s
```

Repeat with props at 1 / 2 / 3 m (or place all three at once, as the sim S1
fixture does — one bag, three distances).

**Offline — the same harness as sim T4, pointed at the hardware bag (D4):**

```bash
T4_BAG=test_fixtures/hw/<date>/b5_t4 \
T4_LAYOUT=test_fixtures/hw/<date>/t4_layout.yaml \
T4_USE_SIM_TIME=false \
  launch_test test/test_detection_static.launch_test.py
```

**Gates (§18):** centre error ≤ 0.10 m, `true_radius` error ≤ 0.05 m
(pre-inflation, on `/tracked_obstacles`), detection latency ≤ 2 frames.

**Also measured, not gated:**

- **The r ≥ 0.30 bias question.** The harness prints per-target centre and
  radius error with the target's `r_gt`, so the Phase-4 finding ("bias grows
  with radius": +33 mm radius / 83 mm centre at r=0.30 in sim) either
  reproduces on real data or it does not. **Write down which** — the whole
  `fixed_inflation = 0.051` calibration rests on it.
- **Per-track residuals AND covariances for P-3's k_σ.** P-3 is now
  *implemented* (fork patch 0007: `CircleObstacle.covariance`), so this is a
  data drop, not a development task. `phase4_obstacles_dump.py` records `cov`
  alongside the residual, and `calibrate_k_sigma.py` consumes the result
  unchanged from sim:

  ```bash
  /usr/bin/python3 test/phase4_obstacles_dump.py 120 hw_b5.jsonl   # during replay
  /usr/bin/python3 test/calibrate_k_sigma.py b5=hw_b5.jsonl --fixed 0.051
  ```

  **Set `measurement_variance` from block 1's measurement first** — otherwise
  σ is the uncertainty of a tracker that has been told its LiDAR measures
  centres to ±1 m, and the k_σ that comes out is a scale factor for the wrong
  quantity (gap G1). `calibrate_k_sigma.py` now refuses to emit one in that
  case rather than printing an authoritative-looking number. Also report
  `corr(F_req, σ)`: sim gives +0.339, weak. If hardware does not improve on
  that, the honest verdict is that the σ term is the wrong remedy and the
  occlusion residual needs the detector-level arc-truncation fix instead.
- **Arc-truncation check.** Place two props so one partly occludes the other
  from the robot. The sim model says required inflation scales as
  `F_req ≈ k(arc)·r` with k rising 0.22 → 0.96 as visibility falls 100 % → 50 %
  (evidence/interim/circle_fit_bias.md). One 60 s bag tests the whole model.

**Abort criteria:** no circles detected at all at 1 m ⇒ check `/scan` first
(block 2 territory); if `/scan` is fine, record 60 s of `/scan` +
`/raw_obstacles` for offline extractor tuning and end the block.

---

## Block 6 — Q-3 / Q-8 captures

**Q-3 (rosette density parity).** Place the props at the *same* distances the
Phase-2 baseline used (wall at 1/2/4 m, cylinders r=0.15 at 1/2/3 m) and
record 30 s.

```bash
ros2 bag record -o test_fixtures/hw/<date>/b6_q3 /livox/lidar /scan /tf /tf_static
```

Offline, compare against the Phase-2 sim table (occupied-bin fraction /
points-per-occupied-bin): wall_1m 1.0 / 20.37, wall_2m 1.0 / 13.77,
wall_4m 0.9977 / 8.72, cyl_1m 1.0 / 20.56, cyl_2m 1.0 / 14.04,
cyl_3m 1.0 / 10.17. `test/phase2_probe.py` computes the same statistics.

**Q-8 (head-aperture ground truth).** Near-field scan: nothing within 2 m
except the robot; record 30 s of raw cloud and look at which bearings/
elevations are *missing* — that is the head shell's occlusion footprint,
which the sim approximates with `group="2"` masking.

```bash
ros2 bag record -o test_fixtures/hw/<date>/b6_q8 /livox/lidar
```

**Abort criteria:** none; both are captures.

---

## Block 7 — Q-5, opportunistic (costs ~2 minutes)

While anything else is running, add Unitree's own odometry topics to a
recording so DLIO can be compared against them offline. With
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the SDK2 domain, they appear as
ROS2 topics (§12.2).

```bash
ros2 topic list | grep -i odom          # find the actual name on this robot
ros2 bag record -o test_fixtures/hw/<date>/b7_q5 /odom /rt/odommodestate /tf
```

Note: `ros2 bag record` may fail on Unitree message types the CLI cannot
resolve (the Phase-4 log records `ros2 topic hz` failing on `unitree_hg`
types for exactly this reason). If it does, record `/odom` alone and note
that Q-5 needs an SDK2-linked recorder — a follow-up, not a session problem.

---

## Block 8 — CPU / latency benchmark

**Only meaningful on whatever actually ran the perception container.** If the
topology decision in block 0 put perception on the dev machine, say so
explicitly in the log and carry "Orin benchmark" as a named follow-up — do not
report a dev-machine number against the §17.4 Orin budget.

```bash
# with the full stack running on the target, 60 s:
python3 test/phase4_latency_probe.py 60      # cloud -> /obstacles_safe p50/p95/p99
top -b -n 1 | head -20                       # or cgroup stats for the container
```

**Gates:** perception container **< 1 core** (§17.4); cloud →
`/obstacles_safe` **p95 ≤ 60 ms** (§17.2).

Reference numbers to compare against, all dev machine:

| | sim (Phase 4) | hardware path, DLIO in the loop (5A bench) |
|---|---|---|
| cloud → `/scan` p50/p95 | 0.48 / 0.95 ms | 9.24 / 12.81 ms |
| cloud → `/obstacles_safe` p50/p95 | 1.28 / 1.76 ms | 9.61 / 13.19 ms |
| container CPU | 4.5 % of one core | — |

The hardware-path column is dominated by waiting for DLIO's scan-rate TF. If
the Orin's per-scan processing is slower, that wait grows — **this is the
number that decides conditional patch P-5** (broadcast `odom→base_link` from
DLIO's 100 Hz `publishPose()` as well). Record it even if the gate passes.

---

## Block 9 (optional, off-robot) — what walking changed about the A/B

Not a robot block. Recorded here because it changes how block 5's numbers
should be read and what the follow-on closed-loop session must do.

`phase4_ab_run.sh` (the §17.3 offline A/B) replays the fixture bags against a
robot **pinned at (0,0,0)**. It scores command tracking, and it structurally
cannot produce a collision rate or a clearance distribution, because nothing
moves. `walk_ab_run.sh` is the closed-loop counterpart: g1_ctrl's policy
walking under a scripted profile, DPCBF in the 1 kHz seam, the perception
stack live, once per DPCBF mode on an identical seeded field. Both are needed
and they answer different questions — keep running both.

Consequences for 5B:

* **The fixture bags have a perfectly stationary sensor.** `s1_surveyed`,
  `s2_cross_*`, `s3_swarm` and `s4_occlusion` were recorded with a constant
  grounded `qpos` (`make_scenario_scene.grounded_qpos`). Every containment and
  A/B number derived from them is therefore a **lower bound** on what a moving
  platform sees. When a hardware capture is taken with the robot walking, do
  not compare it against the fixture numbers as if they were like-for-like —
  say which platform state each came from.
* **Bring-up on hardware is the same procedure the scripted profile encodes**:
  hold the robot, `L2+up` → FixStand, `R2+A` → RLBase, *then* lower it. Do not
  lower onto FixStand's PD hold pose.
* **`walk_ab_run.sh` needs no bags** — it is a live session — so it is the
  cheapest way to re-verify the whole chain on the target before block 5, if
  there is a safe space to walk in.

---

## End of session

- [ ] every bag `md5sum`'d, sized, and listed in `test_fixtures/README.md`
      with its recipe and what it is for
- [ ] session log written: date, operator, Q-1 answers, topology decision,
      per-block outcome with numbers, anything aborted and why
- [ ] props and survey measurements photographed or sketched (the layout file
      is only as good as the survey behind it)
- [ ] nothing tuned on the robot: every retune is offline, from a bag, with
      the measurement recorded
