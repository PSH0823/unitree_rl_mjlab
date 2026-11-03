# Test fixtures

Bags are gitignored (size); regenerate with the recipe below and verify
against the hash recorded here.

**Any change to `obstacle_detector/msg/Obstacles` invalidates every bag that
carries it** (all six below record `/sim/gt_obstacles`). Patch 0007 did exactly
that; five bags were regenerated in the interim block and their hashes are
updated here. `s1_static_reference` was the sixth and was missed —
**regenerated 2026-08-02 in the runbook block; all six are now post-0007.**
[Checked 2026-08-02.]

## s1_static_reference (Phase 1, 2026-08-01)

S1-style scene: robot suspended by the elastic band (sensor ≈1.84 m), 90 GT
obstacles moving. sqlite3.
**Regenerated 2026-08-02 (post-patch-0007): 28.89 s, 98.1 MiB, 289 clouds,
md5(db3) `3466e2cf54b1f3374497796c33fbcbbb`** (the pre-0007 bag was
27.87 s / 92.9 MiB / `d372a619c563e0b4953247dafdfc51a0`).
Verified after regeneration: `test_projection_replay` gives 289 `/scan`
frames over 28.8 s sim time → 10.00 Hz, 0.0 % drop, T9 288/0 misses.

Regenerate (shadow run-tree simulator with the ROS2 module + description +
sidecar running — NOT the perception container, which is what replays it;
see `doc/operator_runbook.md` §3.2 and §9.1). Record ~29 s of wall clock at
realtime factor ≈1; with the container also running the factor drops to ≈0.44
and the same wall time yields half the bag:

```bash
ros2 bag record -o s1_static_reference /livox/lidar /odom /tf /tf_static \
    /sim/gt_obstacles /sim/mj_state /clock
```

## scenarios (Phase 3, 2026-08-01) — COMMITTED

Scenario scene generator for the S1/S2 fixtures below:
`scenarios/make_scenario_scene.py` builds scene_g1.xml + 4 mocap cylinders
and writes the mirror model + scenario jsons to `/tmp/scenarios_phase3`.
The trajectories are driven at record time by
`g1_perception_bringup/test/scenario_state_source.py` (mini-sim: /clock +
/sim/mj_state with scripted mocap_pos + /sim/gt_obstacles).

## s1_surveyed (Phase 3, 2026-08-01)

T4 fixture: 3 surveyed cylinders (r=0.15, faces at exactly 1/2/3 m,
bearings 45°/135°/−45°), grounded standing pose, static, 30 s. GT in-bag.
sqlite3, 75 MiB, md5(db3) `889d4c34872652c9b5fa9d92ec4b5c15`
(regenerated post-patch-0007; the pre-0007 bag was `64abe6da…`).

## s2_cross_05 / s2_cross_08 (Phase 3, 2026-08-01)

T5 fixtures: single cylinder crossing on x=2.0 from y=−4.9 to +4.9 at 0.5
resp. 0.8 m/s — start/end OUTSIDE range_max so the track is born on a
moving object (the T5 "crossing" reading). GT (position + commanded
velocity) in-bag. sqlite3; 53 MiB md5 `3fde31e71d5b1124aa8a74fea9456850`
(0.5); 36 MiB md5 `3174aa0bf76d12dc41b7c0cdecbc6309` (0.8).
(regenerated post-patch-0007; pre-0007 were `72b811a9…` / `352f72ad…`.)

Regenerate all three (workspace built & sourced):

```bash
/usr/bin/python3 test_fixtures/scenarios/make_scenario_scene.py --out /tmp/scenarios_phase3
# for each scenario json (s1_static / s2_cross_05 / s2_cross_08):
#   1. launch description.launch.py, perception.launch.py (use_sim_time:=true)
#      and source_sim.launch.py mirror_model_path:=/tmp/scenarios_phase3/scenario_mirror.xml
#   2. ros2 bag record -o test_fixtures/<name> /livox/lidar /odom /tf /tf_static \
#          /sim/gt_obstacles /sim/mj_state /clock
#   3. run src/g1_perception/g1_perception_bringup/test/scenario_state_source.py \
#          --scenario-json /tmp/scenarios_phase3/<name>.json   # exits at scenario end
# (record BEFORE starting the scenario source so t=0 is in-bag; SIGINT + pkill
#  discipline between runs — see ros2/README.md runtime notes)
```

## s3_swarm (Phase 4, 2026-08-01)

§17.1 S3: 20-obstacle seeded swarm (seed 20260801), radii ∈ [0.15, 0.28]
(straddles the min_radius 0.20 clamp), box-reflect motion at 0.2–0.8 m/s in
x∈[0.8,7], y∈[−4,4] — the DynamicObstacleManager motion model in scripted
form. Mirror: `scenario_mirror_p4.xml` (nmocap=25 — the S1/S2 mirror stays
untouched because those bags pin nmocap=4). 33 s, 87 MiB, md5(db3)
`4282942ccc6e471761d73ebfd5c7d13d` (regenerated post-patch-0007; pre-0007
`5f0e98e3…`).

## s4_occlusion (Phase 4, 2026-08-01)

§17.1 S4: static blocker r=0.30 at (2.0, 0) + crosser on x=2.6 at 0.6 m/s —
INSIDE p_max (first cut used x=3.2; DPCBF culls at 3.0 m, so occlusion
containment there had no safety meaning). Shadow occludes the crosser
~1.8 s > tracking_duration 1.0 s: the track dies mid-shadow and re-acquires
on emergence (the §10.3 worst case, on purpose). 21.4 s, 49 MiB, md5(db3)
`8851168594c5018dbb865e46401e1490` (regenerated post-patch-0007; pre-0007
`83b2311f…`).

Regenerate S3/S4: same recipe as above with
`mirror_model_path:=/tmp/scenarios_phase3/scenario_mirror_p4.xml` and the
s3_swarm / s4_occlusion jsons.

## t1_baseline (Phase 4, 2026-08-01) — see t1_baseline/README.md

Gate-T1 pre-refactor Filter-I/O capture (patch + profile committed, capture
gitignored; md5 `e4a5caef830ce24cd05280467ac629a7`).

## wall_scene (Phase 2, 2026-08-01) — COMMITTED

Measured-wall validation fixture (generator + GT, no binary payload): see
`wall_scene/README.md`. Outputs regenerate to `/tmp/wall_scene_phase2` at
test time; the ±2 cm gate runs in CTest
(`test_wall_accuracy.launch_test.py`).

## hw/<YYYY-MM-DD>/ (Phase 5B) — robot-session bags, NOT YET RECORDED

Reserved layout for the hardware session. One directory per session date,
one bag per checklist block (`doc/phase5b_checklists.md`):

| bag | block | contents |
|---|---|---|
| `b1_raw_driver` | 1 | `/livox/lidar /livox/imu` — first artefact, recorded before anything consumes it |
| `b2_wall` | 2 | + `/odom /tf /tf_static /scan` — surveyed-wall extrinsic check |
| `b3_dlio_static` | 3 | ≥10 min stationary, for the <1 cm/min drift gate |
| `b3_dlio_carried` | 3 | optional, externally moved |
| `b4_selfhit` | 4 | `/livox/lidar` only, **perception NOT running** (CropBox bypassed): static / arm-swing / torso-motion |
| `b5_t4` | 5 | full stack + surveyed props incl. one r ≥ 0.30 m; paired with `t4_layout.yaml` |
| `b6_q3`, `b6_q8` | 6 | rosette density parity vs the Phase-2 table; near-field head-aperture scan |
| `b7_q5` | 7 | Unitree onboard odometry alongside `/odom` |

Each bag gets an md5, a size, a duration and a one-line "what it is for" here
at the end of the session, same as the sim fixtures above. The layout file
`t4_layout.yaml` (surveyed GT, odom frame) is **committed** — it is the
hardware equivalent of `scenarios/make_scenario_scene.py`'s GT, and
`test_detection_static.launch_test.py` reads it via `T4_LAYOUT`.
