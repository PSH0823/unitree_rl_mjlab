# Test fixtures

Bags are gitignored (size); regenerate with the recipe below and verify
against the recorded hash in the §21 log entry that references them.

## s1_static_reference (Phase 1, 2026-08-01)

S1-style scene: robot suspended by the elastic band (sensor ≈1.84 m), 90 GT
obstacles moving. sqlite3 (MCAP blocked on apt/sudo — see log).
27.87 s, 92.9 MiB, md5(db3) `d372a619c563e0b4953247dafdfc51a0`.

Regenerate (simulator with ROS2 module + bringup running):

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
sqlite3, 75 MiB, md5(db3) `64abe6da6688bd8b8a9b480294e6cb07`.

## s2_cross_05 / s2_cross_08 (Phase 3, 2026-08-01)

T5 fixtures: single cylinder crossing on x=2.0 from y=−4.9 to +4.9 at 0.5
resp. 0.8 m/s — start/end OUTSIDE range_max so the track is born on a
moving object (the T5 "crossing" reading). GT (position + commanded
velocity) in-bag. sqlite3; 53 MiB md5 `72b811a9f1c5211986fd50eaec098bb4`
(0.5); 35 MiB md5 `352f72adce964ffa2b84301e802ebd74` (0.8).

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

## wall_scene (Phase 2, 2026-08-01) — COMMITTED

Measured-wall validation fixture (generator + GT, no binary payload): see
`wall_scene/README.md`. Outputs regenerate to `/tmp/wall_scene_phase2` at
test time; the ±2 cm gate runs in CTest
(`test_wall_accuracy.launch_test.py`).
