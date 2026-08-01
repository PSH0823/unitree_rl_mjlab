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

## wall_scene (Phase 2, 2026-08-01) — COMMITTED

Measured-wall validation fixture (generator + GT, no binary payload): see
`wall_scene/README.md`. Outputs regenerate to `/tmp/wall_scene_phase2` at
test time; the ±2 cm gate runs in CTest
(`test_wall_accuracy.launch_test.py`).
