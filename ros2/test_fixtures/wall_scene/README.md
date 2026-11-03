# wall_scene — measured-wall validation fixture (Phase 2 gate, §18)

Self-contained end-to-end accuracy fixture for the projection chain: G1
(scene_g1.xml, H-1 site) + six surveyed targets, grounded standing pose.
`make_wall_scene.py` compiles the scene with python mujoco and emits the
sidecar mirror XML plus a ground-truth JSON (outputs go to `/tmp`, nothing
here is generated-state). No simulate binary, no GL, no dynamics: the pose
is a static qpos published by `test/wall_state_source.py`, so the robot is
GROUNDED (pelvis z 0.8091 m, mid360 origin 1.2816 m — zero-joint standing,
feet AABB touching z=0), not hanging in the Phase-1 suspension rig.

## Surveyed geometry (ranges from base_footprint origin to nearest surface)

| target | kind | bearing | face range | role |
|---|---|---|---|---|
| wall_1m | box face | 0° | 1.000 m | ±2 cm gate |
| wall_2m | box face | 90° | 2.000 m | ±2 cm gate |
| wall_4m | box face | 180° | 4.000 m | ±2 cm gate |
| cyl_1m | cylinder r=0.15 | 45° | 1.000 m | Q-3 bin-occupancy baseline |
| cyl_2m | cylinder r=0.15 | 135° | 2.000 m | Q-3 bin-occupancy baseline |
| cyl_3m | cylinder r=0.15 | −45° | 3.000 m | Q-3 bin-occupancy baseline |

Bearing spans are disjoint by construction (walls 1 m wide). Walls are 2.5 m
tall, cylinders 1.5 m (the sim obstacle height); all intersect the §9.4
height band [0.15, 1.60] from the world-downward Mid360 fan (roll=π mount:
datasheet −7…+52° maps to +7° up / 52° down in world).

## Run

CTest: part of `colcon test` (g1_perception_bringup,
`test_wall_accuracy.launch_test.py` — SKIPs without python mujoco).
Standalone gate + benchmark:

```bash
python3 test_fixtures/wall_scene/make_wall_scene.py            # -> /tmp/wall_scene_phase2
launch_test src/g1_perception/g1_perception_bringup/test/test_wall_accuracy.launch_test.py
# Q-3 stats: run the stack (see the launch test) + test/phase2_probe.py --gt-json
```

## Phase-2 results (2026-08-01, full numbers in the §21 log)

Wall median errors −0.04 / −0.07 / −0.15 mm at 1/2/4 m (gate ±20 mm);
T9 zero misses; evidence `ros2/evidence/phase2/scan_vs_gt_topdown.png`.
Known artifact of the grounded zero pose: both wrist links survive the
Appendix-A CropBox at horizontal 0.29–0.36 m (z 0.87–0.94 m, i.e. 0.34–0.41 m
below the sensor, outside the box's 0.25 m below-sensor coverage) and appear
in /scan just past range_min — Phase-3 must expect them, Phase-5 retunes the
box with real self-hit data.

Phase 5 repeats the same assertions against a real wall (tape-measured) —
keep the tolerance and bearings-window logic in sync with the launch test.
