# G1 navigation policy bundle

This directory is self-contained so it can be copied to the robot without
depending on the training checkout.

- `exported/high_level_navigation_policy/gat_encoder.onnx`: dynamic
  `[batch, N, 8]` nodes to the 16D robot embedding. Nodes are ordered as
  robot, goal, then up to ten prioritized perception obstacles; there are no
  dummy nodes or valid masks.
- `exported/high_level_navigation_policy/policy_head.onnx`: 16D robot
  embedding plus the 13D local state to physical `[a_s, a_l, yaw_rate]`,
  10 Hz.
- `exported/low_level_locomotion_policy.onnx`: 98D locomotion observation to
  29 joint actions, 50 Hz.
- `params/navigation.yaml`: goal, watchdog and visualization settings.
- `params/low_level_deploy.yaml`: the low-level observation/action pipeline.

Source file checksums:

```text
888ace07bd01fe3162f338b2ee4ee6547b193d5d87cb04a1e0cc0cb4646fc6ff  high_level_navigation_policy/gat_encoder.onnx
b4cd7af597775deff1f502a47666a255fe55244fbc35018053f0b3f29b877b93  high_level_navigation_policy/policy_head.onnx
c69420ee69213fe3c2afcbca3216bf023779c9be964ab9474dcd1c9e75172d4d  low_level_locomotion_policy.onnx
```

The eight node features are `[is_robot, is_obstacle, is_goal, relative_x,
relative_y, radius, relative_vx, relative_vy]` in the robot body frame. The
obstacle radius is the circle radius received from `/obstacles_safe`; the GAT
computes pairwise surface distances internally.

Navigation always waits for the first external `/navigation/goal`. After that
goal is reached in both position and heading, random follow-up goals are
generated only when `enable_random_goal: true` and `g1_ctrl` runs with
`--network=lo`. Hardware never generates random goals. Otherwise,
`hold_goal_after_reaching: true` preserves the reached goal: the command stays
zero until a selected, sufficiently close obstacle approaches, then Navigation
avoids it and returns to the preserved pose. A right-click Stop always clears
the goal and keeps the command at zero.
