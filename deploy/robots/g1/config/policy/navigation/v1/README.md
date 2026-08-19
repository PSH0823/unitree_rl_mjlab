# G1 navigation policy bundle

This directory is self-contained so it can be copied to the robot without
depending on the training checkout.

- `exported/navigation.onnx`: 121D observation to physical
  `[a_s, a_l, yaw_rate]`, 10 Hz.
- `exported/low_level_policy.onnx`: 98D velocity-policy observation to 29
  joint actions, 50 Hz.
- `params/navigation.yaml`: goal, watchdog and visualization settings.
- `params/low_level_deploy.yaml`: the low-level observation/action pipeline.

Source file checksums:

```text
50cc6d6310ad34dd4cf3e32a4a902bcd5babdeaa97a5bd5cd2c14c5d7b471b22  navigation.onnx
c69420ee69213fe3c2afcbca3216bf023779c9be964ab9474dcd1c9e75172d4d  low_level_policy.onnx
```

Navigation always waits for the first external `/navigation/goal`. After that
goal is reached in both position and heading, random follow-up goals are
generated only when `enable_random_goal: true` and `g1_ctrl` runs with
`--network=lo`. Hardware never generates random goals.
