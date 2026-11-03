# T1 baseline capture (gate T1, §16.2 — oracle equivalence)

The pre-refactor reference for the Phase-4 seam refactor: every `Filter()`
call (inputs AND outputs) logged at the 1 kHz seam by the **pre-refactor**
simulate binary, driven by a deterministic scripted command profile on the
seeded (seed 42) 90-obstacle field, robot in the elastic-band suspension rig.

Files:

- `t1_command_profile.txt` — committed. Sim-time axis breakpoints
  (piecewise-constant hold) consumed by the ScriptedJoystick
  (`UNITREE_MUJOCO_SCRIPTED_COMMANDS`).
- `0001-t1-baseline-instrumentation.patch` — committed. The exact
  instrumentation applied to the baseline tree (worktree at `9ca8a2e`):
  ScriptedJoystick + FilterIoWriter logging ONLY — the seam itself is
  untouched. The capture struct layout comes from the main worktree's
  `dpcbf_ros_adapter/filter_io_log.h`, so both binaries share one format
  by construction.
- `t1_baseline_capture.bin` — generated, gitignored.
  Phase-4 reference: 38 402+ records, md5 `e4a5caef830ce24cd05280467ac629a7`
  (tail may be SIGINT-truncated; t1_replay compares complete records only).

## Regenerating

```bash
git worktree add /tmp/t1_baseline_tree 9ca8a2e
cd /tmp/t1_baseline_tree && git apply <this dir>/0001-t1-baseline-instrumentation.patch
# scratch-only config: use_joystick 0, enable_elastic_band 1
mkdir -p simulate/build && cd simulate/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUNITREE_MUJOCO_WITH_ROS2=OFF \
      -DCMAKE_PREFIX_PATH=<repo>/ros2/install
make -j unitree_mujoco
LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
UNITREE_MUJOCO_SCRIPTED_COMMANDS=<this dir>/t1_command_profile.txt \
UNITREE_DPCBF_FILTER_LOG=<this dir>/t1_baseline_capture.bin \
timeout -s INT 40 ./unitree_mujoco
```

## Checking (the gate)

```bash
cd simulate/build_ros2 && ctest -R t1_oracle_equivalence --output-on-failure
# or directly:
./t1_replay <this dir>/t1_baseline_capture.bin ../../dpcbf/config/dpcbf_config.yaml
```

Phase-4 result: **T1 PASS — 38 402 Filter() calls byte-identical** (commands,
accelerations, constraint bookkeeping, solved flags, output axes), including
ticks where OSQP hit kMaxIterations — the hold-last-feasible fallback replays
bit-exactly too.
