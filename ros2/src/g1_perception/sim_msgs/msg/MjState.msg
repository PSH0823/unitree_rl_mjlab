# Mirror state of the running MuJoCo simulation (§7.1, §11.2).
# Published by the simulate ROS2 module at 100 Hz, BestEffort depth 1.
# The sidecar writes qpos/mocap into its mirror model and runs mj_kinematics —
# poses only, no dynamics.

float64 sim_time      # MuJoCo d->time [s]; the authoritative sim clock for stamps
float64[] qpos        # length nq
float64[] mocap_pos   # length 3*nmocap, row-major per mocap body
float64[] mocap_quat  # length 4*nmocap, (w x y z) per mocap body
