# One decimated snapshot of the DPCBF control seam, published by the
# in-process plot bridge (dpcbf_ros_adapter::DpcbfVizPublisher) at the
# plot rate (default 30 Hz), NOT at the control rate. Everything here is
# copied from ONE control tick — the fields are mutually consistent.
#
# header.stamp is the WALL clock of the publishing process at sample copy
# time (the plotting client computes message age against its own clock and,
# more robustly, against time-since-last-receive). t_ctrl is the control
# loop's own clock (sim time in simulation; monotonic seconds on hardware)
# and is the x-axis to plot against.

std_msgs/Header header

uint64 tick               # control ticks since start (Filter() calls)
float64 t_ctrl            # control-side time of this tick [s]

# Obstacle source mode (dpcbf_ros_adapter::ObstacleSource::Mode)
uint8 MODE_ORACLE=0
uint8 MODE_SHADOW=1
uint8 MODE_ESTIMATED=2
uint8 mode

# Robot state the filter used (dpcbf::RobotState; odom frame, body-frame vels)
float64 robot_x
float64 robot_y
float64 robot_phi
float64 robot_sagittal_velocity
float64 robot_lateral_velocity

# Command chain of this tick (§10.5 seam order):
#   nominal  = AxesToDesired(joystick)         — the operator's intent
#   scaled   = nominal * command_scale         — §10.3 staleness degrade ramp
#   safe     = Filter(robot, scaled, obstacles).command — what the robot gets
VelocityCommand2D nominal
VelocityCommand2D scaled
VelocityCommand2D safe
float64 command_scale     # 1.0 when the obstacle stream is fresh

# QP outcome
bool solved
int32 active_constraints
int32 active_dpcbf_constraints
int32 active_ecbf_constraints
float64[3] acceleration   # SafetyFilterResult::acceleration (normalized)

# True when the QP changed the command it was given (safe != scaled beyond
# 1e-6 on any axis). The staleness ramp is deliberately NOT counted as an
# intervention — compare nominal vs scaled / command_scale for that.
bool intervention

# min over selected obstacles of the DPCBF barrier h. Valid only when
# min_h_valid (i.e. at least one obstacle was selected this tick).
bool min_h_valid
float64 min_h
# min over selected obstacles of (distance - radius) [m]: the closest
# surface clearance the filter saw. Valid under the same flag.
float64 min_clearance

# Obstacle stream staleness as the control side saw it
# (dpcbf_ros_adapter::StalenessState; age is -1 when unknown/oracle).
uint8 STALENESS_FRESH=0
uint8 STALENESS_DEGRADE=1
uint8 STALENESS_STOP=2
uint8 STALENESS_NO_DATA=3
uint8 staleness_state
float64 obstacle_age_s

# The obstacle set the QP consumed this tick (<= qp max constraints, so this
# stays small). The FULL perception stream is /obstacles_safe — subscribe to
# that directly if you need obstacles beyond the filter's selection.
uint32 obstacle_total     # obstacles offered to the filter before selection
PlotObstacle[] obstacles
