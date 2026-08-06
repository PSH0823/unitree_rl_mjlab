# One obstacle AS THE FILTER CONSUMED IT this tick (from
# SafetyFilterResult::selected_obstacles — post staleness materialization,
# post candidate gate/ordering/truncation), with its DPCBF barrier value.
# World (odom) frame, metres / m/s.
int32 id
float64 x
float64 y
float64 radius          # radius the QP used (already margin/inflation-carrying)
float64 velocity_x
float64 velocity_y
float64 distance        # centre-to-centre robot distance [m]
float64 h               # DPCBF barrier value; h < 0 means constraint violated
