#pragma once
// Joystick-axis <-> velocity-command mapping used at the DPCBF seam. Moved
// VERBATIM from simulate/src/main.cc (baseline f111cfa lineage) so that the
// production seam and the T1 replay harness compile the exact same code —
// any divergence here is precisely what T1 exists to catch.

#include <algorithm>

namespace dpcbf_ros_adapter {

inline double AxisToCommand(double axis, double minimum, double maximum) {
  const double normalized = std::clamp(axis, -1.0, 1.0);
  return normalized >= 0.0 ? normalized * maximum : -normalized * minimum;
}

inline float CommandToAxis(double command, double minimum, double maximum) {
  if (command >= 0.0) {
    return static_cast<float>(maximum > 0.0 ? command / maximum : 0.0);
  }
  return static_cast<float>(minimum < 0.0 ? -command / minimum : 0.0);
}

}  // namespace dpcbf_ros_adapter
