#pragma once
// The complete axes→command→axes seam logic shared by simulate/src/main.cc
// (production) and tools/t1_replay.cc (gate T1). The expressions are the
// baseline f111cfa lambda's, verbatim — including the axis sign conventions.
// T1 compares Filter() I/O byte-for-byte against the pre-refactor capture;
// keeping this code in ONE translation-unit-shared header is what makes that
// comparison meaningful over time.

#include <array>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/axis_command_map.h"

namespace dpcbf_ros_adapter {

struct SeamLimits {
  double sagittal_min = 0.0, sagittal_max = 0.0;
  double lateral_min = 0.0, lateral_max = 0.0;
  double yaw_rate_min = 0.0, yaw_rate_max = 0.0;

  static SeamLimits FromFilter(const dpcbf::DpcbfSafetyFilter& f) {
    return {f.sagittal_velocity_min(), f.sagittal_velocity_max(),
            f.lateral_velocity_min(),  f.lateral_velocity_max(),
            f.yaw_rate_min(),          f.yaw_rate_max()};
  }
};

inline dpcbf::VelocityCommand AxesToDesired(float lx, float ly, float rx,
                                            const SeamLimits& lim) {
  dpcbf::VelocityCommand desired;
  desired.sagittal = AxisToCommand(ly, lim.sagittal_min, lim.sagittal_max);
  desired.lateral = AxisToCommand(-lx, lim.lateral_min, lim.lateral_max);
  desired.yaw_rate = AxisToCommand(-rx, lim.yaw_rate_min, lim.yaw_rate_max);
  return desired;
}

inline std::array<float, 3> CommandToAxes(const dpcbf::VelocityCommand& cmd,
                                          const SeamLimits& lim) {
  return {-CommandToAxis(cmd.lateral, lim.lateral_min, lim.lateral_max),
          CommandToAxis(cmd.sagittal, lim.sagittal_min, lim.sagittal_max),
          -CommandToAxis(cmd.yaw_rate, lim.yaw_rate_min, lim.yaw_rate_max)};
}

// §10.3/§10.5: the staleness command scaling applied at the call site,
// BEFORE Filter(). scale is Snapshot::command_scale (1.0 when fresh).
inline dpcbf::VelocityCommand ScaleDesired(const dpcbf::VelocityCommand& d,
                                           double scale) {
  if (scale >= 1.0) return d;  // bit-exact no-op on the fresh/oracle path
  dpcbf::VelocityCommand out;
  out.sagittal = d.sagittal * scale;
  out.lateral = d.lateral * scale;
  out.yaw_rate = d.yaw_rate * scale;
  return out;
}

}  // namespace dpcbf_ros_adapter
