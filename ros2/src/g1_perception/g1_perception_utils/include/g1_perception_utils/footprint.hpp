#pragma once

#include <cmath>

namespace g1_perception_utils {

struct Pose {
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
};

// base_footprint (§8.1): base_link projected to z = 0 with roll = pitch = 0,
// keeping x, y and the base yaw. Yaw is extracted ZYX-style so it stays the
// heading even when the torso pitches/rolls during walking.
inline Pose ProjectToFootprint(const Pose& base) {
  Pose out;
  out.x = base.x;
  out.y = base.y;
  out.z = 0.0;
  const double yaw = std::atan2(
      2.0 * (base.qw * base.qz + base.qx * base.qy),
      1.0 - 2.0 * (base.qy * base.qy + base.qz * base.qz));
  out.qx = 0.0;
  out.qy = 0.0;
  out.qz = std::sin(0.5 * yaw);
  out.qw = std::cos(0.5 * yaw);
  return out;
}

}  // namespace g1_perception_utils
