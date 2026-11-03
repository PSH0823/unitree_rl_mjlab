#include <cmath>

#include <gtest/gtest.h>

#include "g1_perception_utils/footprint.hpp"

using g1_perception_utils::Pose;
using g1_perception_utils::ProjectToFootprint;

namespace {

Pose FromRpy(double x, double y, double z, double roll, double pitch, double yaw) {
  // URDF-convention quaternion from extrinsic-xyz rpy.
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  Pose p;
  p.x = x; p.y = y; p.z = z;
  p.qw = cr * cp * cy + sr * sp * sy;
  p.qx = sr * cp * cy - cr * sp * sy;
  p.qy = cr * sp * cy + sr * cp * sy;
  p.qz = cr * cp * sy - sr * sp * cy;
  return p;
}

double Yaw(const Pose& p) {
  return std::atan2(2.0 * (p.qw * p.qz + p.qx * p.qy),
                    1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz));
}

TEST(Footprint, DropsHeightKeepsXY) {
  const auto fp = ProjectToFootprint(FromRpy(1.5, -2.0, 0.79, 0, 0, 0));
  EXPECT_DOUBLE_EQ(fp.x, 1.5);
  EXPECT_DOUBLE_EQ(fp.y, -2.0);
  EXPECT_DOUBLE_EQ(fp.z, 0.0);
  EXPECT_DOUBLE_EQ(fp.qw, 1.0);
}

TEST(Footprint, KeepsPureYaw) {
  const double yaw = 2.2;
  const auto fp = ProjectToFootprint(FromRpy(0, 0, 0.8, 0, 0, yaw));
  EXPECT_NEAR(Yaw(fp), yaw, 1e-12);
  EXPECT_NEAR(fp.qx, 0.0, 1e-12);
  EXPECT_NEAR(fp.qy, 0.0, 1e-12);
}

TEST(Footprint, RemovesRollPitchKeepsHeading) {
  // Walking sway: roll/pitch up to ~0.3 rad must vanish, heading must stay.
  const double yaw = -1.1;
  const auto fp = ProjectToFootprint(FromRpy(0.3, 0.4, 0.75, 0.25, -0.3, yaw));
  EXPECT_NEAR(Yaw(fp), yaw, 1e-9);
  EXPECT_NEAR(fp.qx, 0.0, 1e-12);
  EXPECT_NEAR(fp.qy, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(fp.z, 0.0);
  // Unit quaternion out.
  const double n = fp.qw * fp.qw + fp.qx * fp.qx + fp.qy * fp.qy + fp.qz * fp.qz;
  EXPECT_NEAR(n, 1.0, 1e-12);
}

TEST(Footprint, YawWrapStable) {
  for (double yaw = -3.1; yaw <= 3.1; yaw += 0.37) {
    const auto fp = ProjectToFootprint(FromRpy(0, 0, 1.0, 0.1, 0.1, yaw));
    EXPECT_NEAR(std::remainder(Yaw(fp) - yaw, 2 * M_PI), 0.0, 1e-9) << "yaw=" << yaw;
  }
}

}  // namespace
