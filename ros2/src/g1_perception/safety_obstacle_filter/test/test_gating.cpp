// Unit tests for every §9.6 rule: age gate, wall-arc gate, min-radius clamp,
// inflation formula, velocity clamp, empty input, passthrough fields.
#include "safety_obstacle_filter/gating.h"

#include <gtest/gtest.h>

namespace sof = safety_obstacle_filter;

namespace {

obstacle_detector::msg::CircleObstacle Circle(double x, double y, double tr,
                                              double vx, double vy,
                                              std::uint64_t uid = 1) {
  obstacle_detector::msg::CircleObstacle c;
  c.uid = uid;
  c.center.x = x;
  c.center.y = y;
  c.true_radius = tr;
  c.radius = tr + 0.17;  // detector margin — must be ignored by the filter
  c.velocity.x = vx;
  c.velocity.y = vy;
  return c;
}

obstacle_detector::msg::Obstacles Msg(double stamp,
                                      std::vector<obstacle_detector::msg::CircleObstacle> cs) {
  obstacle_detector::msg::Obstacles m;
  m.header.stamp.sec = static_cast<int32_t>(stamp);
  m.header.stamp.nanosec =
      static_cast<uint32_t>((stamp - m.header.stamp.sec) * 1e9);
  m.header.frame_id = "odom";
  m.circles = std::move(cs);
  return m;
}

}  // namespace

TEST(Gating, InflationFormula) {
  sof::Params p;  // Appendix-A defaults
  auto out = sof::Apply(Msg(10.0, {Circle(1, 2, 0.25, 0.6, -0.8)}), p, 10.1);
  ASSERT_EQ(out.circles.size(), 1u);
  // r=max(0.25,0.20)=0.25; |v|=1.0; radius=0.25+0.03+1.0*0.12=0.40.
  EXPECT_NEAR(out.circles[0].radius, 0.40, 1e-12);
  EXPECT_DOUBLE_EQ(out.circles[0].true_radius, 0.25);  // passthrough
  EXPECT_EQ(out.circles[0].uid, 1u);
  EXPECT_DOUBLE_EQ(out.circles[0].center.x, 1.0);
}

TEST(Gating, MinRadiusClamp) {
  sof::Params p;
  auto out = sof::Apply(Msg(0.0, {Circle(0, 0, 0.15, 0, 0)}), p, 0.0);
  ASSERT_EQ(out.circles.size(), 1u);
  // clamp to 0.20, static: 0.20 + 0.03 = 0.23.
  EXPECT_NEAR(out.circles[0].radius, 0.23, 1e-12);
}

TEST(Gating, WallArcGate) {
  sof::Params p;
  auto out = sof::Apply(
      Msg(0.0, {Circle(0, 0, 0.61, 0, 0, 7), Circle(1, 1, 0.59, 0, 0, 8)}),
      p, 0.0);
  ASSERT_EQ(out.circles.size(), 1u);  // 0.61 > max_circle_radius dropped
  EXPECT_EQ(out.circles[0].uid, 8u);
}

TEST(Gating, VelocityClampBeforeInflation) {
  sof::Params p;
  // KF spike: 3-4-5 triangle at |v|=5 → clamped to 1.5 keeping direction.
  auto out = sof::Apply(Msg(0.0, {Circle(0, 0, 0.25, 3.0, 4.0)}), p, 0.0);
  ASSERT_EQ(out.circles.size(), 1u);
  EXPECT_NEAR(out.circles[0].velocity.x, 1.5 * 3.0 / 5.0, 1e-12);
  EXPECT_NEAR(out.circles[0].velocity.y, 1.5 * 4.0 / 5.0, 1e-12);
  // Inflation uses the CLAMPED speed: 0.25 + 0.03 + 1.5*0.12 = 0.46.
  EXPECT_NEAR(out.circles[0].radius, 0.46, 1e-12);
}

TEST(Gating, StaleMessageDropsAllCirclesKeepsStamp) {
  sof::Params p;
  auto out = sof::Apply(Msg(10.0, {Circle(0, 0, 0.25, 0, 0)}), p, 10.31);
  EXPECT_TRUE(out.circles.empty());
  // The measurement stamp passes through so the adapter's age accounting
  // still sees the truth.
  EXPECT_EQ(out.header.stamp.sec, 10);
  // Boundary: age == max_age is NOT stale (stamp 0 keeps the subtraction
  // exact — 10.30 - 10.0 is not representable as exactly 0.30).
  out = sof::Apply(Msg(0.0, {Circle(0, 0, 0.25, 0, 0)}), p, 0.30);
  EXPECT_EQ(out.circles.size(), 1u);
}

TEST(Gating, EmptyInput) {
  sof::Params p;
  auto out = sof::Apply(Msg(1.0, {}), p, 1.0);
  EXPECT_TRUE(out.circles.empty());
  EXPECT_EQ(out.header.frame_id, "odom");
}

TEST(Gating, SegmentsNotForwarded) {
  sof::Params p;
  auto in = Msg(1.0, {Circle(0, 0, 0.25, 0, 0)});
  in.segments.emplace_back();
  auto out = sof::Apply(in, p, 1.0);
  EXPECT_TRUE(out.segments.empty());
  EXPECT_EQ(out.circles.size(), 1u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
