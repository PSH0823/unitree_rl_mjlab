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

TEST(Gating, WallArcGate) {
  sof::Params p;
  auto out = sof::Apply(
      Msg(0.0, {Circle(0, 0, 0.61, 0, 0, 7), Circle(1, 1, 0.59, 0, 0, 8)}),
      p, 0.0);
  ASSERT_EQ(out.circles.size(), 1u);  // 0.61 > max_circle_radius dropped
  EXPECT_EQ(out.circles[0].uid, 8u);
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

TEST(Gating, RadiusScaleIsConfigurable) {
  sof::Params p;
  p.min_radius = 0.20;
  p.fixed_inflation = 0.03;
  p.latency_horizon = 0.10;
  p.radius_scale = 0.50;
  auto out = sof::Apply(Msg(0.0, {Circle(0, 0, 0.10, 1.0, 0.0)}), p, 0.0);
  ASSERT_EQ(out.circles.size(), 1u);
  // 0.5 * (min_radius 0.20 + fixed 0.03 + speed horizon 0.10)
  EXPECT_NEAR(out.circles[0].radius, 0.165, 1e-12);
}

// --- Observability of the silent paths (gaps G1/G2) -------------------------

TEST(Stats, LargeRadiusDropIsCounted) {
  // The whole G2 lesson: a drop nobody can observe is the defect, wherever
  // the threshold sits. Apply() must be able to say what it threw away.
  sof::Params p;
  sof::Stats s;
  auto out = sof::Apply(
      Msg(0.0, {Circle(0, 0, 0.25, 0, 0), Circle(2, 0, 0.90, 0, 0, 2)}), p,
      0.0, &s);
  EXPECT_EQ(out.circles.size(), 1u);
  EXPECT_EQ(s.dropped_large_radius, 1);
  EXPECT_NEAR(s.radius_max_dropped, 0.90, 1e-12);
  EXPECT_EQ(s.stale_messages, 0);
}

TEST(Stats, StaleMessageIsCounted) {
  sof::Params p;
  sof::Stats s;
  auto out = sof::Apply(Msg(0.0, {Circle(0, 0, 0.25, 0, 0)}), p, 1.0, &s);
  EXPECT_TRUE(out.circles.empty());
  EXPECT_EQ(s.stale_messages, 1);
}

TEST(Stats, ImplausibleSigmaIsFlaggedSeparatelyFromTheCap) {
  // sqrt(P(0,0)) is metres by construction; a half-metre sigma means the
  // tracker's measurement_variance (m^2) was never set from data (gap G1).
  // That must be distinguishable from a legitimately capped diverged track,
  // because the remedies are opposite: fix R, versus trust the cap.
  sof::Params p;
  p.use_covariance = true;
  p.sigma_max = 0.50;
  sof::Stats s;
  auto c = Circle(0, 0, 0.25, 0, 0);
  c.covariance = {0.34, 0.34, 0.0};  // sigma ≈ 0.583 m — the measured value
  sof::Apply(Msg(0.0, {c}), p, 0.0, &s);
  EXPECT_EQ(s.sigma_implausible, 1);
  EXPECT_EQ(s.sigma_clamped, 1);
  EXPECT_GT(s.sigma_max_seen, sof::kSigmaPlausibleMax);

  // A plausible sigma trips neither counter.
  sof::Stats s2;
  auto c2 = Circle(0, 0, 0.25, 0, 0);
  c2.covariance = {1.0e-4, 1.0e-4, 1.0e-4};  // sigma ≈ 14 mm
  sof::Apply(Msg(0.0, {c2}), p, 0.0, &s2);
  EXPECT_EQ(s2.sigma_implausible, 0);
  EXPECT_EQ(s2.sigma_clamped, 0);
}

TEST(Stats, NullStatsIsAllowedAndBehaviourIsIdentical) {
  sof::Params p;
  sof::Stats s;
  auto a = sof::Apply(Msg(0.0, {Circle(0, 0, 0.25, 0.5, 0)}), p, 0.0);
  auto b = sof::Apply(Msg(0.0, {Circle(0, 0, 0.25, 0.5, 0)}), p, 0.0, &s);
  ASSERT_EQ(a.circles.size(), b.circles.size());
  EXPECT_DOUBLE_EQ(a.circles[0].radius, b.circles[0].radius);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
