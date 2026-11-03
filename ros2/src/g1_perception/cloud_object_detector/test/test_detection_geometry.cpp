// Unit tests for the pure geometry in detection_geometry.h. Same philosophy
// as safety_obstacle_filter/test_gating.cpp: everything a reviewer would
// want to poke at without a ROS graph.
#include "cloud_object_detector/detection_geometry.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace cloud_object_detector {
namespace {

double SignedArea(const std::vector<Point2>& poly) {
  double a = 0.0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const auto& p = poly[i];
    const auto& q = poly[(i + 1) % poly.size()];
    a += p.x * q.y - q.x * p.y;
  }
  return 0.5 * a;
}

TEST(ConvexHull2D, SquareWithInteriorPointsKeepsCornersOnly) {
  std::vector<Point2> pts = {{0, 0}, {1, 0}, {1, 1}, {0, 1},
                             {0.5, 0.5}, {0.2, 0.8}, {0.9, 0.1}};
  const auto hull = ConvexHull2D(pts);
  ASSERT_EQ(hull.size(), 4u);
  EXPECT_NEAR(SignedArea(hull), 1.0, 1e-12);  // positive = CCW
}

TEST(ConvexHull2D, CollinearInputHasNoArea) {
  std::vector<Point2> pts = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
  const auto hull = ConvexHull2D(pts);
  EXPECT_LT(hull.size(), 3u);
}

TEST(ConvexHull2D, DuplicatePointsDoNotInflateTheHull) {
  std::vector<Point2> pts = {{0, 0}, {0, 0}, {1, 0}, {1, 0}, {0.5, 1},
                             {0.5, 1}};
  const auto hull = ConvexHull2D(pts);
  EXPECT_EQ(hull.size(), 3u);
}

TEST(DecimateHull, CapsVertexCountAndKeepsEndpointsConvex) {
  // Regular 32-gon.
  std::vector<Point2> poly;
  for (int i = 0; i < 32; ++i) {
    const double th = 2.0 * M_PI * i / 32.0;
    poly.push_back({std::cos(th), std::sin(th)});
  }
  DecimateHull(&poly, 8);
  ASSERT_EQ(poly.size(), 8u);
  EXPECT_GT(SignedArea(poly), 0.0);  // still CCW
}

TEST(DecimateHull, NoOpWhenSmallEnough) {
  std::vector<Point2> poly = {{0, 0}, {1, 0}, {0, 1}};
  DecimateHull(&poly, 8);
  EXPECT_EQ(poly.size(), 3u);
}

TEST(EstimateShape, BoxClusterYieldsCentredFootprintAndHeight) {
  // 2 x 1 x 1.5 box of points centred at (10, 5), z in [0.25, 1.75].
  std::vector<std::array<double, 3>> pts;
  for (double x : {9.0, 11.0})
    for (double y : {4.5, 5.5})
      for (double z : {0.25, 1.75}) pts.push_back({x, y, z});
  ShapeParams p;
  const auto est = EstimateShape(pts, p);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(est.cx, 10.0, 1e-12);
  EXPECT_NEAR(est.cy, 5.0, 1e-12);
  EXPECT_NEAR(est.cz, 1.0, 1e-12);
  EXPECT_NEAR(est.height, 1.5, 1e-12);
  ASSERT_EQ(est.footprint.size(), 4u);
  // Footprint is relative to the centroid: corners at (±1, ±0.5).
  for (const auto& v : est.footprint) {
    EXPECT_NEAR(std::abs(v.x), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(v.y), 0.5, 1e-12);
  }
  EXPECT_NEAR(SignedArea(est.footprint), 2.0, 1e-12);
}

TEST(EstimateShape, FlatClusterRejectedByMinHeight) {
  std::vector<std::array<double, 3>> pts = {
      {0, 0, 0.10}, {1, 0, 0.11}, {0, 1, 0.12}, {1, 1, 0.10}};
  ShapeParams p;  // min_height default 0.05 > 0.02 spread
  EXPECT_FALSE(EstimateShape(pts, p).valid);
}

TEST(EstimateShape, CollinearClusterRejected) {
  std::vector<std::array<double, 3>> pts = {
      {0, 0, 0.0}, {1, 1, 0.5}, {2, 2, 1.0}, {3, 3, 1.5}};
  ShapeParams p;
  EXPECT_FALSE(EstimateShape(pts, p).valid);
}

TEST(EstimateShape, TooFewPointsRejected) {
  std::vector<std::array<double, 3>> pts = {{0, 0, 0}, {1, 0, 1}};
  ShapeParams p;
  EXPECT_FALSE(EstimateShape(pts, p).valid);
}

TEST(MeasureHull, RectangleExtentAndWidth) {
  // 2.0 x 0.2 rectangle: extent = diagonal, width = 0.2.
  const std::vector<Point2> hull = {{0, 0}, {2, 0}, {2, 0.2}, {0, 0.2}};
  const auto m = MeasureHull(hull);
  EXPECT_NEAR(m.max_extent, std::hypot(2.0, 0.2), 1e-12);
  EXPECT_NEAR(m.min_width, 0.2, 1e-12);
}

TEST(EstimateShape, WallSegmentRejectedAsWall) {
  // 2 m long, 0.15 m thick, 1.5 m tall: a wall face seen by the lidar.
  std::vector<std::array<double, 3>> pts;
  for (double x = 0.0; x <= 2.0; x += 0.25)
    for (double y : {0.0, 0.15})
      for (double z : {0.2, 1.7}) pts.push_back({x, y, z});
  ShapeParams p;
  const auto est = EstimateShape(pts, p);
  EXPECT_FALSE(est.valid);
  EXPECT_TRUE(est.is_wall);
}

TEST(EstimateShape, PillarSurvivesWallGate) {
  // r=0.25 pillar: extent 0.5 < wall_min_length, kept.
  std::vector<std::array<double, 3>> pts;
  for (int a = 0; a < 12; ++a) {
    const double th = 2.0 * M_PI * a / 12.0;
    pts.push_back({0.25 * std::cos(th), 0.25 * std::sin(th), 0.2});
    pts.push_back({0.25 * std::cos(th), 0.25 * std::sin(th), 1.5});
  }
  ShapeParams p;
  const auto est = EstimateShape(pts, p);
  EXPECT_TRUE(est.valid);
  EXPECT_FALSE(est.is_wall);
}

TEST(EstimateShape, TwoAbreastPedestriansSurviveWallGate) {
  // Merged cluster ~1.1 m across but ~0.5 m thick: long but NOT thin.
  std::vector<std::array<double, 3>> pts;
  for (double cx : {0.0, 0.8}) {
    for (int a = 0; a < 12; ++a) {
      const double th = 2.0 * M_PI * a / 12.0;
      pts.push_back({cx + 0.25 * std::cos(th), 0.25 * std::sin(th), 0.2});
      pts.push_back({cx + 0.25 * std::cos(th), 0.25 * std::sin(th), 1.6});
    }
  }
  ShapeParams p;
  const auto est = EstimateShape(pts, p);
  EXPECT_TRUE(est.valid);
  EXPECT_FALSE(est.is_wall);
}

TEST(EstimateShape, HugeBlobRejectedByAbsoluteCap) {
  // 4 x 4 m block: not thin, but bigger than any obstacle -> dropped.
  std::vector<std::array<double, 3>> pts;
  for (double x = 0.0; x <= 4.0; x += 0.5)
    for (double y = 0.0; y <= 4.0; y += 0.5)
      for (double z : {0.2, 1.5}) pts.push_back({x, y, z});
  ShapeParams p;
  const auto est = EstimateShape(pts, p);
  EXPECT_FALSE(est.valid);
  EXPECT_TRUE(est.is_wall);
}

TEST(EstimateShape, FrontFaceArcGetsDepthCompleted) {
  // Sensor at origin; pillar r=0.25 at x=2 seen as its front half only —
  // the realistic single-viewpoint cluster. Unpadded, that hull is only
  // ~0.25 m deep; depth completion must extrude it away from the sensor and
  // move the pose off the visible surface toward the true center (x=2).
  std::vector<std::array<double, 3>> pts;
  for (int a = 0; a < 13; ++a) {
    const double th = M_PI / 2.0 + M_PI * a / 12.0;  // sensor-facing half
    const double x = 2.0 + 0.25 * std::cos(th);
    const double y = 0.25 * std::sin(th);
    pts.push_back({x, y, 0.2});
    pts.push_back({x, y, 1.5});
  }
  ShapeParams p;
  p.min_thickness = 0.4;  // > the arc's own depth (0.25), forces extrusion
  const auto est = EstimateShape(pts, p, 0.0, 0.0);
  ASSERT_TRUE(est.valid);
  const auto m = MeasureHull(est.footprint);
  EXPECT_GE(m.min_width, 0.9 * p.min_thickness);
  // Surface points span x in [1.75, 2.0]; their mean is ~1.83. The padded
  // centroid must sit past the surface mean, toward/behind the center.
  EXPECT_GT(est.cx, 1.9);
  EXPECT_NEAR(est.cy, 0.0, 0.05);
}

TEST(EstimateShape, ThickClusterNotPadded) {
  // Full ring r=0.25 (min_width 0.5 > min_thickness): footprint unchanged
  // by depth completion even with a sensor position provided.
  std::vector<std::array<double, 3>> pts;
  for (int a = 0; a < 16; ++a) {
    const double th = 2.0 * M_PI * a / 16.0;
    pts.push_back({2.0 + 0.25 * std::cos(th), 0.25 * std::sin(th), 0.2});
    pts.push_back({2.0 + 0.25 * std::cos(th), 0.25 * std::sin(th), 1.5});
  }
  ShapeParams p;
  const auto est = EstimateShape(pts, p, 0.0, 0.0);
  ASSERT_TRUE(est.valid);
  const auto m = MeasureHull(est.footprint);
  EXPECT_NEAR(m.max_extent, 0.5, 0.02);  // still the ring, not a slab
  EXPECT_NEAR(est.cx, 2.0, 1e-6);
}

TEST(EstimateShape, ClusterAlongRayGetsLateralThickness) {
  // A near-line of points ALONG the viewing ray (sensor at origin, cluster
  // on the x axis, 0.6 m long, ~4 cm wide): stage-1 extrusion away from the
  // sensor only lengthens it; stage 2 must pad its thin axis so it is no
  // longer a line. Also exercises the padded point set.
  std::vector<std::array<double, 3>> pts;
  for (int i = 0; i <= 6; ++i) {
    const double x = 2.0 + 0.1 * i;
    const double y = (i % 2 == 0) ? 0.02 : -0.02;
    pts.push_back({x, y, 0.2});
    pts.push_back({x, y, 1.4});
  }
  ShapeParams p;  // min_thickness 0.25, wall gate: extent 0.6 < 1.0 -> kept
  const auto est = EstimateShape(pts, p, 0.0, 0.0);
  ASSERT_TRUE(est.valid);
  const auto m = MeasureHull(est.footprint);
  EXPECT_GE(m.min_width, 0.9 * p.min_thickness);
  EXPECT_GT(est.padded_points.size(), pts.size());
  // The padded body keeps the z extent of the cluster.
  double zmin = 1e9, zmax = -1e9;
  for (const auto& q : est.padded_points) {
    zmin = std::min(zmin, q[2]);
    zmax = std::max(zmax, q[2]);
  }
  EXPECT_NEAR(zmin, 0.2, 1e-9);
  EXPECT_NEAR(zmax, 1.4, 1e-9);
}

TEST(EstimateShape, PaddedPointsAreExactCopiesOfTheCluster) {
  // Regression: the extrusion once held a reference into the vector it was
  // appending to; after reallocation the copies carried z = 0. Every padded
  // point must be an input point shifted horizontally by a bounded amount.
  std::vector<std::array<double, 3>> pts;
  for (int a = 0; a < 13; ++a) {
    const double th = M_PI / 2.0 + M_PI * a / 12.0;
    pts.push_back({2.0 + 0.25 * std::cos(th), 0.25 * std::sin(th), 0.3});
    pts.push_back({2.0 + 0.25 * std::cos(th), 0.25 * std::sin(th), 1.1});
  }
  ShapeParams p;
  p.min_thickness = 0.6;  // forces both stages
  const auto est = EstimateShape(pts, p, 0.0, 0.0);
  ASSERT_TRUE(est.valid);
  ASSERT_GT(est.padded_points.size(), pts.size());
  for (const auto& q : est.padded_points) {
    EXPECT_TRUE(q[2] == 0.3 || q[2] == 1.1) << "z corrupted: " << q[2];
    double best = 1e9;
    for (const auto& o : pts) {
      if (o[2] != q[2]) continue;
      best = std::min(best, std::hypot(q[0] - o[0], q[1] - o[1]));
    }
    EXPECT_LE(best, p.min_thickness + 1e-9);
  }
}

TEST(EstimateShape, WallGateDisabledByZeros) {
  std::vector<std::array<double, 3>> pts;
  for (double x = 0.0; x <= 2.0; x += 0.25)
    for (double y : {0.0, 0.15})
      for (double z : {0.2, 1.7}) pts.push_back({x, y, z});
  ShapeParams p;
  p.wall_min_length = 0.0;
  p.max_object_extent = 0.0;
  EXPECT_TRUE(EstimateShape(pts, p).valid);
}

}  // namespace
}  // namespace cloud_object_detector
