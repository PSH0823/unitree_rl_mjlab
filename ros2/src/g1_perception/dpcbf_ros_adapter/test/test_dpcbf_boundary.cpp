// Unit cover for the overlay's recomputed constraint geometry.
//
// The authoritative gate is `boundary_check math`, which proves this header
// against the frozen dpcbf library on a real capture — but that capture is a
// generated fixture and absent on a bare clone, so the gate SKIPs there.
// These cases pin the parts most likely to be got wrong, on any machine:
// the selection rule (which is NOT nearest-first under the shipped config)
// and the h=0 <-> (vertex, curvature) identity.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "dpcbf_ros_adapter/dpcbf_boundary.h"

namespace dra = dpcbf_ros_adapter;

namespace {

dra::BoundaryParams ShippedParams() {
  dra::BoundaryParams p;  // defaults mirror dpcbf/config/dpcbf_config.yaml
  return p;
}

dpcbf::ObstacleState Obs(int id, double x, double y, double r, double vx = 0.0,
                         double vy = 0.0) {
  return {x, y, r, vx, vy, id};
}

}  // namespace

// The gate is centre-to-centre against p_max; the obstacle radius is NOT
// subtracted. An obstacle whose *surface* is inside p_max but whose centre is
// outside is not constrained — drawing it as selected would be a lie.
TEST(DpcbfBoundary, DetectionGateIsCentreToCentre) {
  const auto p = ShippedParams();
  dpcbf::RobotState robot;
  const std::vector<dpcbf::ObstacleState> obstacles = {
      Obs(1, 2.99, 0.0, 0.3),
      Obs(2, 3.01, 0.0, 0.3),  // surface at 2.71 m, centre outside p_max
  };
  const auto sel = dra::SelectAndEvaluate(p, robot, obstacles);
  ASSERT_EQ(sel.size(), 1u);
  EXPECT_EQ(sel[0].obstacle.id, 1);
}

// obstacle_priority: 0 -> nearest first.
TEST(DpcbfBoundary, PriorityZeroOrdersByDistance) {
  auto p = ShippedParams();
  p.obstacle_priority = 0;
  dpcbf::RobotState robot;
  const std::vector<dpcbf::ObstacleState> obstacles = {
      Obs(1, 2.5, 0.0, 0.25),
      Obs(2, 1.0, 0.0, 0.25),
      Obs(3, 2.0, 0.0, 0.25),
  };
  const auto sel = dra::SelectAndEvaluate(p, robot, obstacles);
  ASSERT_EQ(sel.size(), 3u);
  EXPECT_EQ(sel[0].obstacle.id, 2);
  EXPECT_EQ(sel[1].obstacle.id, 3);
  EXPECT_EQ(sel[2].obstacle.id, 1);
}

// obstacle_priority: 1 (the SHIPPED value) -> highest closing alignment
// first, distance only as tie-break. The far obstacle closing head-on
// outranks the near one drifting away; an overlay that highlights "the
// nearest N" would mark the wrong ones.
TEST(DpcbfBoundary, PriorityOneOrdersByClosingAlignment) {
  auto p = ShippedParams();
  p.obstacle_priority = 1;
  dpcbf::RobotState robot;  // at rest at the origin
  const std::vector<dpcbf::ObstacleState> obstacles = {
      Obs(1, 1.0, 0.0, 0.25, 1.0, 0.0),   // near, receding
      Obs(2, 2.5, 0.0, 0.25, -1.0, 0.0),  // far, closing head-on
  };
  const auto sel = dra::SelectAndEvaluate(p, robot, obstacles);
  ASSERT_EQ(sel.size(), 2u);
  EXPECT_EQ(sel[0].obstacle.id, 2);
  EXPECT_GT(sel[0].closing_alignment, 0.99);
  EXPECT_LT(sel[1].closing_alignment, -0.99);
}

TEST(DpcbfBoundary, TruncatesToMaxConstraints) {
  auto p = ShippedParams();
  p.max_constraints = 3;
  dpcbf::RobotState robot;
  std::vector<dpcbf::ObstacleState> obstacles;
  for (int i = 0; i < 8; ++i) {
    obstacles.push_back(Obs(i, 0.5 + 0.2 * i, 0.0, 0.2));
  }
  const auto sel = dra::SelectAndEvaluate(p, robot, obstacles);
  EXPECT_EQ(sel.size(), 3u);
  for (std::size_t i = 0; i < sel.size(); ++i) {
    EXPECT_EQ(sel[i].rank, static_cast<int>(i));
  }
}

// The whole point of the overlay's constraint layer: the drawn curve
// x~ = vertex - curvature * y~^2 is exactly the h=0 level set. If this
// identity breaks, the picture stops meaning what its caption says.
TEST(DpcbfBoundary, ParabolaIsTheZeroLevelSetOfH) {
  const auto p = ShippedParams();
  dpcbf::RobotState robot;
  robot.sagittal_velocity = 0.6;
  robot.lateral_velocity = -0.2;
  robot.phi = 0.4;
  const std::vector<dpcbf::ObstacleState> obstacles = {
      Obs(7, 1.4, 0.9, 0.27, -0.3, 0.15)};
  const auto sel = dra::SelectAndEvaluate(p, robot, obstacles);
  ASSERT_EQ(sel.size(), 1u);
  const auto& b = sel[0];

  // h, rewritten through the two drawn coefficients, must reproduce h.
  const double x_tilde = b.relative_velocity_los[0];
  const double y_tilde = b.relative_velocity_los[1];
  const double from_curve =
      x_tilde + b.boundary_curvature * y_tilde * y_tilde - b.boundary_vertex_x;
  EXPECT_NEAR(from_curve, b.h, 1e-12);

  // The drawn curve separates the two signs of h the way the picture claims:
  // the shaded side (x~ left of the curve) is h < 0, the open side h > 0.
  // Evaluated with the frozen coefficients — which is exactly what is drawn.
  const auto h_at = [&b](double x, double y) {
    return x + b.boundary_curvature * y * y - b.boundary_vertex_x;
  };
  for (const double y : {-1.0, -0.25, 0.0, 0.4, 1.3}) {
    const double x_on = b.boundary_vertex_x - b.boundary_curvature * y * y;
    EXPECT_NEAR(h_at(x_on, y), 0.0, 1e-12);
    EXPECT_LT(h_at(x_on - 0.1, y), 0.0);
    EXPECT_GT(h_at(x_on + 0.1, y), 0.0);
  }
}

// The eCBF family the shipped config also runs: its zero set is the circle
// |p| = r_rob + r_obs, which is the one piece of constraint geometry that
// genuinely belongs on a metre grid.
TEST(DpcbfBoundary, EcbfZeroSetIsTheCombinedRadiusCircle) {
  const auto p = ShippedParams();
  dpcbf::RobotState robot;
  const double r_obs = 0.3;
  const double touch = p.robot_radius + r_obs;
  const auto sel = dra::SelectAndEvaluate(
      p, robot, {Obs(1, touch, 0.0, r_obs), Obs(2, 0.5 * touch, 0.0, r_obs)});
  ASSERT_EQ(sel.size(), 2u);
  for (const auto& b : sel) {
    EXPECT_NEAR(b.ecbf_radius, touch, 1e-15);
    if (b.obstacle.id == 1) {
      EXPECT_NEAR(b.ecbf_h, 0.0, 1e-12);  // touching
    }
    if (b.obstacle.id == 2) {
      EXPECT_LT(b.ecbf_h, 0.0);  // overlapping
    }
  }
}

TEST(DpcbfBoundary, EmptyInputSelectsNothing) {
  const auto p = ShippedParams();
  dpcbf::RobotState robot;
  EXPECT_TRUE(dra::SelectAndEvaluate(p, robot, {}).empty());
}
