#pragma once
// Recomputation of the DPCBF constraint geometry, for visualization only.
//
// WHAT THE BOUNDARY IS (settled from source, not from the name "parabola"):
// the DPCBF barrier is
//
//   h = x~ + A*( lambda*y~^2 + k_mu*d_safe ),
//       A      = sqrt(s^2 - 1) / r_safe
//       lambda = k_lambda * d_safe / v_safe
//
// (dpcbf/src/dpcbf_safety_filter.cpp EvaluateConstraint), so h >= 0 is
//
//   x~ >= vertex_x - curvature * y~^2
//
// with the two coefficients the filter itself already names in Filter()
// (vertex_x = -A*k_mu*d_safe, curvature = A*k_lambda*d_safe/v_safe).
//
// (x~, y~) is the RELATIVE VELOCITY v_obs - v_robot rotated into the
// obstacle line-of-sight frame. Both axes are m/s. The boundary therefore
// lives in VELOCITY SPACE, per obstacle -- it is NOT an envelope drawn
// around the obstacle in the world. Rendering it in a metric world frame is
// a category error; see dpcbf/include/dpcbf/dpcbf_safety_filter.h:47.
//
// The shipped config also runs ecbf_enabled, which adds a SECOND family per
// obstacle: a distance barrier px^2+py^2-(r_rob+r_obs)^2 whose zero set IS
// honest world geometry (a circle of radius r_rob+r_obs). Both are exposed
// here so a viewer can show what the QP actually solves rather than half of
// it.
//
// This header is the single implementation shared by two consumers:
//   * g1_perception_utils/dpcbf_overlay_node  (draws it, live)
//   * dpcbf_ros_adapter/tools/boundary_check  (proves it against the frozen
//     library, offline -- CTest dpcbf_boundary_recomputation)
// so the picture and the gate can never drift apart. dpcbf/ is frozen (D3)
// and is only READ here.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"

namespace dpcbf_ros_adapter {

// Mirrors dpcbf_safety_filter.cpp's file-local kMinimumDenominator.
constexpr double kBoundaryMinDenominator = 1.0e-9;

// The subset of dpcbf_config.yaml that determines selection and geometry.
// Read from the same file the simulator loads, never re-declared as ROS
// defaults -- a drifting copy would make the overlay lie confidently.
struct BoundaryParams {
  double k_mu = 1.0;
  double k_lambda = 0.5;
  double robot_radius = 0.30;      // robot.r_rob
  double detection_radius = 3.0;   // robot.p_max
  double safety_factor = 1.05;     // robot.s
  double eps_v = 0.05;
  double eps_d = 0.05;
  int obstacle_priority = 1;       // qp_parameters.obstacle_priority
  int max_constraints = 10;        // qp_parameters.default_num_constraints
  bool ecbf_enabled = true;
  bool dpcbf_enabled = true;
  bool slack_enabled = true;
};

// One obstacle the filter would constrain on this tick, with both constraint
// families evaluated. Field names match the filter's own vocabulary.
struct BoundaryObstacle {
  dpcbf::ObstacleState obstacle;
  int rank = 0;                 // 0 = first row of the QP
  double distance = 0.0;        // centre-to-centre, the gate the filter uses
  double closing_alignment = 0.0;

  double los_angle = 0.0;
  double relative_velocity_world[2] = {0.0, 0.0};
  double relative_velocity_los[2] = {0.0, 0.0};   // (x~, y~), m/s

  // DPCBF: h = 0 is x~ = boundary_vertex_x - boundary_curvature * y~^2,
  // in the LoS relative-velocity plane. Units: vertex m/s, curvature s/m.
  double boundary_vertex_x = 0.0;
  double boundary_curvature = 0.0;
  double h = 0.0;

  // eCBF: the distance barrier's zero set, which IS world geometry.
  double ecbf_radius = 0.0;     // r_rob + r_obs  [m]
  double ecbf_h = 0.0;          // px^2+py^2 - ecbf_radius^2  [m^2]

  // Shared geometry, world frame, honest metres.
  double safe_radius = 0.0;     // (r_rob + r_obs) * s
  double safe_distance = 0.0;   // smooth clearance to the safe_radius circle
};

// Reproduces Filter()'s candidate gate + ordering + truncation exactly.
//   gate:  centre-to-centre distance <= p_max  (obstacle radius NOT subtracted)
//   order: obstacle_priority == 1 -> closing alignment descending, distance
//          as tie-break; otherwise nearest distance first
//   count: truncate to default_num_constraints
// Deliberately a std::stable_sort mirror of the filter's std::sort with the
// same comparator: see boundary_check for the equivalence proof against the
// frozen library on real captures.
inline std::vector<BoundaryObstacle> SelectAndEvaluate(
    const BoundaryParams& p, const dpcbf::RobotState& robot,
    const std::vector<dpcbf::ObstacleState>& obstacles) {
  const double robot_velocity_x = robot.sagittal_velocity * std::cos(robot.phi) -
                                  robot.lateral_velocity * std::sin(robot.phi);
  const double robot_velocity_y = robot.sagittal_velocity * std::sin(robot.phi) +
                                  robot.lateral_velocity * std::cos(robot.phi);

  struct Candidate {
    double distance;
    double closing_alignment;
    const dpcbf::ObstacleState* obstacle;
  };
  std::vector<Candidate> nearest;
  nearest.reserve(obstacles.size());
  for (const dpcbf::ObstacleState& o : obstacles) {
    const double dx = o.x - robot.x;
    const double dy = o.y - robot.y;
    const double distance = std::hypot(dx, dy);
    if (distance > p.detection_radius) continue;
    const double rvx = o.velocity_x - robot_velocity_x;
    const double rvy = o.velocity_y - robot_velocity_y;
    const double relative_speed = std::hypot(rvx, rvy);
    double closing_alignment = 0.0;
    if (relative_speed > kBoundaryMinDenominator &&
        distance > kBoundaryMinDenominator) {
      closing_alignment = -(rvx * dx + rvy * dy) / (relative_speed * distance);
      closing_alignment = std::max(-1.0, std::min(closing_alignment, 1.0));
    }
    nearest.push_back({distance, closing_alignment, &o});
  }

  std::sort(nearest.begin(), nearest.end(),
            [&p](const Candidate& left, const Candidate& right) {
              if (p.obstacle_priority == 1 &&
                  std::abs(left.closing_alignment - right.closing_alignment) >
                      1.0e-12) {
                return left.closing_alignment > right.closing_alignment;
              }
              return left.distance < right.distance;
            });
  if (nearest.size() > static_cast<std::size_t>(p.max_constraints)) {
    nearest.resize(static_cast<std::size_t>(p.max_constraints));
  }

  std::vector<BoundaryObstacle> out;
  out.reserve(nearest.size());
  for (std::size_t i = 0; i < nearest.size(); ++i) {
    const dpcbf::ObstacleState& o = *nearest[i].obstacle;
    const double px = o.x - robot.x;
    const double py = o.y - robot.y;
    const double p_squared =
        std::max(px * px + py * py, kBoundaryMinDenominator);

    const double los = std::atan2(py, px);
    const double cos_a = std::cos(los);
    const double sin_a = std::sin(los);

    const double rvx = o.velocity_x - robot_velocity_x;
    const double rvy = o.velocity_y - robot_velocity_y;
    const double x_tilde = cos_a * rvx + sin_a * rvy;
    const double y_tilde = -sin_a * rvx + cos_a * rvy;
    const double safe_velocity =
        std::sqrt(rvx * rvx + rvy * rvy + p.eps_v * p.eps_v);

    const double combined_radius = p.robot_radius + o.radius;
    const double safe_radius = combined_radius * p.safety_factor;
    const double smooth_clearance = std::sqrt(
        std::max(p_squared - safe_radius * safe_radius + p.eps_d * p.eps_d,
                 kBoundaryMinDenominator));
    const double safe_distance = smooth_clearance - p.eps_d;
    const double adaptive_scale =
        std::sqrt(p.safety_factor * p.safety_factor - 1.0) / safe_radius;
    const double lambda_unscaled = p.k_lambda * safe_distance / safe_velocity;

    BoundaryObstacle b;
    b.obstacle = o;
    b.rank = static_cast<int>(i);
    b.distance = nearest[i].distance;
    b.closing_alignment = nearest[i].closing_alignment;
    b.los_angle = los;
    b.relative_velocity_world[0] = rvx;
    b.relative_velocity_world[1] = rvy;
    b.relative_velocity_los[0] = x_tilde;
    b.relative_velocity_los[1] = y_tilde;
    b.boundary_vertex_x = -adaptive_scale * p.k_mu * safe_distance;
    b.boundary_curvature =
        adaptive_scale * p.k_lambda * safe_distance / safe_velocity;
    b.h = x_tilde +
          adaptive_scale * (lambda_unscaled * y_tilde * y_tilde +
                            p.k_mu * safe_distance);
    b.ecbf_radius = combined_radius;
    b.ecbf_h = px * px + py * py - combined_radius * combined_radius;
    b.safe_radius = safe_radius;
    b.safe_distance = safe_distance;
    out.push_back(b);
  }
  return out;
}

}  // namespace dpcbf_ros_adapter
