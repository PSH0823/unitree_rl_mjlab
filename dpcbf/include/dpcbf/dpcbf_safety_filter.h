#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace dpcbf {

struct RobotState {
  double x = 0.0;
  double y = 0.0;
  double phi = 0.0;
  double sagittal_velocity = 0.0;
  double lateral_velocity = 0.0;
};

struct ObstacleState {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  int id = -1;
};

struct VelocityCommand {
  double sagittal = 0.0;
  double lateral = 0.0;
  double yaw_rate = 0.0;
};

struct DpcbfConstraint {
  double h = 0.0;
  double lf_h = 0.0;
  std::array<double, 3> lg_h{};
};

struct EcbfConstraint {
  double distance_barrier = 0.0;
  double distance_barrier_dot = 0.0;
  std::array<double, 3> control_coefficients{};
  double drift = 0.0;
};

// Frozen-parameter h=0 parabola in the obstacle Line-of-Sight velocity frame:
//   X = boundary_vertex_x - boundary_curvature * Y^2
struct DpcbfVisualizationObstacle {
  ObstacleState obstacle;
  DpcbfConstraint constraint;
  double distance = 0.0;
  std::array<double, 2> relative_velocity_world{};
  std::array<double, 2> relative_velocity_los{};
  double boundary_vertex_x = 0.0;
  double boundary_curvature = 0.0;
};

struct SafetyFilterResult {
  VelocityCommand command;
  // [normalized_delta_v_s, normalized_delta_v_l, yaw_rate]. The first two
  // entries use the configured reference control frequency. The applied
  // per-tick delta-v is value * reference_frequency_hz * control_dt.
  std::array<double, 3> acceleration{};
  int active_constraints = 0;
  int active_dpcbf_constraints = 0;
  int active_ecbf_constraints = 0;
  bool solved = false;
  std::vector<double> decay_variables;
  std::vector<double> slack_variables;
  std::vector<DpcbfVisualizationObstacle> selected_obstacles;
};

class DpcbfSafetyFilter {
 public:
  DpcbfSafetyFilter();
  ~DpcbfSafetyFilter();
  DpcbfSafetyFilter(DpcbfSafetyFilter&&) noexcept;
  DpcbfSafetyFilter& operator=(DpcbfSafetyFilter&&) noexcept;
  DpcbfSafetyFilter(const DpcbfSafetyFilter&) = delete;
  DpcbfSafetyFilter& operator=(const DpcbfSafetyFilter&) = delete;

  void LoadConfig(const std::filesystem::path& path);
  void Initialize(double control_dt);
  SafetyFilterResult Filter(const RobotState& robot,
                            const VelocityCommand& desired,
                            const std::vector<ObstacleState>& obstacles);

  DpcbfConstraint EvaluateConstraint(const RobotState& robot,
                                     const ObstacleState& obstacle) const;
  EcbfConstraint EvaluateEcbfConstraint(
      const RobotState& robot, const ObstacleState& obstacle) const;

  bool enabled() const;
  double sagittal_velocity_min() const;
  double sagittal_velocity_max() const;
  double lateral_velocity_min() const;
  double lateral_velocity_max() const;
  double yaw_rate_min() const;
  double yaw_rate_max() const;
  const std::string& base_body_name() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dpcbf
