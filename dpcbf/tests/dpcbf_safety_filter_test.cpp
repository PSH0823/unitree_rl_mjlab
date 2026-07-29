#include "dpcbf/dpcbf_safety_filter.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

void ExpectNear(double actual, double expected, double tolerance,
                const char* label) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(label) + " mismatch: actual=" +
                             std::to_string(actual) + ", expected=" +
                             std::to_string(expected));
  }
}

dpcbf::SafetyFilterResult RunFeatureCombination(
    const YAML::Node& source_config, const char* name,
    bool dpcbf_enabled, bool ecbf_enabled,
    bool odcbf_enabled, bool slack_enabled) {
  YAML::Node config = YAML::Clone(source_config);
  YAML::Node qp = config["qp_parameters"];
  qp["enabled"] = true;
  qp["dpcbf_enabled"] = dpcbf_enabled;
  qp["ecbf_enabled"] = ecbf_enabled;
  qp["odcbf_enabled"] = odcbf_enabled;
  qp["slack_enabled"] = slack_enabled;
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      (std::string("dpcbf_") + name + "_test.yaml");
  {
    std::ofstream output(path);
    output << config;
  }

  dpcbf::DpcbfSafetyFilter filter;
  filter.LoadConfig(path);
  filter.Initialize(0.002);
  const dpcbf::RobotState robot{};
  const dpcbf::VelocityCommand desired{0.3, 0.0, 0.0};
  const std::vector<dpcbf::ObstacleState> obstacles{
      {2.0, 0.4, 0.2, 0.0, 0.0, 7}};
  const auto result = filter.Filter(robot, desired, obstacles);
  std::filesystem::remove(path);
  if (!result.solved) {
    throw std::runtime_error(
        std::string("OSQP feature combination failed: ") + name);
  }
  return result;
}

int SelectPriorityObstacle(const YAML::Node& source_config,
                           int obstacle_priority) {
  YAML::Node config = YAML::Clone(source_config);
  YAML::Node qp = config["qp_parameters"];
  qp["enabled"] = true;
  qp["dpcbf_enabled"] = true;
  qp["ecbf_enabled"] = false;
  qp["odcbf_enabled"] = false;
  qp["slack_enabled"] = false;
  qp["default_num_constraints"] = 1;
  qp["obstacle_priority"] = obstacle_priority;
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("dpcbf_priority_" + std::to_string(obstacle_priority) +
       "_test.yaml");
  {
    std::ofstream output(path);
    output << config;
  }

  dpcbf::DpcbfSafetyFilter filter;
  filter.LoadConfig(path);
  filter.Initialize(0.002);
  const dpcbf::RobotState robot{0.0, 0.0, 0.0, 0.5, 0.0};
  const dpcbf::VelocityCommand desired{0.5, 0.0, 0.0};
  const std::vector<dpcbf::ObstacleState> obstacles{
      {-0.5, 0.0, 0.1, 0.0, 0.0, 10},
      {2.0, 0.0, 0.1, 0.0, 0.0, 20},
  };
  const auto result = filter.Filter(robot, desired, obstacles);
  std::filesystem::remove(path);
  if (result.selected_obstacles.size() != 1) {
    throw std::runtime_error("obstacle priority selection count mismatch");
  }
  return result.selected_obstacles.front().obstacle.id;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: dpcbf_safety_filter_test <dpcbf_config.yaml>\n";
    return 2;
  }

  try {
    const YAML::Node config = YAML::LoadFile(argv[1]);
    const YAML::Node qp_config = config["qp_parameters"];
    const int configured_max_constraints =
        qp_config["default_num_constraints"].as<int>();
    const bool configured_dpcbf =
        qp_config["dpcbf_enabled"].as<bool>(true);
    const bool configured_ecbf =
        qp_config["ecbf_enabled"].as<bool>(false);
    const bool configured_odcbf =
        qp_config["odcbf_enabled"].as<bool>(false);
    const bool configured_slack =
        qp_config["slack_enabled"].as<bool>(false);
    dpcbf::DpcbfSafetyFilter filter;
    filter.LoadConfig(std::filesystem::path(argv[1]));
    constexpr double dt = 0.002;
    filter.Initialize(dt);

    const dpcbf::RobotState state{0.3, -0.4, 0.37, 0.8, -0.23};
    const dpcbf::ObstacleState obstacle{2.1, 0.7, 0.32, -0.18, 0.27};
    const auto analytical = filter.EvaluateConstraint(state, obstacle);
    const auto ecbf = filter.EvaluateEcbfConstraint(state, obstacle);
    constexpr double epsilon = 1.0e-6;

    for (int index = 0; index < 3; ++index) {
      auto plus = state;
      auto minus = state;
      double* plus_value = nullptr;
      double* minus_value = nullptr;
      if (index == 0) {
        plus_value = &plus.sagittal_velocity;
        minus_value = &minus.sagittal_velocity;
      } else if (index == 1) {
        plus_value = &plus.lateral_velocity;
        minus_value = &minus.lateral_velocity;
      } else {
        plus_value = &plus.phi;
        minus_value = &minus.phi;
      }
      *plus_value += epsilon;
      *minus_value -= epsilon;
      const double numerical =
          (filter.EvaluateConstraint(plus, obstacle).h -
           filter.EvaluateConstraint(minus, obstacle).h) /
          (2.0 * epsilon);
      ExpectNear(analytical.lg_h[static_cast<std::size_t>(index)], numerical,
                 2.0e-5, "L_g h");
    }

    const double velocity_x = state.sagittal_velocity * std::cos(state.phi) -
                              state.lateral_velocity * std::sin(state.phi);
    const double velocity_y = state.sagittal_velocity * std::sin(state.phi) +
                              state.lateral_velocity * std::cos(state.phi);
    auto robot_plus = state;
    auto robot_minus = state;
    auto obstacle_plus = obstacle;
    auto obstacle_minus = obstacle;
    robot_plus.x += velocity_x * epsilon;
    robot_plus.y += velocity_y * epsilon;
    robot_minus.x -= velocity_x * epsilon;
    robot_minus.y -= velocity_y * epsilon;
    obstacle_plus.x += obstacle.velocity_x * epsilon;
    obstacle_plus.y += obstacle.velocity_y * epsilon;
    obstacle_minus.x -= obstacle.velocity_x * epsilon;
    obstacle_minus.y -= obstacle.velocity_y * epsilon;
    const double numerical_drift =
        (filter.EvaluateConstraint(robot_plus, obstacle_plus).h -
         filter.EvaluateConstraint(robot_minus, obstacle_minus).h) /
        (2.0 * epsilon);
    ExpectNear(analytical.lf_h, numerical_drift, 2.0e-5, "L_f h + obstacle drift");

    const double px = obstacle.x - state.x;
    const double py = obstacle.y - state.y;
    const double cosine = std::cos(state.phi);
    const double sine = std::sin(state.phi);
    const double p_s = px * cosine + py * sine;
    const double p_l = -px * sine + py * cosine;
    ExpectNear(ecbf.control_coefficients[0], -2.0 * p_s, 1.0e-12,
               "eCBF sagittal coefficient");
    ExpectNear(ecbf.control_coefficients[1], -2.0 * p_l, 1.0e-12,
               "eCBF lateral coefficient");
    ExpectNear(ecbf.control_coefficients[2],
               2.0 * (p_s * state.lateral_velocity -
                      p_l * state.sagittal_velocity),
               1.0e-12, "eCBF yaw coefficient");

    const dpcbf::RobotState stopped{};
    const dpcbf::VelocityCommand desired{2.0, 1.0, 1.0};
    const auto unconstrained = filter.Filter(stopped, desired, {});
    if (!unconstrained.solved || unconstrained.active_constraints != 0) {
      throw std::runtime_error("unconstrained OSQP solve failed");
    }
    ExpectNear(unconstrained.command.sagittal,
               unconstrained.acceleration[0] * dt * 500.0, 2.0e-5,
               "sagittal command integration");
    ExpectNear(unconstrained.command.lateral,
               unconstrained.acceleration[1] * dt * 500.0, 2.0e-5,
               "lateral command integration");
    ExpectNear(unconstrained.command.yaw_rate, 1.0, 2.0e-4,
               "yaw command");

    dpcbf::DpcbfSafetyFilter slower_filter;
    slower_filter.LoadConfig(std::filesystem::path(argv[1]));
    constexpr double slower_dt = 0.01;
    slower_filter.Initialize(slower_dt);
    const auto scaled_increment =
        slower_filter.Filter(stopped, desired, {});
    if (!scaled_increment.solved) {
      throw std::runtime_error("scaled velocity-increment solve failed");
    }
    ExpectNear(
        scaled_increment.command.sagittal,
        scaled_increment.acceleration[0] * slower_dt * 500.0, 2.0e-5,
        "scaled sagittal command integration");
    ExpectNear(
        scaled_increment.command.lateral,
        scaled_increment.acceleration[1] * slower_dt * 500.0, 2.0e-5,
        "scaled lateral command integration");

    std::vector<dpcbf::ObstacleState> many_obstacles;
    for (int i = 0; i < 15; ++i) {
      many_obstacles.push_back(
          {1.0 + 0.1 * i, 0.5, 0.1, 0.0, 0.0, i});
    }
    const auto constrained = filter.Filter(stopped, desired, many_obstacles);
    if (!constrained.solved ||
        constrained.active_constraints != configured_max_constraints ||
        constrained.active_dpcbf_constraints !=
            (configured_dpcbf ? configured_max_constraints : 0) ||
        constrained.active_ecbf_constraints !=
            (configured_ecbf ? configured_max_constraints : 0) ||
        constrained.selected_obstacles.size() !=
            static_cast<std::size_t>(configured_max_constraints)) {
      throw std::runtime_error("nearest-obstacle constraint cap was not applied");
    }
    const std::size_t expected_decay_count =
        configured_dpcbf && configured_odcbf
            ? static_cast<std::size_t>(configured_max_constraints)
            : 0U;
    const std::size_t expected_slack_count =
        configured_slack
            ? static_cast<std::size_t>(
                  configured_max_constraints *
                  (static_cast<int>(configured_dpcbf) +
                   static_cast<int>(configured_ecbf)))
            : 0U;
    if (constrained.decay_variables.size() != expected_decay_count ||
        constrained.slack_variables.size() != expected_slack_count) {
      throw std::runtime_error("ODCBF or slack variable count mismatch");
    }
    for (double decay : constrained.decay_variables) {
      if (decay < 1.0 - 2.0e-5) {
        throw std::runtime_error("ODCBF decay lower bound was violated");
      }
    }
    for (double slack : constrained.slack_variables) {
      if (slack < -2.0e-5) {
        throw std::runtime_error("CBF slack lower bound was violated");
      }
    }
    for (std::size_t i = 0; i < constrained.selected_obstacles.size(); ++i) {
      const auto& visual = constrained.selected_obstacles[i];
      if (i > 0 &&
          constrained.selected_obstacles[i - 1].distance > visual.distance) {
        throw std::runtime_error("visualized obstacles are not distance sorted");
      }
      const double x = visual.relative_velocity_los[0];
      const double y = visual.relative_velocity_los[1];
      ExpectNear(x + visual.boundary_curvature * y * y -
                     visual.boundary_vertex_x,
                 visual.constraint.h, 2.0e-8,
                 "visualized h=0 boundary");
    }

    if (SelectPriorityObstacle(config, 0) != 10) {
      throw std::runtime_error(
          "distance priority did not select the nearest obstacle");
    }
    if (SelectPriorityObstacle(config, 1) != 20) {
      throw std::runtime_error(
          "alignment priority did not select the closing obstacle");
    }

    const auto dpcbf_only = RunFeatureCombination(
        config, "dpcbf_only", true, false, false, false);
    if (dpcbf_only.active_dpcbf_constraints != 1 ||
        dpcbf_only.active_ecbf_constraints != 0 ||
        !dpcbf_only.decay_variables.empty() ||
        !dpcbf_only.slack_variables.empty()) {
      throw std::runtime_error("DPCBF-only feature gating failed");
    }
    const auto ecbf_only = RunFeatureCombination(
        config, "ecbf_only", false, true, false, false);
    if (ecbf_only.active_dpcbf_constraints != 0 ||
        ecbf_only.active_ecbf_constraints != 1 ||
        !ecbf_only.decay_variables.empty() ||
        !ecbf_only.slack_variables.empty()) {
      throw std::runtime_error("eCBF-only feature gating failed");
    }
    const auto odcbf_only = RunFeatureCombination(
        config, "odcbf", true, false, true, false);
    if (odcbf_only.decay_variables.size() != 1 ||
        !odcbf_only.slack_variables.empty()) {
      throw std::runtime_error("ODCBF feature gating failed");
    }
    const auto slack_both = RunFeatureCombination(
        config, "slack_both", true, true, false, true);
    if (!slack_both.decay_variables.empty() ||
        slack_both.slack_variables.size() != 2) {
      throw std::runtime_error("slack feature gating failed");
    }

    std::cout << "DPCBF, eCBF, ODCBF, slack, visualization, and OSQP tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
