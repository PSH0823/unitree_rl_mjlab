#pragma once
// Loads BoundaryParams from the SAME dpcbf_config.yaml the simulator loads.
//
// Split out of dpcbf_boundary.h so the math header stays dependency-free
// (unit-testable without yaml-cpp). Consumers that need this pull yaml-cpp
// themselves; dpcbf_ros_adapter's own library does not include this header
// and so gains no dependency.
//
// The keys and defaults mirror DpcbfSafetyFilter::LoadConfig exactly. Keys
// the filter requires (ReadDouble) are required here too: a config the
// filter would reject must not silently yield a plausible-looking overlay.

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "dpcbf_ros_adapter/dpcbf_boundary.h"

namespace dpcbf_ros_adapter {

inline BoundaryParams LoadBoundaryParams(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node robot = root["robot"];
  const YAML::Node qp = root["qp_parameters"];
  if (!robot || !qp) {
    throw std::runtime_error("missing 'robot' or 'qp_parameters' in " +
                             path.string());
  }
  auto required = [](const YAML::Node& node, const char* key) {
    if (!node[key]) {
      throw std::runtime_error(std::string("missing '") + key + "'");
    }
    return node[key].as<double>();
  };

  BoundaryParams p;
  p.k_mu = required(robot, "k_mu");
  p.k_lambda = required(robot, "k_lambda");
  p.robot_radius = required(robot, "r_rob");
  p.detection_radius = required(robot, "p_max");
  p.safety_factor = required(robot, "s");
  p.eps_v = required(robot, "eps_v");
  p.eps_d = required(robot, "eps_d");
  p.obstacle_priority = qp["obstacle_priority"].as<int>(0);
  if (!qp["default_num_constraints"]) {
    throw std::runtime_error("missing 'default_num_constraints'");
  }
  p.max_constraints = qp["default_num_constraints"].as<int>();
  p.dpcbf_enabled = qp["dpcbf_enabled"].as<bool>(true);
  p.ecbf_enabled = qp["ecbf_enabled"].as<bool>(false);
  p.slack_enabled = qp["slack_enabled"].as<bool>(false);
  if (p.safety_factor <= 1.0 || p.robot_radius <= 0.0 ||
      p.detection_radius <= 0.0 || p.eps_v <= 0.0 || p.eps_d <= 0.0 ||
      p.max_constraints <= 0) {
    throw std::runtime_error("invalid boundary parameter range in " +
                             path.string());
  }
  return p;
}

}  // namespace dpcbf_ros_adapter
