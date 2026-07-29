#include "dpcbf/dpcbf_safety_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <osqp++.h>
#include <yaml-cpp/yaml.h>

namespace dpcbf {
namespace {

constexpr double kMinimumDenominator = 1.0e-9;

double ReadDouble(const YAML::Node& node, const char* key) {
  if (!node[key]) {
    throw std::runtime_error(std::string("missing '") + key + "'");
  }
  return node[key].as<double>();
}

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

enum class BarrierType {
  kDpcbf,
  kEcbf,
};

struct QpBarrierConstraint {
  BarrierType type = BarrierType::kDpcbf;
  int row = 0;
  int obstacle_index = 0;
  std::array<double, 3> control_coefficients{};
  double lower_bound = 0.0;
  double decay_coefficient = 0.0;
};

}  // namespace

struct DpcbfSafetyFilter::Impl {
  bool enabled = true;
  bool dpcbf_enabled = true;
  bool ecbf_enabled = false;
  bool odcbf_enabled = false;
  bool slack_enabled = false;
  std::string base_body_name = "pelvis";
  double k_mu = 0.5;
  double k_lambda = 0.1;
  double robot_radius = 0.25;
  double detection_radius = 5.0;
  double sagittal_velocity_max = 2.0;
  double sagittal_velocity_min = -1.0;
  double lateral_velocity_max = 1.0;
  double lateral_velocity_min = -1.0;
  double yaw_rate_max = 1.0;
  double yaw_rate_min = -1.0;
  double sagittal_acceleration_max = 4.0;
  double sagittal_acceleration_min = -2.0;
  double lateral_acceleration_max = 2.0;
  double lateral_acceleration_min = -2.0;
  double safety_factor = 1.05;
  double eps_v = 0.05;
  double eps_d = 0.05;
  double k_a_s = 1.0;
  double k_a_l = 1.0;
  double alpha = 1.0;
  double alpha_1 = 1.0;
  double alpha_2 = 1.0;
  double decay_weight = 1.0e6;
  double slack_weight = 1.0e6;
  double reference_control_frequency_hz = 500.0;
  double velocity_increment_scale = 1.0;
  int obstacle_priority = 0;
  int max_constraints = 10;
  double dt = 0.0;
  bool initialized = false;
  osqp::OsqpSolver solver;
  std::array<double, 3> last_acceleration{};
  unsigned long long failure_count = 0;

  int MaxDpcbfRows() const {
    return dpcbf_enabled ? max_constraints : 0;
  }

  int MaxEcbfRows() const {
    return ecbf_enabled ? max_constraints : 0;
  }

  int MaxBarrierRows() const {
    return MaxDpcbfRows() + MaxEcbfRows();
  }

  int DecayVariableCount() const {
    return dpcbf_enabled && odcbf_enabled ? max_constraints : 0;
  }

  int SlackVariableCount() const {
    return slack_enabled ? MaxBarrierRows() : 0;
  }

  int NumVariables() const {
    return 3 + DecayVariableCount() + SlackVariableCount();
  }

  int DecayOffset() const { return 3; }

  int SlackOffset() const {
    return DecayOffset() + DecayVariableCount();
  }

  int ControlBoundRowOffset() const {
    return MaxBarrierRows();
  }

  int DecayBoundRowOffset() const {
    return ControlBoundRowOffset() + 5;
  }

  int SlackBoundRowOffset() const {
    return DecayBoundRowOffset() + DecayVariableCount();
  }

  int NumRows() const {
    return MaxBarrierRows() + 5 + DecayVariableCount() +
           SlackVariableCount();
  }

  Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int>
  MakeConstraintMatrix(
      const std::vector<QpBarrierConstraint>& constraints) const {
    using Triplet = Eigen::Triplet<double, osqp::c_int>;
    std::vector<const QpBarrierConstraint*> by_row(
        static_cast<std::size_t>(MaxBarrierRows()), nullptr);
    for (const auto& constraint : constraints) {
      by_row[static_cast<std::size_t>(constraint.row)] =
          &constraint;
    }
    std::vector<Triplet> entries;
    entries.reserve(static_cast<std::size_t>(
        3 * MaxBarrierRows() + DecayVariableCount() +
        SlackVariableCount() + 5 + DecayVariableCount() +
        SlackVariableCount()));
    for (int row = 0; row < MaxBarrierRows(); ++row) {
      const QpBarrierConstraint* constraint =
          by_row[static_cast<std::size_t>(row)];
      for (int col = 0; col < 3; ++col) {
        const double value =
            constraint
                ? constraint->control_coefficients[
                      static_cast<std::size_t>(col)]
                : 0.0;
        entries.emplace_back(row, col, value);
      }
      if (row < MaxDpcbfRows() && DecayVariableCount() > 0) {
        const double value =
            constraint ? constraint->decay_coefficient : 0.0;
        entries.emplace_back(row, DecayOffset() + row, value);
      }
      if (SlackVariableCount() > 0) {
        entries.emplace_back(row, SlackOffset() + row, 1.0);
      }
    }
    for (int col = 0; col < 3; ++col) {
      entries.emplace_back(ControlBoundRowOffset() + col, col, 1.0);
    }
    entries.emplace_back(ControlBoundRowOffset() + 3, 0, 1.0);
    entries.emplace_back(ControlBoundRowOffset() + 4, 1, 1.0);
    for (int index = 0; index < DecayVariableCount(); ++index) {
      entries.emplace_back(DecayBoundRowOffset() + index,
                           DecayOffset() + index, 1.0);
    }
    for (int index = 0; index < SlackVariableCount(); ++index) {
      entries.emplace_back(SlackBoundRowOffset() + index,
                           SlackOffset() + index, 1.0);
    }
    Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int> matrix(
        NumRows(), NumVariables());
    matrix.setFromTriplets(entries.begin(), entries.end());
    matrix.makeCompressed();
    return matrix;
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> MakeBounds(
      const RobotState& robot,
      const std::vector<QpBarrierConstraint>& constraints) const {
    const double infinity = std::numeric_limits<double>::infinity();
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(NumRows());
    Eigen::VectorXd upper = Eigen::VectorXd::Constant(NumRows(), infinity);
    for (const auto& constraint : constraints) {
      lower[constraint.row] = constraint.lower_bound;
    }
    lower[ControlBoundRowOffset() + 0] = sagittal_acceleration_min;
    upper[ControlBoundRowOffset() + 0] = sagittal_acceleration_max;
    lower[ControlBoundRowOffset() + 1] = lateral_acceleration_min;
    upper[ControlBoundRowOffset() + 1] = lateral_acceleration_max;
    lower[ControlBoundRowOffset() + 2] = yaw_rate_min;
    upper[ControlBoundRowOffset() + 2] = yaw_rate_max;
    lower[ControlBoundRowOffset() + 3] =
        (sagittal_velocity_min - robot.sagittal_velocity) /
        velocity_increment_scale;
    upper[ControlBoundRowOffset() + 3] =
        (sagittal_velocity_max - robot.sagittal_velocity) /
        velocity_increment_scale;
    lower[ControlBoundRowOffset() + 4] =
        (lateral_velocity_min - robot.lateral_velocity) /
        velocity_increment_scale;
    upper[ControlBoundRowOffset() + 4] =
        (lateral_velocity_max - robot.lateral_velocity) /
        velocity_increment_scale;
    for (int i = 0; i < DecayVariableCount(); ++i) {
      lower[DecayBoundRowOffset() + i] = 1.0;
    }
    for (int i = 0; i < SlackVariableCount(); ++i) {
      lower[SlackBoundRowOffset() + i] = 0.0;
    }
    return {std::move(lower), std::move(upper)};
  }
};

DpcbfSafetyFilter::DpcbfSafetyFilter() : impl_(std::make_unique<Impl>()) {}
DpcbfSafetyFilter::~DpcbfSafetyFilter() = default;
DpcbfSafetyFilter::DpcbfSafetyFilter(DpcbfSafetyFilter&&) noexcept = default;
DpcbfSafetyFilter& DpcbfSafetyFilter::operator=(DpcbfSafetyFilter&&) noexcept = default;

void DpcbfSafetyFilter::LoadConfig(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node robot = root["robot"];
  const YAML::Node qp = root["qp_parameters"];
  if (!robot || !qp) {
    throw std::runtime_error("missing 'robot' or 'qp_parameters' section");
  }

  impl_->enabled = qp["enabled"].as<bool>(true);
  impl_->dpcbf_enabled = qp["dpcbf_enabled"].as<bool>(true);
  impl_->ecbf_enabled = qp["ecbf_enabled"].as<bool>(false);
  impl_->odcbf_enabled = qp["odcbf_enabled"].as<bool>(false);
  impl_->slack_enabled = qp["slack_enabled"].as<bool>(false);
  impl_->base_body_name = robot["base_body"].as<std::string>("pelvis");
  impl_->k_mu = ReadDouble(robot, "k_mu");
  impl_->k_lambda = ReadDouble(robot, "k_lambda");
  impl_->robot_radius = ReadDouble(robot, "r_rob");
  impl_->detection_radius = ReadDouble(robot, "p_max");
  impl_->sagittal_velocity_max = ReadDouble(robot, "v_s_max");
  impl_->sagittal_velocity_min = ReadDouble(robot, "v_s_min");
  impl_->lateral_velocity_max = ReadDouble(robot, "v_l_max");
  impl_->lateral_velocity_min = ReadDouble(robot, "v_l_min");
  impl_->yaw_rate_max = ReadDouble(robot, "w_max");
  impl_->yaw_rate_min = ReadDouble(robot, "w_min");
  impl_->sagittal_acceleration_max = ReadDouble(robot, "a_s_max");
  impl_->sagittal_acceleration_min = ReadDouble(robot, "a_s_min");
  impl_->lateral_acceleration_max = ReadDouble(robot, "a_l_max");
  impl_->lateral_acceleration_min = ReadDouble(robot, "a_l_min");
  impl_->safety_factor = ReadDouble(robot, "s");
  impl_->eps_v = ReadDouble(robot, "eps_v");
  impl_->eps_d = ReadDouble(robot, "eps_d");
  impl_->k_a_s = ReadDouble(qp, "k_a_s");
  impl_->k_a_l = ReadDouble(qp, "k_a_l");
  impl_->alpha = ReadDouble(qp, "alpha");
  if (qp["alpha_ecbf"]) {
    impl_->alpha_1 = qp["alpha_ecbf"].as<double>();
    impl_->alpha_2 = impl_->alpha_1;
  } else {
    impl_->alpha_1 = qp["alpha_1"].as<double>(1.0);
    impl_->alpha_2 = qp["alpha_2"].as<double>(impl_->alpha_1);
  }
  impl_->decay_weight = qp["decay_weight"].as<double>(1.0e6);
  impl_->slack_weight = qp["slack_weight"].as<double>(1.0e6);
  impl_->reference_control_frequency_hz =
      qp["reference_control_frequency_hz"].as<double>(500.0);
  impl_->obstacle_priority = qp["obstacle_priority"].as<int>(0);
  impl_->max_constraints = qp["default_num_constraints"].as<int>();

  const auto ordered = [](double lower, double upper) { return lower <= upper; };
  if (impl_->k_mu < 0.0 || impl_->k_lambda < 0.0 || impl_->robot_radius <= 0.0 ||
      impl_->detection_radius <= 0.0 || impl_->safety_factor <= 1.0 ||
      impl_->eps_v <= 0.0 || impl_->eps_d <= 0.0 || impl_->k_a_s < 0.0 ||
      impl_->k_a_l < 0.0 || impl_->alpha <= 0.0 || impl_->max_constraints <= 0 ||
      impl_->alpha_1 <= 0.0 || impl_->alpha_2 <= 0.0 ||
      impl_->decay_weight <= 0.0 || impl_->slack_weight <= 0.0 ||
      impl_->reference_control_frequency_hz <= 0.0 ||
      (impl_->obstacle_priority != 0 && impl_->obstacle_priority != 1) ||
      !ordered(impl_->sagittal_velocity_min, impl_->sagittal_velocity_max) ||
      !ordered(impl_->lateral_velocity_min, impl_->lateral_velocity_max) ||
      !ordered(impl_->yaw_rate_min, impl_->yaw_rate_max) ||
      !ordered(impl_->sagittal_acceleration_min, impl_->sagittal_acceleration_max) ||
      !ordered(impl_->lateral_acceleration_min, impl_->lateral_acceleration_max)) {
    throw std::runtime_error("invalid robot or QP parameter range");
  }
  impl_->initialized = false;
  std::cout << "Loaded DPCBF-QP configuration from " << path << '\n';
}

void DpcbfSafetyFilter::Initialize(double control_dt) {
  if (control_dt <= 0.0 || !std::isfinite(control_dt)) {
    throw std::runtime_error("DPCBF control dt must be positive and finite");
  }
  impl_->dt = control_dt;
  impl_->velocity_increment_scale =
      control_dt * impl_->reference_control_frequency_hz;
  if (!impl_->enabled ||
      (!impl_->dpcbf_enabled && !impl_->ecbf_enabled)) {
    impl_->initialized = true;
    return;
  }

  osqp::OsqpInstance instance;
  instance.objective_matrix.resize(impl_->NumVariables(),
                                   impl_->NumVariables());
  for (int index = 0; index < 3; ++index) {
    instance.objective_matrix.insert(index, index) = 2.0;
  }
  for (int index = 0; index < impl_->DecayVariableCount(); ++index) {
    const int variable = impl_->DecayOffset() + index;
    instance.objective_matrix.insert(variable, variable) =
        2.0 * impl_->decay_weight;
  }
  for (int index = 0; index < impl_->SlackVariableCount(); ++index) {
    const int variable = impl_->SlackOffset() + index;
    instance.objective_matrix.insert(variable, variable) =
        2.0 * impl_->slack_weight;
  }
  instance.objective_matrix.makeCompressed();
  instance.objective_vector =
      Eigen::VectorXd::Zero(impl_->NumVariables());
  for (int index = 0; index < impl_->DecayVariableCount(); ++index) {
    instance.objective_vector[impl_->DecayOffset() + index] =
        -2.0 * impl_->decay_weight;
  }
  instance.constraint_matrix = impl_->MakeConstraintMatrix({});
  const RobotState zero_state;
  std::tie(instance.lower_bounds, instance.upper_bounds) =
      impl_->MakeBounds(zero_state, {});

  osqp::OsqpSettings settings;
  settings.verbose = false;
  settings.warm_start = true;
  settings.polish = true;
  settings.eps_abs = 1.0e-5;
  settings.eps_rel = 1.0e-5;
  settings.max_iter = 2000;
  const absl::Status status = impl_->solver.Init(instance, settings);
  if (!status.ok()) {
    throw std::runtime_error("failed to initialize OSQP: " + status.ToString());
  }
  impl_->initialized = true;
  std::cout << "Initialized OSQP safety filter at dt=" << impl_->dt
            << " s, velocity increment scale="
            << impl_->velocity_increment_scale
            << ", variables=" << impl_->NumVariables()
            << ", rows=" << impl_->NumRows()
            << ", DPCBF=" << impl_->dpcbf_enabled
            << ", eCBF=" << impl_->ecbf_enabled
            << ", ODCBF=" << impl_->odcbf_enabled
            << ", slack=" << impl_->slack_enabled << '\n';
}

DpcbfConstraint DpcbfSafetyFilter::EvaluateConstraint(
const RobotState& robot, const ObstacleState& obstacle) const {
  const double px = obstacle.x - robot.x;
  const double py = obstacle.y - robot.y;
  const double p_squared = std::max(px * px + py * py, kMinimumDenominator);

  const double alpha_angle = std::atan2(py, px);
  const double cos_alpha = std::cos(alpha_angle);
  const double sin_alpha = std::sin(alpha_angle);

  const double robot_velocity_x =
      robot.sagittal_velocity * std::cos(robot.phi) -
      robot.lateral_velocity * std::sin(robot.phi);
  const double robot_velocity_y =
      robot.sagittal_velocity * std::sin(robot.phi) +
      robot.lateral_velocity * std::cos(robot.phi);
  
  const double relative_velocity_x = obstacle.velocity_x - robot_velocity_x;
  const double relative_velocity_y = obstacle.velocity_y - robot_velocity_y;
  const double x_tilde = cos_alpha * relative_velocity_x + sin_alpha * relative_velocity_y;
  const double y_tilde = -sin_alpha * relative_velocity_x + cos_alpha * relative_velocity_y;
  const double velocity = std::sqrt(relative_velocity_x * relative_velocity_x +
                               relative_velocity_y * relative_velocity_y);
  const double safe_velocity = std::sqrt(relative_velocity_x * relative_velocity_x +
                                         relative_velocity_y * relative_velocity_y +
                                         impl_->eps_v * impl_->eps_v);

  const double combined_radius = impl_->robot_radius + obstacle.radius;
  const double safe_radius = combined_radius * impl_->safety_factor;
  const double smooth_clearance = std::sqrt(std::max(
      p_squared - safe_radius * safe_radius + impl_->eps_d * impl_->eps_d,
      kMinimumDenominator));
  const double safe_distance = smooth_clearance - impl_->eps_d;
  const double adaptive_scale =
      std::sqrt(impl_->safety_factor * impl_->safety_factor - 1.0) / safe_radius;
  const double lambda_unscaled = impl_->k_lambda * safe_distance / safe_velocity;

  DpcbfConstraint constraint;
  constraint.h = x_tilde + adaptive_scale *
      (lambda_unscaled * y_tilde * y_tilde + impl_->k_mu * safe_distance);
  
  const double position_term =
      impl_->k_lambda * y_tilde * y_tilde / safe_velocity + impl_->k_mu;
  
  const double dh_dx = py * y_tilde / p_squared - adaptive_scale *
      (2.0 * lambda_unscaled * x_tilde * y_tilde * py / p_squared +
       px * position_term / smooth_clearance);
  const double dh_dy = -px * y_tilde / p_squared + adaptive_scale *
      (2.0 * lambda_unscaled * x_tilde * y_tilde * px / p_squared -
       py * position_term / smooth_clearance);

  const double phi_tilde = robot.phi - alpha_angle;
  const double cosine = std::cos(phi_tilde);
  const double sine = std::sin(phi_tilde);
  const double v1 = robot.sagittal_velocity * sine + robot.lateral_velocity * cosine;
  const double v2 = robot.sagittal_velocity * cosine - robot.lateral_velocity * sine;
  const double velocity_cubed = safe_velocity * safe_velocity * safe_velocity;
  const double common_x = impl_->k_lambda * safe_distance * x_tilde * y_tilde * y_tilde / velocity_cubed;
  const double common_y = impl_->k_lambda * safe_distance * (2.0 * y_tilde / safe_velocity - y_tilde * y_tilde * y_tilde / velocity_cubed);

  const double dh_dphi = v1 - adaptive_scale * (v1 * common_x + v2 * common_y);
  const double dh_dvs = -cosine + adaptive_scale * (cosine * common_x - sine * common_y);
  const double dh_dvl = sine - adaptive_scale * (sine * common_x + cosine * common_y);

  constraint.lg_h = {dh_dvs, dh_dvl, dh_dphi};
  // The obstacle-position drift term is required for moving obstacles because
  // p_rel = p_obs - p_robot.
  constraint.lf_h = dh_dx * (robot_velocity_x - obstacle.velocity_x) +
                    dh_dy * (robot_velocity_y - obstacle.velocity_y);
  return constraint;
}

EcbfConstraint DpcbfSafetyFilter::EvaluateEcbfConstraint(
    const RobotState& robot, const ObstacleState& obstacle) const {
  const double px = obstacle.x - robot.x;
  const double py = obstacle.y - robot.y;
  const double cosine = std::cos(robot.phi);
  const double sine = std::sin(robot.phi);
  const double robot_velocity_x =
      robot.sagittal_velocity * cosine -
      robot.lateral_velocity * sine;
  const double robot_velocity_y =
      robot.sagittal_velocity * sine +
      robot.lateral_velocity * cosine;
  const double relative_velocity_x =
      obstacle.velocity_x - robot_velocity_x;
  const double relative_velocity_y =
      obstacle.velocity_y - robot_velocity_y;
  const double relative_speed_squared =
      relative_velocity_x * relative_velocity_x +
      relative_velocity_y * relative_velocity_y;

  const double combined_radius = impl_->robot_radius + obstacle.radius;
  const double distance_barrier =
      px * px + py * py - combined_radius * combined_radius;
  const double distance_barrier_dot =
      2.0 * (px * relative_velocity_x +
             py * relative_velocity_y);
  const double p_s = px * cosine + py * sine;
  const double p_l = -px * sine + py * cosine;

  EcbfConstraint constraint;
  constraint.distance_barrier = distance_barrier;
  constraint.distance_barrier_dot = distance_barrier_dot;
  constraint.control_coefficients = {
      -2.0 * p_s,
      -2.0 * p_l,
      2.0 * (p_s * robot.lateral_velocity -
             p_l * robot.sagittal_velocity)};
  constraint.drift =
      2.0 * relative_speed_squared +
      (impl_->alpha_1 + impl_->alpha_2) * distance_barrier_dot +
      impl_->alpha_1 * impl_->alpha_2 * distance_barrier;
  return constraint;
}

SafetyFilterResult DpcbfSafetyFilter::Filter(
    const RobotState& robot, const VelocityCommand& desired,
    const std::vector<ObstacleState>& obstacles) {
  if (!impl_->initialized) {
    throw std::runtime_error("DPCBF safety filter has not been initialized");
  }

  SafetyFilterResult result;
  const double robot_velocity_x =
      robot.sagittal_velocity * std::cos(robot.phi) -
      robot.lateral_velocity * std::sin(robot.phi);
  const double robot_velocity_y =
      robot.sagittal_velocity * std::sin(robot.phi) +
      robot.lateral_velocity * std::cos(robot.phi);
  struct ObstacleCandidate {
    double distance = 0.0;
    double closing_alignment = 0.0;
    const ObstacleState* obstacle = nullptr;
  };
  std::vector<ObstacleCandidate> nearest;
  nearest.reserve(obstacles.size());
  for (const ObstacleState& obstacle : obstacles) {
    const double dx = obstacle.x - robot.x;
    const double dy = obstacle.y - robot.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= impl_->detection_radius) {
      const double relative_velocity_x =
          obstacle.velocity_x - robot_velocity_x;
      const double relative_velocity_y =
          obstacle.velocity_y - robot_velocity_y;
      const double relative_speed =
          std::hypot(relative_velocity_x, relative_velocity_y);
      double closing_alignment = 0.0;
      if (relative_speed > kMinimumDenominator &&
          distance > kMinimumDenominator) {
        closing_alignment =
            -(relative_velocity_x * dx +
              relative_velocity_y * dy) /
            (relative_speed * distance);
        closing_alignment =
            Clamp(closing_alignment, -1.0, 1.0);
      }
      nearest.push_back(
          {distance, closing_alignment, &obstacle});
    }
  }
  std::sort(
      nearest.begin(), nearest.end(),
      [this](const ObstacleCandidate& left,
             const ObstacleCandidate& right) {
        if (impl_->obstacle_priority == 1 &&
            std::abs(left.closing_alignment -
                     right.closing_alignment) > 1.0e-12) {
          return left.closing_alignment >
                 right.closing_alignment;
        }
        return left.distance < right.distance;
      });
  if (nearest.size() > static_cast<std::size_t>(impl_->max_constraints)) {
    nearest.resize(static_cast<std::size_t>(impl_->max_constraints));
  }

  std::vector<DpcbfConstraint> dpcbf_constraints;
  dpcbf_constraints.reserve(nearest.size());
  for (const auto& item : nearest) {
    dpcbf_constraints.push_back(
        EvaluateConstraint(robot, *item.obstacle));
  }
  result.active_constraints = static_cast<int>(nearest.size());
  result.active_dpcbf_constraints =
      impl_->enabled && impl_->dpcbf_enabled
          ? static_cast<int>(nearest.size())
          : 0;
  result.active_ecbf_constraints =
      impl_->enabled && impl_->ecbf_enabled
          ? static_cast<int>(nearest.size())
          : 0;

  result.selected_obstacles.reserve(nearest.size());
  for (std::size_t i = 0; i < nearest.size(); ++i) {
    const ObstacleState& obstacle = *nearest[i].obstacle;
    const double px = obstacle.x - robot.x;
    const double py = obstacle.y - robot.y;
    const double los_angle = std::atan2(py, px);
    const double cosine = std::cos(los_angle);
    const double sine = std::sin(los_angle);
    const double relative_x = obstacle.velocity_x - robot_velocity_x;
    const double relative_y = obstacle.velocity_y - robot_velocity_y;
    const double x_tilde = cosine * relative_x + sine * relative_y;
    const double y_tilde = -sine * relative_x + cosine * relative_y;

    const double safe_velocity =
        std::sqrt(relative_x * relative_x +
                  relative_y * relative_y +
                  impl_->eps_v * impl_->eps_v);
    const double safe_radius = (impl_->robot_radius + obstacle.radius) * impl_->safety_factor;
    const double smooth_clearance = std::sqrt(
        std::max(px * px + py * py - safe_radius * safe_radius + impl_->eps_d * impl_->eps_d,
        kMinimumDenominator));
    const double safe_distance = smooth_clearance - impl_->eps_d;
    const double adaptive_scale = std::sqrt(impl_->safety_factor * impl_->safety_factor -1.0) / safe_radius;

    DpcbfVisualizationObstacle visual;
    visual.obstacle = obstacle;
    visual.constraint = dpcbf_constraints[i];
    visual.distance = nearest[i].distance;
    visual.relative_velocity_world = {relative_x, relative_y};
    visual.relative_velocity_los = {x_tilde, y_tilde};
    visual.boundary_vertex_x = -adaptive_scale * impl_->k_mu * safe_distance;
    visual.boundary_curvature = adaptive_scale * impl_->k_lambda * safe_distance / safe_velocity;
    result.selected_obstacles.push_back(visual);
  }

  if (!impl_->enabled ||
      (!impl_->dpcbf_enabled && !impl_->ecbf_enabled)) {
    result.command = desired;
    result.solved = true;
    return result;
  }

  std::vector<QpBarrierConstraint> qp_constraints;
  qp_constraints.reserve(
      static_cast<std::size_t>(impl_->MaxBarrierRows()));
  if (impl_->dpcbf_enabled) {
    for (std::size_t i = 0; i < dpcbf_constraints.size(); ++i) {
      const DpcbfConstraint& constraint = dpcbf_constraints[i];
      QpBarrierConstraint qp_constraint;
      qp_constraint.type = BarrierType::kDpcbf;
      qp_constraint.row = static_cast<int>(i);
      qp_constraint.obstacle_index = static_cast<int>(i);
      qp_constraint.control_coefficients = constraint.lg_h;
      if (impl_->odcbf_enabled) {
        qp_constraint.lower_bound = -constraint.lf_h;
        qp_constraint.decay_coefficient =
            impl_->alpha * constraint.h;
      } else {
        qp_constraint.lower_bound =
            -(constraint.lf_h +
              impl_->alpha * constraint.h);
      }
      qp_constraints.push_back(qp_constraint);
    }
  }
  if (impl_->ecbf_enabled) {
    for (std::size_t i = 0; i < nearest.size(); ++i) {
      const EcbfConstraint constraint =
          EvaluateEcbfConstraint(robot, *nearest[i].obstacle);
      QpBarrierConstraint qp_constraint;
      qp_constraint.type = BarrierType::kEcbf;
      qp_constraint.row =
          impl_->MaxDpcbfRows() + static_cast<int>(i);
      qp_constraint.obstacle_index = static_cast<int>(i);
      qp_constraint.control_coefficients =
          constraint.control_coefficients;
      qp_constraint.lower_bound = -constraint.drift;
      qp_constraints.push_back(qp_constraint);
    }
  }

  const double sagittal_reference = Clamp(
      impl_->k_a_s * (desired.sagittal - robot.sagittal_velocity),
      impl_->sagittal_acceleration_min, impl_->sagittal_acceleration_max);
  const double lateral_reference = Clamp(
      impl_->k_a_l * (desired.lateral - robot.lateral_velocity),
      impl_->lateral_acceleration_min, impl_->lateral_acceleration_max);
  const double yaw_reference = Clamp(desired.yaw_rate,
                                     impl_->yaw_rate_min, impl_->yaw_rate_max);
  Eigen::VectorXd objective =
      Eigen::VectorXd::Zero(impl_->NumVariables());
  objective[0] = -2.0 * sagittal_reference;
  objective[1] = -2.0 * lateral_reference;
  objective[2] = -2.0 * yaw_reference;
  for (int index = 0; index < impl_->DecayVariableCount();
       ++index) {
    objective[impl_->DecayOffset() + index] =
        -2.0 * impl_->decay_weight;
  }

  const auto matrix =
      impl_->MakeConstraintMatrix(qp_constraints);
  auto [lower, upper] =
      impl_->MakeBounds(robot, qp_constraints);
  const absl::Status matrix_status = impl_->solver.UpdateConstraintMatrix(matrix);
  const absl::Status objective_status = impl_->solver.SetObjectiveVector(objective);
  const absl::Status bounds_status = impl_->solver.SetBounds(lower, upper);
  if (!matrix_status.ok() || !objective_status.ok() || !bounds_status.ok()) {
    if ((impl_->failure_count++ % 500) == 0) {
      std::cerr << "DPCBF OSQP update failed; holding last feasible command\n";
    }
  } else {
    const osqp::OsqpExitCode exit_code = impl_->solver.Solve();
    if (exit_code == osqp::OsqpExitCode::kOptimal ||
        exit_code == osqp::OsqpExitCode::kOptimalInaccurate) {
      const Eigen::VectorXd solution = impl_->solver.primal_solution();
      impl_->last_acceleration = {solution[0], solution[1], solution[2]};
      if (impl_->DecayVariableCount() > 0) {
        result.decay_variables.reserve(nearest.size());
        for (std::size_t i = 0; i < nearest.size(); ++i) {
          result.decay_variables.push_back(
              solution[impl_->DecayOffset() +
                       static_cast<int>(i)]);
        }
      }
      if (impl_->SlackVariableCount() > 0) {
        result.slack_variables.reserve(qp_constraints.size());
        for (const auto& constraint : qp_constraints) {
          result.slack_variables.push_back(
              solution[impl_->SlackOffset() +
                       constraint.row]);
        }
      }
      result.solved = true;
    } else {
      if ((impl_->failure_count++ % 500) == 0) {
        std::cerr << "DPCBF OSQP solve failed (" << osqp::ToString(exit_code)
                  << "); holding last feasible command\n";
        // std::cout << "last sagittal acceleration: " << impl_->last_acceleration[0] << '\n'
        //           << "last lateral acceleration: " << impl_->last_acceleration[1] << '\n'
        //           << "last yaw rate: " << impl_->last_acceleration[2] << '\n';
      }
    }
  }

  result.acceleration = impl_->last_acceleration;
  result.command.sagittal = Clamp(
      robot.sagittal_velocity +
          result.acceleration[0] * impl_->velocity_increment_scale,
      impl_->sagittal_velocity_min, impl_->sagittal_velocity_max);
  result.command.lateral = Clamp(
      robot.lateral_velocity +
          result.acceleration[1] * impl_->velocity_increment_scale,
      impl_->lateral_velocity_min, impl_->lateral_velocity_max);
  result.command.yaw_rate = Clamp(result.acceleration[2],
                                  impl_->yaw_rate_min, impl_->yaw_rate_max);
  return result;
}

bool DpcbfSafetyFilter::enabled() const { return impl_->enabled; }
double DpcbfSafetyFilter::sagittal_velocity_min() const { return impl_->sagittal_velocity_min; }
double DpcbfSafetyFilter::sagittal_velocity_max() const { return impl_->sagittal_velocity_max; }
double DpcbfSafetyFilter::lateral_velocity_min() const { return impl_->lateral_velocity_min; }
double DpcbfSafetyFilter::lateral_velocity_max() const { return impl_->lateral_velocity_max; }
double DpcbfSafetyFilter::yaw_rate_min() const { return impl_->yaw_rate_min; }
double DpcbfSafetyFilter::yaw_rate_max() const { return impl_->yaw_rate_max; }
const std::string& DpcbfSafetyFilter::base_body_name() const {
  return impl_->base_body_name;
}

}  // namespace dpcbf
