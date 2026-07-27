#include "dpcbf/dpcbf_safety_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double WrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double ReadPositive(const YAML::Node& node, const char* key) {
  const double value = node[key].as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(std::string(key) + " must be positive and finite");
  }
  return value;
}

std::array<double, 2> ReadRange(const YAML::Node& node, const char* key) {
  const YAML::Node range = node[key];
  if (!range || !range.IsSequence() || range.size() != 2) {
    throw std::runtime_error(std::string(key) + " must contain [min, max]");
  }
  const std::array<double, 2> values{range[0].as<double>(),
                                     range[1].as<double>()};
  if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
      values[0] > values[1]) {
    throw std::runtime_error(std::string(key) + " has an invalid range");
  }
  return values;
}

std::array<double, 2> ReadPair(const YAML::Node& node, const char* key) {
  const YAML::Node pair = node[key];
  if (!pair || !pair.IsSequence() || pair.size() != 2) {
    throw std::runtime_error(std::string(key) + " must contain two values");
  }
  const std::array<double, 2> values{pair[0].as<double>(),
                                     pair[1].as<double>()};
  if (!std::isfinite(values[0]) || !std::isfinite(values[1])) {
    throw std::runtime_error(std::string(key) + " must be finite");
  }
  return values;
}

struct EvaluationConfig {
  int rollout_start_index = 0;
  int rollouts = 1;
  int max_steps = 1;
  std::uint64_t seed = 42;
  double dt = 0.01;
  double fixed_command_fraction = 0.3;
  double perlin_frequency_hz = 0.12;
  double perlin_velocity_amplitude = 0.35;
  double perlin_yaw_amplitude = 0.35;
  double waypoint_speed = 1.0;
  double waypoint_reached_radius = 0.4;
  double boundary_command_margin = 1.0;

  int obstacle_count = 5;
  std::array<double, 2> obstacle_radius_range{0.1, 0.5};
  std::array<double, 2> obstacle_speed_range{0.0, 0.6};

  double arena_width = 10.0;
  double arena_height = 10.0;
  double arena_center_x = 0.0;
  double arena_center_y = 0.0;

  double robot_radius = 0.25;
  double spawn_clearance = 0.5;
  double v_s_min = -1.0;
  double v_s_max = 2.0;
  double v_l_min = -1.0;
  double v_l_max = 1.0;
  double w_min = -1.0;
  double w_max = 1.0;
  double a_s_min = -2.0;
  double a_s_max = 4.0;
  double a_l_min = -2.0;
  double a_l_max = 2.0;
  double k_a_s = 1.0;
  double k_a_l = 1.0;
};

EvaluationConfig LoadEvaluationConfig(const YAML::Node& root) {
  EvaluationConfig config;
  const YAML::Node simulation = root["optimization_simulation"];
  const YAML::Node obstacles = root["dynamic_obstacles"];
  const YAML::Node arena = obstacles["arena"];
  const YAML::Node robot = root["robot"];
  const YAML::Node qp = root["qp_parameters"];
  if (!simulation || !obstacles || !arena || !robot || !qp) {
    throw std::runtime_error(
        "optimization_simulation, dynamic_obstacles.arena, robot, and "
        "qp_parameters are required");
  }

  config.rollout_start_index =
      simulation["rollout_start_index"].as<int>(0);
  config.rollouts = simulation["rollouts"].as<int>();
  config.dt = ReadPositive(simulation, "control_dt");
  const double duration = ReadPositive(simulation, "episode_duration_s");
  config.max_steps = static_cast<int>(std::ceil(duration / config.dt));
  config.seed = simulation["seed"].as<std::uint64_t>(42);
  config.fixed_command_fraction =
      simulation["fixed_command_fraction"].as<double>(0.3);
  config.perlin_frequency_hz =
      ReadPositive(simulation, "perlin_frequency_hz");
  config.perlin_velocity_amplitude =
      simulation["perlin_velocity_amplitude"].as<double>(0.35);
  config.perlin_yaw_amplitude =
      simulation["perlin_yaw_amplitude"].as<double>(0.35);
  config.waypoint_speed = ReadPositive(simulation, "waypoint_speed");
  config.waypoint_reached_radius =
      ReadPositive(simulation, "waypoint_reached_radius");
  config.boundary_command_margin =
      ReadPositive(simulation, "boundary_command_margin");

  config.obstacle_count = obstacles["count"].as<int>();
  config.obstacle_radius_range = ReadRange(obstacles, "radius_range");
  config.obstacle_speed_range = ReadRange(obstacles, "speed_range");
  const auto arena_size = ReadPair(arena, "size");
  config.arena_width = arena_size[0];
  config.arena_height = arena_size[1];
  const auto arena_center = ReadPair(arena, "center");
  config.arena_center_x = arena_center[0];
  config.arena_center_y = arena_center[1];

  config.robot_radius = ReadPositive(robot, "r_rob");
  config.spawn_clearance =
      robot["spawn_clearance"].as<double>(0.5);
  config.v_s_max = robot["v_s_max"].as<double>();
  config.v_s_min = robot["v_s_min"].as<double>();
  config.v_l_max = robot["v_l_max"].as<double>();
  config.v_l_min = robot["v_l_min"].as<double>();
  config.w_max = robot["w_max"].as<double>();
  config.w_min = robot["w_min"].as<double>();
  config.a_s_max = robot["a_s_max"].as<double>();
  config.a_s_min = robot["a_s_min"].as<double>();
  config.a_l_max = robot["a_l_max"].as<double>();
  config.a_l_min = robot["a_l_min"].as<double>();
  config.k_a_s = qp["k_a_s"].as<double>();
  config.k_a_l = qp["k_a_l"].as<double>();

  if (config.rollout_start_index < 0 || config.rollouts <= 0 ||
      config.max_steps <= 0 || config.obstacle_count <= 0 ||
      config.fixed_command_fraction < 0.0 ||
      config.fixed_command_fraction > 1.0 ||
      config.perlin_velocity_amplitude < 0.0 ||
      config.perlin_yaw_amplitude < 0.0 ||
      config.spawn_clearance < 0.0 ||
      config.arena_width <= 2.0 * config.robot_radius ||
      config.arena_height <= 2.0 * config.robot_radius) {
    throw std::runtime_error("invalid optimization simulation configuration");
  }
  return config;
}

class PerlinNoise1D {
 public:
  explicit PerlinNoise1D(std::uint64_t seed) : seed_(seed) {}

  double Evaluate(double x) const {
    const auto left = static_cast<std::int64_t>(std::floor(x));
    const double fraction = x - static_cast<double>(left);
    const double fade =
        fraction * fraction * fraction *
        (fraction * (fraction * 6.0 - 15.0) + 10.0);
    const double a = Gradient(left) * fraction;
    const double b = Gradient(left + 1) * (fraction - 1.0);
    return 2.0 * (a + fade * (b - a));
  }

 private:
  double Gradient(std::int64_t index) const {
    std::uint64_t value = static_cast<std::uint64_t>(index) +
                          seed_ + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return (value & 1ULL) == 0ULL ? -1.0 : 1.0;
  }

  std::uint64_t seed_;
};

double Uniform(std::mt19937_64& generator, double lower, double upper) {
  return std::uniform_real_distribution<double>(lower, upper)(generator);
}

struct CommandGenerator {
  CommandGenerator(const EvaluationConfig& config, std::mt19937_64& generator,
                   int rollout_index)
      : config(config),
        generator(generator),
        sagittal_noise(generator()),
        lateral_noise(generator()),
        yaw_noise(generator()) {
    fixed = Uniform(generator, 0.0, 1.0) <
            config.fixed_command_fraction;
    fixed_index = rollout_index % static_cast<int>(FixedCommands().size());
    SampleWaypoint();
  }

  dpcbf::VelocityCommand Next(const dpcbf::RobotState& robot,
                              double time) {
    const double left = config.arena_center_x - 0.5 * config.arena_width;
    const double right = config.arena_center_x + 0.5 * config.arena_width;
    const double bottom = config.arena_center_y - 0.5 * config.arena_height;
    const double top = config.arena_center_y + 0.5 * config.arena_height;
    const bool near_boundary =
        robot.x - left < config.boundary_command_margin ||
        right - robot.x < config.boundary_command_margin ||
        robot.y - bottom < config.boundary_command_margin ||
        top - robot.y < config.boundary_command_margin;

    if (fixed && !near_boundary) {
      return FixedCommands()[static_cast<std::size_t>(fixed_index)];
    }
    if (near_boundary) {
      waypoint_x = config.arena_center_x;
      waypoint_y = config.arena_center_y;
    } else if (std::hypot(waypoint_x - robot.x, waypoint_y - robot.y) <
               config.waypoint_reached_radius) {
      SampleWaypoint();
    }

    const double dx = waypoint_x - robot.x;
    const double dy = waypoint_y - robot.y;
    const double distance = std::max(std::hypot(dx, dy), 1.0e-9);
    const double speed = std::min(config.waypoint_speed, distance);
    const double global_vx = speed * dx / distance;
    const double global_vy = speed * dy / distance;
    const double cosine = std::cos(robot.phi);
    const double sine = std::sin(robot.phi);
    const double noise_x =
        config.perlin_frequency_hz * time + 0.17;
    const double noise_y =
        config.perlin_frequency_hz * time + 19.31;
    const double noise_w =
        config.perlin_frequency_hz * time + 47.83;

    dpcbf::VelocityCommand command;
    command.sagittal =
        cosine * global_vx + sine * global_vy +
        config.perlin_velocity_amplitude *
            sagittal_noise.Evaluate(noise_x);
    command.lateral =
        -sine * global_vx + cosine * global_vy +
        config.perlin_velocity_amplitude *
            lateral_noise.Evaluate(noise_y);
    const double desired_heading = std::atan2(dy, dx);
    command.yaw_rate =
        1.2 * WrapAngle(desired_heading - robot.phi) +
        config.perlin_yaw_amplitude * yaw_noise.Evaluate(noise_w);
    command.sagittal =
        Clamp(command.sagittal, config.v_s_min, config.v_s_max);
    command.lateral =
        Clamp(command.lateral, config.v_l_min, config.v_l_max);
    command.yaw_rate =
        Clamp(command.yaw_rate, config.w_min, config.w_max);
    return command;
  }

 private:
  std::array<dpcbf::VelocityCommand, 8> FixedCommands() const {
    return {{
        {0.65 * config.v_s_max, 0.0, 0.0},
        {0.55 * config.v_s_min, 0.0, 0.0},
        {0.0, 0.65 * config.v_l_max, 0.0},
        {0.0, 0.65 * config.v_l_min, 0.0},
        {0.45 * config.v_s_max, 0.45 * config.v_l_max, 0.0},
        {0.45 * config.v_s_max, 0.45 * config.v_l_min, 0.0},
        {0.25 * config.v_s_max, 0.0, 0.65 * config.w_max},
        {0.25 * config.v_s_max, 0.0, 0.65 * config.w_min},
    }};
  }

  void SampleWaypoint() {
    const double x_margin =
        config.robot_radius + config.boundary_command_margin;
    const double y_margin =
        config.robot_radius + config.boundary_command_margin;
    waypoint_x = Uniform(
        generator, config.arena_center_x - 0.5 * config.arena_width + x_margin,
        config.arena_center_x + 0.5 * config.arena_width - x_margin);
    waypoint_y = Uniform(
        generator, config.arena_center_y - 0.5 * config.arena_height + y_margin,
        config.arena_center_y + 0.5 * config.arena_height - y_margin);
  }

  const EvaluationConfig& config;
  std::mt19937_64& generator;
  PerlinNoise1D sagittal_noise;
  PerlinNoise1D lateral_noise;
  PerlinNoise1D yaw_noise;
  bool fixed = false;
  int fixed_index = 0;
  double waypoint_x = 0.0;
  double waypoint_y = 0.0;
};

std::vector<dpcbf::ObstacleState> SpawnObstacles(
    const EvaluationConfig& config, std::mt19937_64& generator) {
  std::vector<dpcbf::ObstacleState> obstacles;
  obstacles.reserve(static_cast<std::size_t>(config.obstacle_count));
  for (int id = 0; id < config.obstacle_count; ++id) {
    bool placed = false;
    for (int attempt = 0; attempt < 20000 && !placed; ++attempt) {
      dpcbf::ObstacleState obstacle;
      obstacle.id = id;
      obstacle.radius =
          Uniform(generator, config.obstacle_radius_range[0],
                  config.obstacle_radius_range[1]);
      obstacle.x = Uniform(
          generator,
          config.arena_center_x - 0.5 * config.arena_width + obstacle.radius,
          config.arena_center_x + 0.5 * config.arena_width - obstacle.radius);
      obstacle.y = Uniform(
          generator,
          config.arena_center_y - 0.5 * config.arena_height + obstacle.radius,
          config.arena_center_y + 0.5 * config.arena_height - obstacle.radius);
      const double robot_distance =
          std::hypot(obstacle.x - config.arena_center_x,
                     obstacle.y - config.arena_center_y);
      if (robot_distance <
          config.robot_radius + obstacle.radius + config.spawn_clearance) {
        continue;
      }
      bool overlaps = false;
      for (const auto& existing : obstacles) {
        if (std::hypot(obstacle.x - existing.x,
                       obstacle.y - existing.y) <
            obstacle.radius + existing.radius) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) {
        continue;
      }
      const double speed =
          Uniform(generator, config.obstacle_speed_range[0],
                  config.obstacle_speed_range[1]);
      const double angle = Uniform(generator, -kPi, kPi);
      obstacle.velocity_x = speed * std::cos(angle);
      obstacle.velocity_y = speed * std::sin(angle);
      obstacles.push_back(obstacle);
      placed = true;
    }
    if (!placed) {
      throw std::runtime_error(
          "could not place all obstacles; reduce count/radii/spawn_clearance");
    }
  }
  return obstacles;
}

void ReflectCoordinate(double lower, double upper, double dt,
                       double& position, double& velocity) {
  position += velocity * dt;
  while (position < lower || position > upper) {
    if (position < lower) {
      position = 2.0 * lower - position;
      velocity = -velocity;
    } else {
      position = 2.0 * upper - position;
      velocity = -velocity;
    }
  }
}

void StepObstacles(const EvaluationConfig& config,
                   std::vector<dpcbf::ObstacleState>& obstacles) {
  for (auto& obstacle : obstacles) {
    ReflectCoordinate(
        config.arena_center_x - 0.5 * config.arena_width + obstacle.radius,
        config.arena_center_x + 0.5 * config.arena_width - obstacle.radius,
        config.dt, obstacle.x, obstacle.velocity_x);
    ReflectCoordinate(
        config.arena_center_y - 0.5 * config.arena_height + obstacle.radius,
        config.arena_center_y + 0.5 * config.arena_height - obstacle.radius,
        config.dt, obstacle.y, obstacle.velocity_y);
  }
}

bool OutsideArena(const EvaluationConfig& config,
                  const dpcbf::RobotState& robot) {
  return std::abs(robot.x - config.arena_center_x) >
             0.5 * config.arena_width - config.robot_radius ||
         std::abs(robot.y - config.arena_center_y) >
             0.5 * config.arena_height - config.robot_radius;
}

double MinimumClearance(const EvaluationConfig& config,
                        const dpcbf::RobotState& robot,
                        const std::vector<dpcbf::ObstacleState>& obstacles) {
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto& obstacle : obstacles) {
    clearance = std::min(
        clearance,
        std::hypot(obstacle.x - robot.x, obstacle.y - robot.y) -
            config.robot_radius - obstacle.radius);
  }
  return clearance;
}

struct RolloutMetrics {
  bool collision = false;
  bool arena_exit = false;
  bool qp_failure = false;
  double survival_ratio = 0.0;
  double tracking_mse = 0.0;
  double intervention_mse = 0.0;
  double slack_mse = 0.0;
  double decay_mse = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();
};

RolloutMetrics RunRollout(const std::filesystem::path& config_path,
                          const EvaluationConfig& config,
                          int rollout_index) {
  const std::uint64_t rollout_seed =
      config.seed +
      0x9e3779b97f4a7c15ULL *
          static_cast<std::uint64_t>(rollout_index + 1);
  std::mt19937_64 generator(rollout_seed);
  auto obstacles = SpawnObstacles(config, generator);
  dpcbf::RobotState robot;
  robot.x = config.arena_center_x;
  robot.y = config.arena_center_y;
  CommandGenerator command_generator(config, generator, rollout_index);

  dpcbf::DpcbfSafetyFilter filter;
  filter.LoadConfig(config_path);
  filter.Initialize(config.dt);

  RolloutMetrics metrics;
  double tracking_sum = 0.0;
  double intervention_sum = 0.0;
  double slack_sum = 0.0;
  std::size_t slack_count = 0;
  double decay_sum = 0.0;
  std::size_t decay_count = 0;
  int executed_steps = 0;

  for (int step = 0; step < config.max_steps; ++step) {
    metrics.minimum_clearance = std::min(
        metrics.minimum_clearance,
        MinimumClearance(config, robot, obstacles));
    if (metrics.minimum_clearance <= 0.0) {
      metrics.collision = true;
      break;
    }
    if (OutsideArena(config, robot)) {
      metrics.arena_exit = true;
      break;
    }

    const double time = static_cast<double>(step) * config.dt;
    const dpcbf::VelocityCommand desired =
        command_generator.Next(robot, time);
    const auto result = filter.Filter(robot, desired, obstacles);
    if (!result.solved) {
      metrics.qp_failure = true;
      break;
    }

    const double tracking_s =
        desired.sagittal - robot.sagittal_velocity;
    const double tracking_l =
        desired.lateral - robot.lateral_velocity;
    const double tracking_w =
        desired.yaw_rate - result.acceleration[2];
    tracking_sum += tracking_s * tracking_s +
                    tracking_l * tracking_l +
                    tracking_w * tracking_w;

    const double reference_s = Clamp(
        config.k_a_s * tracking_s, config.a_s_min, config.a_s_max);
    const double reference_l = Clamp(
        config.k_a_l * tracking_l, config.a_l_min, config.a_l_max);
    const double reference_w =
        Clamp(desired.yaw_rate, config.w_min, config.w_max);
    const double intervention_s =
        result.acceleration[0] - reference_s;
    const double intervention_l =
        result.acceleration[1] - reference_l;
    const double intervention_w =
        result.acceleration[2] - reference_w;
    intervention_sum += intervention_s * intervention_s +
                        intervention_l * intervention_l +
                        intervention_w * intervention_w;
    for (double slack : result.slack_variables) {
      slack_sum += slack * slack;
      ++slack_count;
    }
    for (double decay : result.decay_variables) {
      const double difference = decay - 1.0;
      decay_sum += difference * difference;
      ++decay_count;
    }

    const double cosine = std::cos(robot.phi);
    const double sine = std::sin(robot.phi);
    robot.x +=
        (robot.sagittal_velocity * cosine -
         robot.lateral_velocity * sine) *
        config.dt;
    robot.y +=
        (robot.sagittal_velocity * sine +
         robot.lateral_velocity * cosine) *
        config.dt;
    robot.phi =
        WrapAngle(robot.phi + result.acceleration[2] * config.dt);
    // The filter applies its reference-frequency scaling and velocity bounds
    // when producing the command. Use that exact deployed command here.
    robot.sagittal_velocity = result.command.sagittal;
    robot.lateral_velocity = result.command.lateral;
    StepObstacles(config, obstacles);
    ++executed_steps;

    const double clearance =
        MinimumClearance(config, robot, obstacles);
    metrics.minimum_clearance =
        std::min(metrics.minimum_clearance, clearance);
    if (clearance <= 0.0) {
      metrics.collision = true;
      break;
    }
    if (OutsideArena(config, robot)) {
      metrics.arena_exit = true;
      break;
    }
  }

  metrics.survival_ratio =
      static_cast<double>(executed_steps) /
      static_cast<double>(config.max_steps);
  const double step_denominator =
      static_cast<double>(std::max(executed_steps, 1));
  metrics.tracking_mse = tracking_sum / step_denominator;
  metrics.intervention_mse = intervention_sum / step_denominator;
  metrics.slack_mse =
      slack_count > 0 ? slack_sum / static_cast<double>(slack_count) : 0.0;
  metrics.decay_mse =
      decay_count > 0 ? decay_sum / static_cast<double>(decay_count) : 0.0;
  return metrics;
}

struct AggregateMetrics {
  int rollouts = 0;
  int successes = 0;
  int collisions = 0;
  int arena_exits = 0;
  int qp_failures = 0;
  double survival_ratio = 0.0;
  double tracking_mse = 0.0;
  double intervention_mse = 0.0;
  double slack_mse = 0.0;
  double decay_mse = 0.0;
  double mean_minimum_clearance = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();

  void Add(const RolloutMetrics& metrics) {
    ++rollouts;
    collisions += metrics.collision ? 1 : 0;
    arena_exits += metrics.arena_exit ? 1 : 0;
    qp_failures += metrics.qp_failure ? 1 : 0;
    if (!metrics.collision && !metrics.arena_exit &&
        !metrics.qp_failure) {
      ++successes;
    }
    survival_ratio += metrics.survival_ratio;
    tracking_mse += metrics.tracking_mse;
    intervention_mse += metrics.intervention_mse;
    slack_mse += metrics.slack_mse;
    decay_mse += metrics.decay_mse;
    mean_minimum_clearance += metrics.minimum_clearance;
    minimum_clearance =
        std::min(minimum_clearance, metrics.minimum_clearance);
  }
};

void WriteJson(const std::filesystem::path& output_path,
               const AggregateMetrics& metrics) {
  if (metrics.rollouts <= 0) {
    throw std::runtime_error("no rollouts were evaluated");
  }
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("could not open output file: " +
                             output_path.string());
  }
  const double denominator = static_cast<double>(metrics.rollouts);
  output << std::setprecision(17);
  output << "{\n"
         << "  \"rollouts\": " << metrics.rollouts << ",\n"
         << "  \"successes\": " << metrics.successes << ",\n"
         << "  \"collisions\": " << metrics.collisions << ",\n"
         << "  \"arena_exits\": " << metrics.arena_exits << ",\n"
         << "  \"qp_failures\": " << metrics.qp_failures << ",\n"
         << "  \"mean_survival_ratio\": "
         << metrics.survival_ratio / denominator << ",\n"
         << "  \"mean_tracking_mse\": "
         << metrics.tracking_mse / denominator << ",\n"
         << "  \"mean_intervention_mse\": "
         << metrics.intervention_mse / denominator << ",\n"
         << "  \"mean_slack_mse\": "
         << metrics.slack_mse / denominator << ",\n"
         << "  \"mean_decay_mse\": "
         << metrics.decay_mse / denominator << ",\n"
         << "  \"mean_minimum_clearance\": "
         << metrics.mean_minimum_clearance / denominator << ",\n"
         << "  \"minimum_clearance\": "
         << metrics.minimum_clearance << "\n"
         << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr
        << "usage: dpcbf_rollout_evaluator <trial_config.yaml> <result.json>\n";
    return 2;
  }
  try {
    const std::filesystem::path config_path(argv[1]);
    const YAML::Node root = YAML::LoadFile(config_path.string());
    const EvaluationConfig config = LoadEvaluationConfig(root);
    AggregateMetrics metrics;
    for (int local_index = 0; local_index < config.rollouts;
         ++local_index) {
      metrics.Add(RunRollout(
          config_path, config,
          config.rollout_start_index + local_index));
    }
    WriteJson(std::filesystem::path(argv[2]), metrics);
  } catch (const std::exception& error) {
    std::cerr << "DPCBF rollout evaluation failed: "
              << error.what() << '\n';
    return 1;
  }
  return 0;
}
