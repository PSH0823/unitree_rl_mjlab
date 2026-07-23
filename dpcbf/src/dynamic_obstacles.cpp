#include "dpcbf/dynamic_obstacles.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace dpcbf {
namespace {

template <typename T, std::size_t N>
std::array<T, N> ReadArray(const YAML::Node& node, const char* key) {
  const YAML::Node value = node[key];
  if (!value || !value.IsSequence() || value.size() != N) {
    throw std::runtime_error(std::string("'") + key + "' must contain " +
                             std::to_string(N) + " values");
  }
  std::array<T, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = value[i].as<T>();
  }
  return result;
}

void SetName(mjsElement* element, const std::string& name) {
  if (mjs_setName(element, name.c_str()) != 0) {
    throw std::runtime_error("failed to set MuJoCo element name: " + name);
  }
}

void SetRgba(mjsGeom* geom, const std::array<float, 4>& rgba) {
  std::copy(rgba.begin(), rgba.end(), geom->rgba);
}

void ReflectCoordinate(double min_value, double max_value, double dt,
                       double* position, double* velocity) {
  if (max_value <= min_value) {
    *position = 0.5 * (min_value + max_value);
    *velocity = 0.0;
    return;
  }

  *position += *velocity * dt;
  // The loop also handles unusually large dt values that cross multiple walls.
  while (*position < min_value || *position > max_value) {
    if (*position < min_value) {
      *position = 2.0 * min_value - *position;
      *velocity = std::abs(*velocity);
    }
    if (*position > max_value) {
      *position = 2.0 * max_value - *position;
      *velocity = -std::abs(*velocity);
    }
  }
}

}  // namespace

void DynamicObstacleManager::LoadConfig(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node cfg = root["dynamic_obstacles"];
  if (!cfg) {
    throw std::runtime_error("missing 'dynamic_obstacles' section");
  }

  enabled_ = cfg["enabled"].as<bool>(true);
  count_ = cfg["count"].as<int>(5);
  radius_range_ = ReadArray<double, 2>(cfg, "radius_range");
  speed_range_ = ReadArray<double, 2>(cfg, "speed_range");
  collision_enabled_ = cfg["collision_enabled"].as<bool>(true);
  rgba_ = ReadArray<float, 4>(cfg, "rgba");
  height_ = cfg["height"].as<double>(1.5);
  random_seed_ = cfg["random_seed"].as<unsigned int>(42);

  const YAML::Node arena = cfg["arena"];
  if (!arena) {
    throw std::runtime_error("missing 'dynamic_obstacles.arena' section");
  }
  arena_size_ = ReadArray<double, 2>(arena, "size");
  arena_center_ = ReadArray<double, 2>(arena, "center");
  show_boundary_ = arena["show_boundary"].as<bool>(true);
  boundary_thickness_ = arena["boundary_thickness"].as<double>(0.04);
  boundary_rgba_ = ReadArray<float, 4>(arena, "boundary_rgba");

  if (count_ < 0 || count_ > 1000) {
    throw std::runtime_error("count must be in [0, 1000]");
  }
  if (radius_range_[0] <= 0.0 || radius_range_[0] > radius_range_[1]) {
    throw std::runtime_error("radius_range must be positive and ordered");
  }
  if (speed_range_[0] < 0.0 || speed_range_[0] > speed_range_[1]) {
    throw std::runtime_error("speed_range must be non-negative and ordered");
  }
  if (height_ <= 0.0 || arena_size_[0] <= 0.0 || arena_size_[1] <= 0.0) {
    throw std::runtime_error("height and arena size must be positive");
  }
  if (2.0 * radius_range_[1] >= std::min(arena_size_[0], arena_size_[1])) {
    throw std::runtime_error("maximum obstacle diameter must be smaller than the arena");
  }
  if (boundary_thickness_ <= 0.0) {
    throw std::runtime_error("boundary_thickness must be positive");
  }
  const auto valid_color = [](float value) { return value >= 0.0F && value <= 1.0F; };
  if (!std::all_of(rgba_.begin(), rgba_.end(), valid_color) ||
      !std::all_of(boundary_rgba_.begin(), boundary_rgba_.end(), valid_color)) {
    throw std::runtime_error("RGBA values must be in [0, 1]");
  }

  config_path_ = path;
  std::cout << "Loaded dynamic obstacles from " << config_path_ << '\n';
}

bool DynamicObstacleManager::AddToSpec(mjSpec* spec) {
  const std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
  if (!enabled_) {
    return true;
  }
  if (!spec) {
    return false;
  }

  mjsBody* world = mjs_findBody(spec, "world");
  if (!world) {
    throw std::runtime_error("MuJoCo world body was not found");
  }

  std::mt19937 generator(random_seed_);
  std::uniform_real_distribution<double> radius_distribution(radius_range_[0], radius_range_[1]);
  std::uniform_real_distribution<double> speed_distribution(speed_range_[0], speed_range_[1]);
  std::uniform_real_distribution<double> angle_distribution(0.0, 2.0 * mjPI);

  for (int i = 0; i < count_; ++i) {
    DynamicObstacle obstacle;
    obstacle.radius = radius_distribution(generator);
    const double half_x = 0.5 * arena_size_[0] - obstacle.radius;
    const double half_y = 0.5 * arena_size_[1] - obstacle.radius;
    std::uniform_real_distribution<double> x_distribution(arena_center_[0] - half_x,
                                                          arena_center_[0] + half_x);
    std::uniform_real_distribution<double> y_distribution(arena_center_[1] - half_y,
                                                          arena_center_[1] + half_y);
    obstacle.position = {x_distribution(generator), y_distribution(generator)};
    const double speed = speed_distribution(generator);
    const double angle = angle_distribution(generator);
    obstacle.velocity = {speed * std::cos(angle), speed * std::sin(angle)};
    obstacle.initial_position = obstacle.position;
    obstacle.initial_velocity = obstacle.velocity;
    obstacles_.push_back(obstacle);

    const std::string suffix = std::to_string(i);
    mjsBody* body = mjs_addBody(world, nullptr);
    SetName(body->element, "dpcbf_obstacle_" + suffix);
    body->mocap = 1;
    body->pos[0] = obstacle.position[0];
    body->pos[1] = obstacle.position[1];
    body->pos[2] = 0.5 * height_;

    mjsGeom* geom = mjs_addGeom(body, nullptr);
    SetName(geom->element, "dpcbf_obstacle_geom_" + suffix);
    geom->type = mjGEOM_CYLINDER;
    geom->size[0] = obstacle.radius;
    geom->size[1] = 0.5 * height_;
    geom->contype = collision_enabled_ ? 1 : 0;
    geom->conaffinity = collision_enabled_ ? 1 : 0;
    SetRgba(geom, rgba_);
  }

  if (show_boundary_) {
    const double half_x = 0.5 * arena_size_[0];
    const double half_y = 0.5 * arena_size_[1];
    for (int i = 0; i < 4; ++i) {
      mjsGeom* wall = mjs_addGeom(world, nullptr);
      SetName(wall->element, "dpcbf_arena_wall_" + std::to_string(i));
      wall->type = mjGEOM_BOX;
      wall->contype = 0;
      wall->conaffinity = 0;
      wall->pos[2] = 0.5 * height_;
      wall->size[2] = 0.5 * height_;
      if (i < 2) {
        wall->pos[0] = arena_center_[0] + (i == 0 ? -half_x : half_x);
        wall->pos[1] = arena_center_[1];
        wall->size[0] = 0.5 * boundary_thickness_;
        wall->size[1] = half_y;
      } else {
        wall->pos[0] = arena_center_[0];
        wall->pos[1] = arena_center_[1] + (i == 2 ? -half_y : half_y);
        wall->size[0] = half_x;
        wall->size[1] = 0.5 * boundary_thickness_;
      }
      SetRgba(wall, boundary_rgba_);
    }
  }

  return true;
}

bool DynamicObstacleManager::BindModel(const mjModel* model, mjData* data) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return true;
  }
  if (!model || !data) {
    return false;
  }

  for (std::size_t i = 0; i < obstacles_.size(); ++i) {
    const std::string name = "dpcbf_obstacle_" + std::to_string(i);
    const int body_id = mj_name2id(model, mjOBJ_BODY, name.c_str());
    if (body_id < 0 || model->body_mocapid[body_id] < 0) {
      std::cerr << "Dynamic obstacle body is missing: " << name << '\n';
      return false;
    }
    obstacles_[i].mocap_id = model->body_mocapid[body_id];
  }
  WriteMocapPositions(model, data);
  mj_forward(model, data);
  return true;
}

void DynamicObstacleManager::Step(const mjModel* model, mjData* data, double dt) {
  if (!enabled_ || !model || !data || dt <= 0.0) {
    return;
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  for (DynamicObstacle& obstacle : obstacles_) {
    const double min_x = arena_center_[0] - 0.5 * arena_size_[0] + obstacle.radius;
    const double max_x = arena_center_[0] + 0.5 * arena_size_[0] - obstacle.radius;
    const double min_y = arena_center_[1] - 0.5 * arena_size_[1] + obstacle.radius;
    const double max_y = arena_center_[1] + 0.5 * arena_size_[1] - obstacle.radius;
    ReflectCoordinate(min_x, max_x, dt, &obstacle.position[0], &obstacle.velocity[0]);
    ReflectCoordinate(min_y, max_y, dt, &obstacle.position[1], &obstacle.velocity[1]);
  }
  WriteMocapPositions(model, data);
}

void DynamicObstacleManager::Reset(const mjModel* model, mjData* data) {
  const std::lock_guard<std::mutex> lock(mutex_);
  for (DynamicObstacle& obstacle : obstacles_) {
    obstacle.position = obstacle.initial_position;
    obstacle.velocity = obstacle.initial_velocity;
  }
  WriteMocapPositions(model, data);
}

std::vector<DynamicObstacle> DynamicObstacleManager::Snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return obstacles_;
}

void DynamicObstacleManager::WriteMocapPositions(const mjModel* model, mjData* data) const {
  if (!model || !data) {
    return;
  }
  for (const DynamicObstacle& obstacle : obstacles_) {
    if (obstacle.mocap_id < 0 || obstacle.mocap_id >= model->nmocap) {
      continue;
    }
    mjtNum* position = data->mocap_pos + 3 * obstacle.mocap_id;
    position[0] = obstacle.position[0];
    position[1] = obstacle.position[1];
    position[2] = 0.5 * height_;
  }
}

}  // namespace dpcbf
