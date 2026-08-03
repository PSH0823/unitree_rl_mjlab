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

// ---------------------------------------------------------------------------
// Standing-human body plan
// ---------------------------------------------------------------------------
// Every number below is a fraction of the person's total standing height H, so
// one table covers the whole population range. Segment ratios follow the
// Drillis & Contini standing-adult proportions, rounded to 0.005; the widths
// are chosen so a 1.75 m body comes out 0.50 m across the shoulders (arms
// hanging at the sides), 0.24 m deep at the chest and 0.245 m at the foot.
//
// Body-local frame: origin between the feet ON THE GROUND, +x forward (the
// direction the person faces), +y left, +z up. Landmark heights that matter
// when picking the `pointcloud_to_laserscan` height band, in fractions of H:
//
//   0.04 ankle | 0.285 knee | 0.53 hip | 0.60 waist | 0.69 chest bottom
//   0.82 shoulder | 0.87 chin | 1.00 crown
//
// Note when tuning that band: the hanging arms are as wide as the shoulders and
// span 0.455..0.790 H, so every slice from the hips to the chin returns the SAME
// silhouette — narrowing the band buys nothing. What does change the fitted
// radius by 2x is aspect angle (0.51 m across the shoulders vs 0.24 m deep).
// `dpcbf/tools/human_band_probe.cc` measures both; see dpcbf/README.md.
struct HumanPart {
  const char* name;
  mjtGeom type;
  // Capsules: the two axis endpoints. Everything else: `a` is the center and
  // `b` is unused.
  std::array<double, 3> a;
  std::array<double, 3> b;
  // Capsules: {radius, 0, 0}. Ellipsoid/box: the three semi-axes.
  std::array<double, 3> size;
};

constexpr std::array<HumanPart, 14> kHumanParts{{
    {"foot_r", mjGEOM_BOX, {0.030, -0.085, 0.021}, {}, {0.070, 0.028, 0.021}},
    {"foot_l", mjGEOM_BOX, {0.030, 0.085, 0.021}, {}, {0.070, 0.028, 0.021}},
    {"shin_r", mjGEOM_CAPSULE, {0.0, -0.085, 0.055}, {0.0, -0.085, 0.285}, {0.033, 0.0, 0.0}},
    {"shin_l", mjGEOM_CAPSULE, {0.0, 0.085, 0.055}, {0.0, 0.085, 0.285}, {0.033, 0.0, 0.0}},
    {"thigh_r", mjGEOM_CAPSULE, {0.0, -0.085, 0.285}, {0.0, -0.090, 0.530}, {0.048, 0.0, 0.0}},
    {"thigh_l", mjGEOM_CAPSULE, {0.0, 0.085, 0.285}, {0.0, 0.090, 0.530}, {0.048, 0.0, 0.0}},
    {"pelvis", mjGEOM_ELLIPSOID, {0.0, 0.0, 0.555}, {}, {0.068, 0.098, 0.070}},
    {"abdomen", mjGEOM_ELLIPSOID, {0.0, 0.0, 0.655}, {}, {0.062, 0.088, 0.055}},
    {"chest", mjGEOM_ELLIPSOID, {0.0, 0.0, 0.760}, {}, {0.070, 0.108, 0.070}},
    {"shoulders", mjGEOM_CAPSULE, {0.0, -0.093, 0.800}, {0.0, 0.093, 0.800}, {0.048, 0.0, 0.0}},
    {"arm_r", mjGEOM_CAPSULE, {0.0, -0.115, 0.790}, {0.0, -0.115, 0.455}, {0.028, 0.0, 0.0}},
    {"arm_l", mjGEOM_CAPSULE, {0.0, 0.115, 0.790}, {0.0, 0.115, 0.455}, {0.028, 0.0, 0.0}},
    {"neck", mjGEOM_CAPSULE, {0.0, 0.0, 0.820}, {0.0, 0.0, 0.870}, {0.037, 0.0, 0.0}},
    {"head", mjGEOM_ELLIPSOID, {0.005, 0.0, 0.935}, {}, {0.058, 0.050, 0.065}},
}};

// Radius of the smallest vertical cylinder containing the body plan, as a
// fraction of H. Derived from the table so edits above stay consistent with the
// value DPCBF is handed. Conservative for ellipsoids (corner bound), exact for
// capsules and boxes.
double HumanEnclosingRadiusRatio() {
  double worst = 0.0;
  for (const HumanPart& part : kHumanParts) {
    double extent = 0.0;
    if (part.type == mjGEOM_CAPSULE) {
      const double r = part.size[0];
      extent = std::max(std::hypot(part.a[0], part.a[1]),
                        std::hypot(part.b[0], part.b[1])) + r;
    } else {
      extent = std::hypot(std::abs(part.a[0]) + part.size[0],
                          std::abs(part.a[1]) + part.size[1]);
    }
    worst = std::max(worst, extent);
  }
  return worst;
}

// Fills a mocap body with one person of total height `height`. The caller has
// already placed and oriented the body; everything here is body-local.
void AddHumanGeoms(mjsBody* body, double height, const std::array<float, 4>& rgba,
                   bool collide, const std::string& suffix) {
  for (const HumanPart& part : kHumanParts) {
    mjsGeom* geom = mjs_addGeom(body, nullptr);
    SetName(geom->element, std::string("dpcbf_obstacle_") + part.name + "_" + suffix);
    geom->type = part.type;
    if (part.type == mjGEOM_CAPSULE) {
      for (int i = 0; i < 3; ++i) {
        geom->fromto[i] = part.a[i] * height;
        geom->fromto[3 + i] = part.b[i] * height;
      }
      geom->size[0] = part.size[0] * height;
    } else {
      for (int i = 0; i < 3; ++i) {
        geom->pos[i] = part.a[i] * height;
        geom->size[i] = part.size[i] * height;
      }
    }
    geom->contype = collide ? 1 : 0;
    geom->conaffinity = collide ? 1 : 0;
    SetRgba(geom, rgba);
  }
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

  const std::string shape = cfg["shape"].as<std::string>("cylinder");
  if (shape == "cylinder") {
    shape_ = ObstacleShape::kCylinder;
  } else if (shape == "human") {
    shape_ = ObstacleShape::kHuman;
  } else {
    throw std::runtime_error("shape must be 'cylinder' or 'human', got '" + shape + "'");
  }

  radius_range_ = ReadArray<double, 2>(cfg, "radius_range");
  speed_range_ = ReadArray<double, 2>(cfg, "speed_range");
  collision_enabled_ = cfg["collision_enabled"].as<bool>(true);
  rgba_ = ReadArray<float, 4>(cfg, "rgba");
  height_ = cfg["height"].as<double>(1.5);
  random_seed_ = cfg["random_seed"].as<unsigned int>(42);

  const YAML::Node human = cfg["human"];
  if (human) {
    human_height_range_ = ReadArray<double, 2>(human, "height_range");
    human_rgba_ = ReadArray<float, 4>(human, "rgba");
    human_face_travel_direction_ = human["face_travel_direction"].as<bool>(true);
  } else if (shape_ == ObstacleShape::kHuman) {
    throw std::runtime_error("shape 'human' requires a 'dynamic_obstacles.human' section");
  }

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
  if (human_height_range_[0] <= 0.0 || human_height_range_[0] > human_height_range_[1]) {
    throw std::runtime_error("human.height_range must be positive and ordered");
  }
  const double max_radius = shape_ == ObstacleShape::kHuman
                                ? HumanEnclosingRadiusRatio() * human_height_range_[1]
                                : radius_range_[1];
  if (2.0 * max_radius >= std::min(arena_size_[0], arena_size_[1])) {
    throw std::runtime_error("maximum obstacle diameter must be smaller than the arena");
  }
  if (boundary_thickness_ <= 0.0) {
    throw std::runtime_error("boundary_thickness must be positive");
  }
  const auto valid_color = [](float value) { return value >= 0.0F && value <= 1.0F; };
  if (!std::all_of(rgba_.begin(), rgba_.end(), valid_color) ||
      !std::all_of(human_rgba_.begin(), human_rgba_.end(), valid_color) ||
      !std::all_of(boundary_rgba_.begin(), boundary_rgba_.end(), valid_color)) {
    throw std::runtime_error("RGBA values must be in [0, 1]");
  }

  config_path_ = path;
  std::cout << "Loaded dynamic obstacles from " << config_path_ << " (shape: "
            << (shape_ == ObstacleShape::kHuman ? "human" : "cylinder") << ")\n";
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

  const bool is_human = shape_ == ObstacleShape::kHuman;
  const double human_radius_ratio = HumanEnclosingRadiusRatio();

  std::mt19937 generator(random_seed_);
  std::uniform_real_distribution<double> radius_distribution(radius_range_[0], radius_range_[1]);
  std::uniform_real_distribution<double> height_distribution(human_height_range_[0],
                                                             human_height_range_[1]);
  std::uniform_real_distribution<double> speed_distribution(speed_range_[0], speed_range_[1]);
  std::uniform_real_distribution<double> angle_distribution(0.0, 2.0 * mjPI);

  for (int i = 0; i < count_; ++i) {
    DynamicObstacle obstacle;
    // Draw the shape parameter first either way, so the arena placement stream
    // below stays comparable between the two shapes for a given seed.
    if (is_human) {
      obstacle.height = height_distribution(generator);
      obstacle.radius = human_radius_ratio * obstacle.height;
    } else {
      obstacle.radius = radius_distribution(generator);
      obstacle.height = height_;
    }
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
    // A human is not axially symmetric, so where it looks decides how wide it
    // appears to the LiDAR. A walker faces where it is going; a standing one
    // gets a random heading so a static field still covers every aspect angle.
    obstacle.yaw = is_human ? (speed > 0.0 && human_face_travel_direction_
                                   ? angle
                                   : angle_distribution(generator))
                            : 0.0;
    obstacle.initial_position = obstacle.position;
    obstacle.initial_velocity = obstacle.velocity;
    obstacle.initial_yaw = obstacle.yaw;
    obstacles_.push_back(obstacle);

    const std::string suffix = std::to_string(i);
    mjsBody* body = mjs_addBody(world, nullptr);
    SetName(body->element, "dpcbf_obstacle_" + suffix);
    body->mocap = 1;
    body->pos[0] = obstacle.position[0];
    body->pos[1] = obstacle.position[1];
    body->pos[2] = BodyFrameZ(obstacle);
    body->quat[0] = std::cos(0.5 * obstacle.yaw);
    body->quat[1] = 0.0;
    body->quat[2] = 0.0;
    body->quat[3] = std::sin(0.5 * obstacle.yaw);

    if (is_human) {
      AddHumanGeoms(body, obstacle.height, human_rgba_, collision_enabled_, suffix);
    } else {
      mjsGeom* geom = mjs_addGeom(body, nullptr);
      SetName(geom->element, "dpcbf_obstacle_geom_" + suffix);
      geom->type = mjGEOM_CYLINDER;
      geom->size[0] = obstacle.radius;
      geom->size[1] = 0.5 * obstacle.height;
      geom->contype = collision_enabled_ ? 1 : 0;
      geom->conaffinity = collision_enabled_ ? 1 : 0;
      SetRgba(geom, rgba_);
    }
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
  WriteMocapPoses(model, data);
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
    // Turn with the bounce, so a human never walks sideways after hitting a wall.
    if (shape_ == ObstacleShape::kHuman && human_face_travel_direction_ &&
        (obstacle.velocity[0] != 0.0 || obstacle.velocity[1] != 0.0)) {
      obstacle.yaw = std::atan2(obstacle.velocity[1], obstacle.velocity[0]);
    }
  }
  WriteMocapPoses(model, data);
}

void DynamicObstacleManager::Reset(const mjModel* model, mjData* data) {
  const std::lock_guard<std::mutex> lock(mutex_);
  for (DynamicObstacle& obstacle : obstacles_) {
    obstacle.position = obstacle.initial_position;
    obstacle.velocity = obstacle.initial_velocity;
    obstacle.yaw = obstacle.initial_yaw;
  }
  WriteMocapPoses(model, data);
}

std::vector<DynamicObstacle> DynamicObstacleManager::Snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return obstacles_;
}

double DynamicObstacleManager::BodyFrameZ(const DynamicObstacle& obstacle) const {
  return shape_ == ObstacleShape::kHuman ? 0.0 : 0.5 * obstacle.height;
}

void DynamicObstacleManager::WriteMocapPoses(const mjModel* model, mjData* data) const {
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
    position[2] = BodyFrameZ(obstacle);

    // The mirror model in the perception sidecar rebuilds poses from these, so
    // the heading has to travel with the position, not just sit in body_quat.
    mjtNum* orientation = data->mocap_quat + 4 * obstacle.mocap_id;
    orientation[0] = std::cos(0.5 * obstacle.yaw);
    orientation[1] = 0.0;
    orientation[2] = 0.0;
    orientation[3] = std::sin(0.5 * obstacle.yaw);
  }
}

}  // namespace dpcbf
