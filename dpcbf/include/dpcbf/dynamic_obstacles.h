#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

struct mjData_;
struct mjModel_;
struct mjSpec_;

namespace dpcbf {

struct DynamicObstacle {
  double radius = 0.0;
  std::array<double, 2> position{};
  std::array<double, 2> initial_position{};
  std::array<double, 2> velocity{};
  std::array<double, 2> initial_velocity{};
  int mocap_id = -1;
};

class DynamicObstacleManager {
 public:
  // Loads and validates the configuration. Throws on invalid input.
  void LoadConfig(const std::filesystem::path& path);

  // Adds the configured mocap cylinders and visual arena walls before compile.
  bool AddToSpec(mjSpec_* spec);

  // Resolves mocap IDs after a model has been compiled and initializes mjData.
  bool BindModel(const mjModel_* model, mjData_* data);

  // Moves every obstacle by dt and reflects it at the arena boundary.
  void Step(const mjModel_* model, mjData_* data, double dt);

  // Restores the deterministic positions and velocities created at model load.
  void Reset(const mjModel_* model, mjData_* data);

  const std::vector<DynamicObstacle>& obstacles() const { return obstacles_; }
  std::vector<DynamicObstacle> Snapshot() const;
  bool enabled() const { return enabled_; }

 private:
  void WriteMocapPositions(const mjModel_* model, mjData_* data) const;

  bool enabled_ = true;
  int count_ = 5;
  std::array<double, 2> radius_range_{0.1, 0.5};
  std::array<double, 2> speed_range_{0.0, 0.6};
  bool collision_enabled_ = true;
  std::array<float, 4> rgba_{0.1F, 0.4F, 1.0F, 0.45F};
  double height_ = 1.5;
  std::array<double, 2> arena_size_{10.0, 10.0};
  std::array<double, 2> arena_center_{0.0, 0.0};
  bool show_boundary_ = true;
  double boundary_thickness_ = 0.04;
  std::array<float, 4> boundary_rgba_{0.2F, 0.7F, 1.0F, 0.12F};
  unsigned int random_seed_ = 42;
  std::filesystem::path config_path_;
  mutable std::mutex mutex_;
  std::vector<DynamicObstacle> obstacles_;
};

}  // namespace dpcbf
