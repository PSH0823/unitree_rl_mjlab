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

// Geometry each obstacle is built from. `kCylinder` is the original abstraction
// the DPCBF maths assumes; `kHuman` is a standing pedestrian assembled from
// capsules and ellipsoids, used to check that the perception chain still
// recovers a usable circle from a real body profile.
enum class ObstacleShape { kCylinder, kHuman };

struct DynamicObstacle {
  // Circumscribed radius of the standing shape: what DPCBF and the ground-truth
  // oracle treat as the obstacle. For kCylinder this is the geom radius; for
  // kHuman it is derived from the body plan (widest point: the hanging arms or
  // the feet, whichever wins).
  double radius = 0.0;
  double height = 0.0;   // full standing height of this obstacle, meters
  double yaw = 0.0;      // facing direction, radians (kHuman only; 0 for cylinders)
  double initial_yaw = 0.0;
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

  // Adds the configured mocap obstacles and visual arena walls before compile.
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
  ObstacleShape shape() const { return shape_; }

 private:
  void WriteMocapPoses(const mjModel_* model, mjData_* data) const;

  // Height of the mocap body frame above the floor. Cylinders are centered on
  // their own axis; humans stand on their feet, so their frame origin is on the
  // ground.
  double BodyFrameZ(const DynamicObstacle& obstacle) const;

  bool enabled_ = true;
  int count_ = 5;
  ObstacleShape shape_ = ObstacleShape::kCylinder;
  std::array<double, 2> radius_range_{0.1, 0.5};
  std::array<double, 2> speed_range_{0.0, 0.6};
  bool collision_enabled_ = true;
  std::array<float, 4> rgba_{0.1F, 0.4F, 1.0F, 0.45F};
  double height_ = 1.5;
  std::array<double, 2> human_height_range_{1.60, 1.85};
  std::array<float, 4> human_rgba_{0.85F, 0.55F, 0.40F, 1.0F};
  bool human_face_travel_direction_ = true;
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
