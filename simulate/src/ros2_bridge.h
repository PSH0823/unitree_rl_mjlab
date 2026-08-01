#pragma once
// Optional ROS2 module for the simulator (§11.2). Compiled only with
// -DUNITREE_MUJOCO_WITH_ROS2=ON; with the option OFF simulate builds and
// behaves exactly as at baseline f111cfa.
//
// The simulator's entire ROS2 surface (D4):
//   /clock             rosgraph_msgs/Clock          >=100 Hz
//   /sim/mj_state      sim_msgs/MjState             100 Hz, BestEffort depth 1
//   /sim/gt_obstacles  obstacle_detector/Obstacles  50 Hz, odom frame
// plus a one-shot dump of the COMPILED model XML (post-AddToSpec, absolute
// meshdir) for the sim_mjlidar_bridge mirror — the raw scene_g1.xml lacks the
// runtime-added obstacle mocap bodies.
#ifdef UNITREE_MUJOCO_WITH_ROS2

#include <functional>
#include <memory>
#include <vector>

#include "dpcbf/dynamic_obstacles.h"

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;
struct mjSpec_;
typedef struct mjSpec_ mjSpec;

// Saves the compiled spec as the mirror model XML. Called from LoadModel
// after mj_compile, before mj_deleteSpec. Destination:
// $UNITREE_MUJOCO_MIRROR_XML or /tmp/unitree_mujoco_mirror_model.xml.
void Ros2DumpMirrorModel(mjSpec* spec);

class SimRos2Bridge {
 public:
  SimRos2Bridge();
  ~SimRos2Bridge();

  // Call at ~1 kHz from the SDK2 bridge thread's idle loop. Reads d->time,
  // qpos and mocap without locks — the same pre-existing benign-race class as
  // the SDK2 bridge's own state reads (R-9: not worsened, no new locks).
  // Publish cadence is derived from SIM time, so rates stay correct under
  // slow-motion and pause. snapshot() is only invoked at the 50 Hz cadence.
  void SpinOnce(
      const mjModel* m, const mjData* d,
      const std::function<std::vector<dpcbf::DynamicObstacle>()>& snapshot);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // UNITREE_MUJOCO_WITH_ROS2
