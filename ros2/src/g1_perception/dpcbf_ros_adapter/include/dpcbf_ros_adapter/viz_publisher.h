#pragma once
// DpcbfVizPublisher: the control→plot seam. The 1 kHz control loop deposits a
// fixed-size POD snapshot of each Filter() tick (Push, never blocking); a
// separate ROS thread — own node, own SingleThreadedExecutor, the exact
// arrangement ObstacleSource already uses — publishes the LATEST snapshot as
// dpcbf_viz_msgs/DpcbfPlotSample at the configured plot rate (default 30 Hz).
//
// Design rules this class exists to enforce (visualization runbook):
//   * the control loop never allocates, serializes, or touches DDS — Push is
//     a bounded memcpy behind a try_lock and DROPS the sample if the 30 Hz
//     reader happens to hold the lock (a lost 1 kHz sample is invisible at
//     plot rate; a blocked control tick is not);
//   * decimation happens HERE, not on the wire: whatever the control rate,
//     the topic carries at most rate_hz samples/s, each internally consistent
//     (all fields from one tick);
//   * no publish when no new tick arrived since the last one (a stalled
//     control loop becomes VISIBLY stale on the client instead of being
//     papered over by re-sent data);
//   * losing every subscriber, the whole topic, or the network changes
//     nothing on the control side — QoS is BestEffort KeepLast, and the
//     publisher's timer keeps its own thread.
//
// This header is rclcpp-free ONLY in what it exposes (plain structs); the
// class itself links rclcpp + dpcbf_viz_msgs and is used by ROS2 builds of
// the control seam (simulate main.cc today, the §9 hardware seam tomorrow).

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace dpcbf_ros_adapter {

// Mirrors dpcbf_viz_msgs/PlotObstacle.
struct PlotObstacleSample {
  int id = -1;
  double x = 0.0, y = 0.0, radius = 0.0;
  double velocity_x = 0.0, velocity_y = 0.0;
  double distance = 0.0;
  double h = 0.0;
};

// One control tick, fixed size (no heap): what Push copies. Field meanings
// match dpcbf_viz_msgs/DpcbfPlotSample one-to-one.
struct PlotSample {
  // The QP consumes at most qp_parameters.default_num_constraints (10 in the
  // shipped config); 16 leaves headroom without bloating the copy.
  static constexpr std::size_t kMaxObstacles = 16;

  std::uint64_t tick = 0;
  double t_ctrl = 0.0;
  std::uint8_t mode = 0;  // ObstacleSource::Mode value

  double robot_x = 0.0, robot_y = 0.0, robot_phi = 0.0;
  double robot_sagittal_velocity = 0.0, robot_lateral_velocity = 0.0;

  // sagittal, lateral, yaw_rate triplets
  std::array<double, 3> nominal{};
  std::array<double, 3> scaled{};
  std::array<double, 3> safe{};
  double command_scale = 1.0;

  bool solved = false;
  int active_constraints = 0;
  int active_dpcbf_constraints = 0;
  int active_ecbf_constraints = 0;
  std::array<double, 3> acceleration{};

  std::uint8_t staleness_state = 0;  // StalenessState value
  double obstacle_age_s = -1.0;      // -1 = not applicable (oracle)

  std::uint32_t obstacle_total = 0;  // offered to Filter() before selection
  std::uint32_t obstacle_count = 0;  // entries valid in obstacles[]
  std::array<PlotObstacleSample, kMaxObstacles> obstacles{};
};

class DpcbfVizPublisher {
 public:
  struct Config {
    std::string topic = "/dpcbf/plot";
    double rate_hz = 30.0;
    std::string frame_id = "odom";
  };

  explicit DpcbfVizPublisher(Config config);
  ~DpcbfVizPublisher();
  DpcbfVizPublisher(const DpcbfVizPublisher&) = delete;
  DpcbfVizPublisher& operator=(const DpcbfVizPublisher&) = delete;

  // Control-loop side. Never blocks: bounded memcpy under try_lock; on
  // contention the sample is dropped and dropped_pushes() counts it.
  void Push(const PlotSample& sample);

  // Instrumentation (any thread).
  std::uint64_t published_count() const;
  std::uint64_t dropped_pushes() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dpcbf_ros_adapter
