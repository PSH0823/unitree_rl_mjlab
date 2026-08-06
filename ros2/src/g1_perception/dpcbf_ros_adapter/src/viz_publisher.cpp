#include "dpcbf_ros_adapter/viz_publisher.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <dpcbf_viz_msgs/msg/dpcbf_plot_sample.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dpcbf_ros_adapter {

struct DpcbfVizPublisher::Impl {
  Config config;

  // Latest-sample mailbox. The control thread writes under try_lock (drop on
  // contention — see header); the timer thread reads under a blocking lock,
  // which is fine because it is NOT on the control path and the critical
  // section is a bounded memcpy.
  std::mutex mutex;
  PlotSample latest;                 // guarded by mutex
  bool has_sample = false;           // guarded by mutex
  std::uint64_t last_published_tick = 0;
  bool published_anything = false;

  std::atomic<std::uint64_t> published{0};
  std::atomic<std::uint64_t> dropped{0};

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<dpcbf_viz_msgs::msg::DpcbfPlotSample>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;
  std::atomic<bool> shutdown{false};

  void Tick() {
    PlotSample s;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!has_sample) return;
      // Same tick as last publish → the control loop has not advanced;
      // stay silent so the client's staleness display tells the truth.
      if (published_anything && latest.tick == last_published_tick) return;
      s = latest;
      last_published_tick = latest.tick;
      published_anything = true;
    }

    dpcbf_viz_msgs::msg::DpcbfPlotSample msg;
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = config.frame_id;
    msg.tick = s.tick;
    msg.t_ctrl = s.t_ctrl;
    msg.mode = s.mode;
    msg.robot_x = s.robot_x;
    msg.robot_y = s.robot_y;
    msg.robot_phi = s.robot_phi;
    msg.robot_sagittal_velocity = s.robot_sagittal_velocity;
    msg.robot_lateral_velocity = s.robot_lateral_velocity;
    auto cmd = [](const std::array<double, 3>& c) {
      dpcbf_viz_msgs::msg::VelocityCommand2D m;
      m.sagittal = c[0];
      m.lateral = c[1];
      m.yaw_rate = c[2];
      return m;
    };
    msg.nominal = cmd(s.nominal);
    msg.scaled = cmd(s.scaled);
    msg.safe = cmd(s.safe);
    msg.command_scale = s.command_scale;
    msg.solved = s.solved;
    msg.active_constraints = s.active_constraints;
    msg.active_dpcbf_constraints = s.active_dpcbf_constraints;
    msg.active_ecbf_constraints = s.active_ecbf_constraints;
    for (int i = 0; i < 3; ++i) msg.acceleration[i] = s.acceleration[i];
    constexpr double kEps = 1e-6;
    msg.intervention = std::abs(s.safe[0] - s.scaled[0]) > kEps ||
                       std::abs(s.safe[1] - s.scaled[1]) > kEps ||
                       std::abs(s.safe[2] - s.scaled[2]) > kEps;
    msg.staleness_state = s.staleness_state;
    msg.obstacle_age_s = s.obstacle_age_s;
    msg.obstacle_total = s.obstacle_total;
    const std::size_t n =
        s.obstacle_count < PlotSample::kMaxObstacles ? s.obstacle_count
                                                     : PlotSample::kMaxObstacles;
    msg.obstacles.reserve(n);
    double min_h = 0.0, min_clearance = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const PlotObstacleSample& o = s.obstacles[i];
      dpcbf_viz_msgs::msg::PlotObstacle m;
      m.id = o.id;
      m.x = o.x;
      m.y = o.y;
      m.radius = o.radius;
      m.velocity_x = o.velocity_x;
      m.velocity_y = o.velocity_y;
      m.distance = o.distance;
      m.h = o.h;
      const double clearance = o.distance - o.radius;
      if (i == 0 || o.h < min_h) min_h = o.h;
      if (i == 0 || clearance < min_clearance) min_clearance = clearance;
      msg.obstacles.push_back(m);
    }
    msg.min_h_valid = n > 0;
    msg.min_h = min_h;
    msg.min_clearance = min_clearance;

    pub->publish(msg);
    published.fetch_add(1, std::memory_order_relaxed);
  }
};

DpcbfVizPublisher::DpcbfVizPublisher(Config config) : impl_(new Impl) {
  impl_->config = std::move(config);
  const auto& cfg = impl_->config;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions opts;
  // Same §11.2 discipline as ObstacleSource: never consume a /clock this
  // process may itself publish. header.stamp is wall time by contract.
  opts.parameter_overrides({{"use_sim_time", false}});
  impl_->node = rclcpp::Node::make_shared("dpcbf_viz_publisher", opts);
  // BestEffort latest-wins: a plotting client on a lossy link sees the
  // freshest sample; nothing ever queues back toward this process.
  impl_->pub =
      impl_->node->create_publisher<dpcbf_viz_msgs::msg::DpcbfPlotSample>(
          cfg.topic, rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
  impl_->timer = impl_->node->create_wall_timer(
      std::chrono::duration<double>(1.0 / cfg.rate_hz),
      [impl = impl_.get()] { impl->Tick(); });
  impl_->executor =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor->add_node(impl_->node);
  impl_->spin_thread = std::thread([impl = impl_.get()] {
    while (!impl->shutdown.load(std::memory_order_relaxed) && rclcpp::ok()) {
      impl->executor->spin_some(std::chrono::milliseconds(50));
    }
  });
}

DpcbfVizPublisher::~DpcbfVizPublisher() {
  impl_->shutdown.store(true);
  if (impl_->spin_thread.joinable()) impl_->spin_thread.join();
}

void DpcbfVizPublisher::Push(const PlotSample& sample) {
  if (!impl_->mutex.try_lock()) {
    impl_->dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  impl_->latest = sample;
  impl_->has_sample = true;
  impl_->mutex.unlock();
}

std::uint64_t DpcbfVizPublisher::published_count() const {
  return impl_->published.load(std::memory_order_relaxed);
}

std::uint64_t DpcbfVizPublisher::dropped_pushes() const {
  return impl_->dropped.load(std::memory_order_relaxed);
}

}  // namespace dpcbf_ros_adapter
