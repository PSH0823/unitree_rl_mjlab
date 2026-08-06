// DpcbfVizPublisher unit tests (in-process rclcpp): decimation to the
// configured rate, latest-tick-wins content, silence when the control loop
// stalls, non-blocking Push, and the plot_bridge yaml section loader.
#include "dpcbf_ros_adapter/viz_publisher.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <dpcbf_viz_msgs/msg/dpcbf_plot_sample.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dpcbf_ros_adapter/adapter_config.h"

namespace dra = dpcbf_ros_adapter;
using dpcbf_viz_msgs::msg::DpcbfPlotSample;

namespace {

dra::PlotSample MakeSample(std::uint64_t tick) {
  dra::PlotSample s;
  s.tick = tick;
  s.t_ctrl = 0.001 * static_cast<double>(tick);
  s.mode = 2;
  s.robot_x = 1.0;
  s.robot_y = -2.0;
  s.robot_phi = 0.5;
  s.robot_sagittal_velocity = 0.7;
  s.robot_lateral_velocity = -0.1;
  s.nominal = {0.8, 0.0, 0.2};
  s.scaled = {0.4, 0.0, 0.1};
  s.safe = {0.2, 0.05, 0.1};
  s.command_scale = 0.5;
  s.solved = true;
  s.active_constraints = 2;
  s.active_dpcbf_constraints = 1;
  s.active_ecbf_constraints = 1;
  s.staleness_state = 1;
  s.obstacle_age_s = 0.35;
  s.obstacle_total = 5;
  s.obstacle_count = 2;
  s.obstacles[0] = {7, 2.0, 0.0, 0.3, -0.5, 0.0, 2.0, 0.8};
  s.obstacles[1] = {9, 0.0, 3.0, 0.4, 0.0, 0.0, 3.0, -0.2};
  return s;
}

class Collector {
 public:
  explicit Collector(const std::string& topic) {
    node_ = rclcpp::Node::make_shared("viz_test_collector");
    sub_ = node_->create_subscription<DpcbfPlotSample>(
        topic, rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
        [this](DpcbfPlotSample::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          msgs_.push_back(*m);
        });
    executor_.add_node(node_);
    thread_ = std::thread([this] {
      while (!stop_.load() && rclcpp::ok()) {
        executor_.spin_some(std::chrono::milliseconds(20));
      }
    });
  }
  ~Collector() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }
  std::vector<DpcbfPlotSample> msgs() {
    std::lock_guard<std::mutex> lock(mutex_);
    return msgs_;
  }
  std::size_t count() {
    std::lock_guard<std::mutex> lock(mutex_);
    return msgs_.size();
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<DpcbfPlotSample>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::mutex mutex_;
  std::vector<DpcbfPlotSample> msgs_;
};

bool WaitFor(const std::function<bool()>& pred, double timeout_s = 10.0) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return pred();
}

}  // namespace

TEST(VizPublisher, PublishesLatestTickContent) {
  dra::DpcbfVizPublisher::Config cfg;
  cfg.topic = "/viz_test/content";
  cfg.rate_hz = 50.0;
  cfg.frame_id = "odom";
  dra::DpcbfVizPublisher pub(cfg);
  Collector collector(cfg.topic);

  // Push a burst of ticks; the topic must end on the newest one.
  for (std::uint64_t t = 1; t <= 200; ++t) pub.Push(MakeSample(t));
  ASSERT_TRUE(WaitFor([&] {
    auto m = collector.msgs();
    return !m.empty() && m.back().tick == 200;
  })) << "newest tick never arrived";

  const auto msgs = collector.msgs();
  const auto& m = msgs.back();
  EXPECT_EQ(m.header.frame_id, "odom");
  EXPECT_EQ(m.mode, 2);
  EXPECT_DOUBLE_EQ(m.robot_x, 1.0);
  EXPECT_DOUBLE_EQ(m.robot_phi, 0.5);
  EXPECT_DOUBLE_EQ(m.nominal.sagittal, 0.8);
  EXPECT_DOUBLE_EQ(m.scaled.sagittal, 0.4);
  EXPECT_DOUBLE_EQ(m.safe.sagittal, 0.2);
  EXPECT_DOUBLE_EQ(m.command_scale, 0.5);
  EXPECT_TRUE(m.solved);
  EXPECT_TRUE(m.intervention);  // safe != scaled
  EXPECT_EQ(m.staleness_state, 1);
  EXPECT_DOUBLE_EQ(m.obstacle_age_s, 0.35);
  EXPECT_EQ(m.obstacle_total, 5u);
  ASSERT_EQ(m.obstacles.size(), 2u);
  EXPECT_EQ(m.obstacles[0].id, 7);
  EXPECT_DOUBLE_EQ(m.obstacles[1].h, -0.2);
  EXPECT_TRUE(m.min_h_valid);
  EXPECT_DOUBLE_EQ(m.min_h, -0.2);
  // min over (distance - radius): min(2.0-0.3, 3.0-0.4) = 1.7
  EXPECT_DOUBLE_EQ(m.min_clearance, 1.7);
}

TEST(VizPublisher, NoInterventionWhenSafeEqualsScaled) {
  dra::DpcbfVizPublisher::Config cfg;
  cfg.topic = "/viz_test/no_intervention";
  cfg.rate_hz = 50.0;
  dra::DpcbfVizPublisher pub(cfg);
  Collector collector(cfg.topic);

  auto s = MakeSample(1);
  s.safe = s.scaled;
  s.obstacle_count = 0;
  pub.Push(s);
  ASSERT_TRUE(WaitFor([&] { return collector.count() > 0; }));
  const auto m = collector.msgs().back();
  EXPECT_FALSE(m.intervention);
  EXPECT_FALSE(m.min_h_valid);
  EXPECT_TRUE(m.obstacles.empty());
}

TEST(VizPublisher, DecimatesToConfiguredRateAndStopsWhenStalled) {
  dra::DpcbfVizPublisher::Config cfg;
  cfg.topic = "/viz_test/rate";
  cfg.rate_hz = 20.0;
  dra::DpcbfVizPublisher pub(cfg);
  Collector collector(cfg.topic);

  // Feed ~1 kHz for 2 s while the publisher decimates.
  std::atomic<bool> stop{false};
  std::thread feeder([&] {
    std::uint64_t tick = 0;
    while (!stop.load()) {
      pub.Push(MakeSample(++tick));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  // Wait for the subscription to match, then measure over a fixed window.
  ASSERT_TRUE(WaitFor([&] { return collector.count() > 0; }));
  const auto n0 = pub.published_count();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  const auto published = pub.published_count() - n0;
  stop.store(true);
  feeder.join();
  // 20 Hz over 2 s = 40; generous bounds absorb scheduler jitter.
  EXPECT_GE(published, 20u);
  EXPECT_LE(published, 60u);

  // Control loop "stalls": no new tick -> no further publishes. The LAST
  // pushed tick may legitimately still go out once; wait for the count to
  // settle before asserting silence.
  ASSERT_TRUE(WaitFor([&] {
    const auto a = pub.published_count();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    return pub.published_count() == a;
  }));
  const auto stalled_base = pub.published_count();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_EQ(pub.published_count(), stalled_base)
      << "publisher must fall silent when the control loop stops ticking";
}

TEST(VizPublisher, PushIsCheapUnderConcurrentPublishing) {
  dra::DpcbfVizPublisher::Config cfg;
  cfg.topic = "/viz_test/latency";
  cfg.rate_hz = 50.0;
  dra::DpcbfVizPublisher pub(cfg);
  Collector collector(cfg.topic);

  const auto sample = MakeSample(1);
  double worst_us = 0.0;
  for (int i = 0; i < 20000; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    pub.Push(sample);
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    if (us > worst_us) worst_us = us;
  }
  // A bounded memcpy behind a try_lock: generous CI-safe bound, but a Push
  // that ever blocks on DDS/serialization would blow far past this.
  EXPECT_LT(worst_us, 5000.0) << "Push took " << worst_us << " us";
}

TEST(VizBridgeConfig, LoadsAndValidates) {
  const auto dir = std::filesystem::temp_directory_path();
  const auto good = dir / "viz_bridge_good.yaml";
  {
    std::ofstream f(good);
    f << "dpcbf_ros_adapter:\n  topic: /obstacles_safe\n  max_age: 0.3\n"
      << "  fade_out: 0.3\n  hold_after_stale: 1.0\n"
      << "plot_bridge:\n  enabled: true\n  topic: /dpcbf/plot\n"
      << "  rate_hz: 30.0\n  frame_id: odom\n";
  }
  dra::DpcbfVizPublisher::Config cfg;
  EXPECT_TRUE(dra::LoadVizBridgeConfig(good, &cfg));
  EXPECT_EQ(cfg.topic, "/dpcbf/plot");
  EXPECT_DOUBLE_EQ(cfg.rate_hz, 30.0);
  EXPECT_EQ(cfg.frame_id, "odom");

  const auto missing = dir / "viz_bridge_missing.yaml";
  {
    std::ofstream f(missing);
    f << "plot_bridge:\n  enabled: true\n  topic: /dpcbf/plot\n";
  }
  EXPECT_THROW(dra::LoadVizBridgeConfig(missing, &cfg), std::runtime_error);

  const auto no_section = dir / "viz_bridge_none.yaml";
  {
    std::ofstream f(no_section);
    f << "dpcbf_ros_adapter:\n  topic: /obstacles_safe\n";
  }
  EXPECT_THROW(dra::LoadVizBridgeConfig(no_section, &cfg),
               std::runtime_error);

  const auto bad_rate = dir / "viz_bridge_bad_rate.yaml";
  {
    std::ofstream f(bad_rate);
    f << "plot_bridge:\n  enabled: true\n  topic: /dpcbf/plot\n"
      << "  rate_hz: 500.0\n  frame_id: odom\n";
  }
  EXPECT_THROW(dra::LoadVizBridgeConfig(bad_rate, &cfg), std::runtime_error);
  for (const auto& p : {good, missing, no_section, bad_rate}) {
    std::filesystem::remove(p);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
