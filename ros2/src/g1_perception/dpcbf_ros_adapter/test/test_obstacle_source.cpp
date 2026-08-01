// ObstacleSource integration-level unit tests (in-process rclcpp): estimated
// data flow end to end, oracle pass-through, and the structural H-9
// guarantees (mode fixed at construction; no API path mixes sources).
#include "dpcbf_ros_adapter/obstacle_source.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dra = dpcbf_ros_adapter;
using Mode = dra::ObstacleSource::Mode;

namespace {

builtin_interfaces::msg::Time ToStamp(double t) {
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(t);
  out.nanosec = static_cast<uint32_t>((t - out.sec) * 1e9);
  return out;
}

obstacle_detector::msg::Obstacles MakeMsg(double stamp,
                                          std::vector<std::array<double, 5>> circles,
                                          std::uint64_t uid0 = 100) {
  obstacle_detector::msg::Obstacles msg;
  msg.header.stamp = ToStamp(stamp);
  msg.header.frame_id = "odom";
  for (const auto& c : circles) {
    obstacle_detector::msg::CircleObstacle co;
    co.uid = uid0++;
    co.center.x = c[0];
    co.center.y = c[1];
    co.radius = c[2];
    co.true_radius = c[2];
    co.velocity.x = c[3];
    co.velocity.y = c[4];
    msg.circles.push_back(co);
  }
  return msg;
}

// Publish until the source has absorbed at least one more frame: DDS
// discovery is asynchronous, so a single publish can race the subscription
// match and be lost even on a Reliable topic.
template <typename Pub>
bool PublishUntilReceived(Pub& pub, dra::ObstacleSource& src,
                          const obstacle_detector::msg::Obstacles& msg,
                          double timeout_s = 10.0) {
  const std::uint64_t want = src.frames_received() + 1;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (src.frames_received() >= want) return true;
  }
  return false;
}

class ObstacleSourceTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  void SetUp() override {
    pub_node_ = rclcpp::Node::make_shared("test_obstacles_pub");
    pub_ = pub_node_->create_publisher<obstacle_detector::msg::Obstacles>(
        "/obstacles_safe_test", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }
  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr pub_;
};

}  // namespace

TEST_F(ObstacleSourceTest, EstimatedEndToEnd) {
  dra::ObstacleSource::Config cfg;
  cfg.mode = Mode::kEstimated;
  cfg.topic = "/obstacles_safe_test";
  dra::ObstacleSource src(cfg);

  // Before any message: fail-safe stop with an empty set.
  auto snap = src.GetObstacles(5.0);
  EXPECT_EQ(snap.state, dra::StalenessState::kNoData);
  EXPECT_TRUE(snap.obstacles.empty());
  EXPECT_DOUBLE_EQ(snap.command_scale, 0.0);

  ASSERT_TRUE(PublishUntilReceived(pub_, src,
      MakeMsg(10.0, {{1.0, 2.0, 0.4, 0.5, 0.0}}, 4242)));

  snap = src.GetObstacles(10.1);
  ASSERT_EQ(snap.obstacles.size(), 1u);
  EXPECT_NEAR(snap.obstacles[0].x, 1.0 + 0.5 * 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].y, 2.0);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].radius, 0.4);
  EXPECT_EQ(snap.obstacles[0].id, 4242);
  EXPECT_TRUE(snap.fresh);
  EXPECT_NEAR(snap.age_s, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(snap.command_scale, 1.0);

  // Staleness ladder engages purely on sim-time age.
  snap = src.GetObstacles(10.45);
  EXPECT_EQ(snap.state, dra::StalenessState::kDegrade);
  EXPECT_NEAR(snap.command_scale, 0.5, 1e-6);
  ASSERT_EQ(snap.obstacles.size(), 1u);

  snap = src.GetObstacles(11.5);
  EXPECT_EQ(snap.state, dra::StalenessState::kStop);
  EXPECT_DOUBLE_EQ(snap.command_scale, 0.0);
  ASSERT_EQ(snap.obstacles.size(), 1u);  // retained, never emptied
  EXPECT_GT(snap.obstacles[0].radius, 0.4);  // inflated

  // Recovery: a fresh frame restores normal operation instantly.
  ASSERT_TRUE(PublishUntilReceived(pub_, src,
      MakeMsg(11.6, {{3.0, 3.0, 0.3, 0.0, 0.0}})));
  snap = src.GetObstacles(11.65);
  EXPECT_EQ(snap.state, dra::StalenessState::kFresh);
  EXPECT_DOUBLE_EQ(snap.command_scale, 1.0);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].x, 3.0);
}

TEST_F(ObstacleSourceTest, OracleIgnoresTopic) {
  // H-9: in oracle mode the subscription is never created — topic data has
  // no path into the returned set.
  dra::ObstacleSource::Config cfg;
  cfg.mode = Mode::kOracle;
  cfg.topic = "/obstacles_safe_test";
  cfg.oracle = [] {
    return std::vector<dpcbf::ObstacleState>{{7.0, 8.0, 0.25, 0.0, 0.0, 3}};
  };
  dra::ObstacleSource src(cfg);
  EXPECT_EQ(pub_->get_subscription_count(), 0u);

  pub_->publish(MakeMsg(1.0, {{9.0, 9.0, 0.9, 0.0, 0.0}}));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(src.frames_received(), 0u);

  auto snap = src.GetObstacles(2.0);
  ASSERT_EQ(snap.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].x, 7.0);   // oracle, not topic
  EXPECT_TRUE(snap.fresh);
  EXPECT_DOUBLE_EQ(snap.age_s, 0.0);
  EXPECT_DOUBLE_EQ(snap.command_scale, 1.0);
}

TEST_F(ObstacleSourceTest, OraclePassThroughIsExact) {
  // kOracle is a pass-through wrapper: values returned bit-for-bit.
  std::vector<dpcbf::ObstacleState> field;
  for (int i = 0; i < 90; ++i) {
    field.push_back({0.1 * i, -0.2 * i, 0.2 + 0.001 * i, 0.3, -0.4, i});
  }
  dra::ObstacleSource::Config cfg;
  cfg.mode = Mode::kOracle;
  cfg.enable_ros = false;  // fully offline (the T1 replay configuration)
  cfg.oracle = [&field] { return field; };
  dra::ObstacleSource src(cfg);
  auto snap = src.GetObstacles(123.456);
  ASSERT_EQ(snap.obstacles.size(), field.size());
  EXPECT_EQ(0, std::memcmp(snap.obstacles.data(), field.data(),
                           field.size() * sizeof(dpcbf::ObstacleState)));
}

TEST_F(ObstacleSourceTest, ShadowFeedsOracleAndAccumulatesDeltas) {
  dra::ObstacleSource::Config cfg;
  cfg.mode = Mode::kShadow;
  cfg.topic = "/obstacles_safe_test";
  cfg.oracle = [] {
    return std::vector<dpcbf::ObstacleState>{{1.0, 0.0, 0.25, 0.0, 0.0, 0}};
  };
  dra::ObstacleSource src(cfg);
  src.SetRobotXY(0.0, 0.0);

  // Estimated stream: 3 cm off, inflated radius.
  ASSERT_TRUE(PublishUntilReceived(pub_, src,
      MakeMsg(20.0, {{1.03, 0.0, 0.40, 0.0, 0.0}})));

  auto snap = src.GetObstacles(20.05);
  // Filter() gets the ORACLE set, regardless of the estimated stream (H-9).
  ASSERT_EQ(snap.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].x, 1.0);
  EXPECT_DOUBLE_EQ(snap.obstacles[0].radius, 0.25);
  EXPECT_TRUE(snap.fresh);
  EXPECT_DOUBLE_EQ(snap.command_scale, 1.0);

  const auto stats = src.shadow_stats();
  ASSERT_EQ(stats.frames, 1u);
  ASSERT_EQ(stats.matched, 1u);
  EXPECT_NEAR(stats.pos_err_max, 0.03, 1e-9);
  // margin = est.radius - pos_err - oracle.radius = 0.40 - 0.03 - 0.25.
  EXPECT_NEAR(stats.radius_margin_min, 0.12, 1e-9);
}

TEST_F(ObstacleSourceTest, ConstructionEnforcesModeInvariants) {
  // Oracle/shadow require the oracle callback.
  dra::ObstacleSource::Config no_oracle;
  no_oracle.mode = Mode::kOracle;
  EXPECT_THROW(dra::ObstacleSource{no_oracle}, std::invalid_argument);
  no_oracle.mode = Mode::kShadow;
  EXPECT_THROW(dra::ObstacleSource{no_oracle}, std::invalid_argument);
  // Shadow/estimated cannot run without ROS.
  dra::ObstacleSource::Config no_ros;
  no_ros.mode = Mode::kEstimated;
  no_ros.enable_ros = false;
  EXPECT_THROW(dra::ObstacleSource{no_ros}, std::invalid_argument);
  // Mode is per-process-object at construction: the API exposes no setter
  // (compile-time fact) and reports the constructed mode.
  dra::ObstacleSource::Config est;
  est.mode = Mode::kEstimated;
  est.topic = "/obstacles_safe_test";
  dra::ObstacleSource src(est);
  EXPECT_EQ(src.mode(), Mode::kEstimated);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  if (rclcpp::ok()) rclcpp::shutdown();
  return ret;
}
