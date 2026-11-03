// Unit tests for the ROS-free adapter core: extrapolation math, the §10.3
// staleness state machine (all three regimes + boundary times), the wait-free
// double buffer under concurrent publish/query stress, and uid→id edges.
#include "dpcbf_ros_adapter/obstacle_buffer.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <thread>

#include "dpcbf_ros_adapter/axis_command_map.h"

namespace dra = dpcbf_ros_adapter;

namespace {

dra::ObstacleFrame MakeFrame(double stamp,
                             std::vector<dpcbf::ObstacleState> obs) {
  dra::ObstacleFrame f;
  f.stamp = stamp;
  f.count = obs.size();
  for (std::size_t i = 0; i < obs.size(); ++i) f.obstacles[i] = obs[i];
  return f;
}

}  // namespace

TEST(StalenessPolicy, RegimesAndBoundaries) {
  dra::StalenessPolicy p;  // Appendix-A defaults 0.30 / 0.30 / 1.0
  // Fresh regime, inclusive upper boundary.
  EXPECT_EQ(p.Classify(0.0), dra::StalenessState::kFresh);
  EXPECT_EQ(p.Classify(0.29), dra::StalenessState::kFresh);
  EXPECT_EQ(p.Classify(0.30), dra::StalenessState::kFresh);
  EXPECT_DOUBLE_EQ(p.CommandScale(0.30), 1.0);
  // Degrade: linear ramp on (0.30, 0.60].
  EXPECT_EQ(p.Classify(0.301), dra::StalenessState::kDegrade);
  EXPECT_NEAR(p.CommandScale(0.45), 0.5, 1e-12);
  EXPECT_EQ(p.Classify(0.60), dra::StalenessState::kDegrade);
  EXPECT_NEAR(p.CommandScale(0.599), 1.0 - 0.299 / 0.30, 1e-9);
  EXPECT_DOUBLE_EQ(p.CommandScale(0.60), 0.0);
  // Stop beyond fade-out.
  EXPECT_EQ(p.Classify(0.601), dra::StalenessState::kStop);
  EXPECT_DOUBLE_EQ(p.CommandScale(0.601), 0.0);
  EXPECT_DOUBLE_EQ(p.CommandScale(5.0), 0.0);
}

TEST(Materialize, ExtrapolationFreshRegime) {
  dra::StalenessPolicy p;
  auto f = MakeFrame(10.0, {{1.0, 2.0, 0.3, 0.5, -0.25, 7}});
  dra::MaterializedSnapshot s;
  dra::Materialize(f, 10.2, p, &s);
  ASSERT_EQ(s.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(s.obstacles[0].x, 1.0 + 0.5 * 0.2);
  EXPECT_DOUBLE_EQ(s.obstacles[0].y, 2.0 - 0.25 * 0.2);
  EXPECT_DOUBLE_EQ(s.obstacles[0].radius, 0.3);  // no inflation while fresh
  EXPECT_DOUBLE_EQ(s.obstacles[0].velocity_x, 0.5);
  EXPECT_EQ(s.obstacles[0].id, 7);
  EXPECT_TRUE(s.fresh);
  EXPECT_NEAR(s.age_s, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(s.command_scale, 1.0);
  EXPECT_EQ(s.state, dra::StalenessState::kFresh);
}

TEST(Materialize, QueryBeforeStampClampsToZeroAge) {
  dra::StalenessPolicy p;
  auto f = MakeFrame(10.0, {{1.0, 2.0, 0.3, 0.5, 0.5, 1}});
  dra::MaterializedSnapshot s;
  dra::Materialize(f, 9.99, p, &s);
  EXPECT_DOUBLE_EQ(s.obstacles[0].x, 1.0);  // never extrapolates backwards
  EXPECT_DOUBLE_EQ(s.age_s, 0.0);
  EXPECT_TRUE(s.fresh);
}

TEST(Materialize, DegradeExtrapolatesWithoutInflation) {
  dra::StalenessPolicy p;
  auto f = MakeFrame(0.0, {{0.0, 0.0, 0.25, 1.0, 0.0, 0}});
  dra::MaterializedSnapshot s;
  dra::Materialize(f, 0.45, p, &s);
  EXPECT_EQ(s.state, dra::StalenessState::kDegrade);
  EXPECT_FALSE(s.fresh);
  EXPECT_DOUBLE_EQ(s.obstacles[0].x, 0.45);
  EXPECT_DOUBLE_EQ(s.obstacles[0].radius, 0.25);
  EXPECT_NEAR(s.command_scale, 0.5, 1e-12);
}

TEST(Materialize, StopRetainsCapsAndInflates) {
  dra::StalenessPolicy p;
  auto f = MakeFrame(0.0, {{0.0, 0.0, 0.25, 1.0, 0.0, 0}});
  dra::MaterializedSnapshot s;
  // age 0.8: extrapolation capped at 0.6, radius grows by |v|*0.8.
  dra::Materialize(f, 0.8, p, &s);
  EXPECT_EQ(s.state, dra::StalenessState::kStop);
  EXPECT_DOUBLE_EQ(s.obstacles[0].x, 0.6);
  EXPECT_NEAR(s.obstacles[0].radius, 0.25 + 1.0 * 0.8, 1e-12);
  EXPECT_DOUBLE_EQ(s.command_scale, 0.0);
  ASSERT_FALSE(s.obstacles.empty());  // empty-on-stale is forbidden (§10.3)
  // age 2.5: inflation capped at hold_after_stale (1.0 s), set retained.
  dra::Materialize(f, 2.5, p, &s);
  ASSERT_EQ(s.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(s.obstacles[0].x, 0.6);
  EXPECT_NEAR(s.obstacles[0].radius, 0.25 + 1.0 * 1.0, 1e-12);
}

TEST(Materialize, NoDataIsStopWithEmptySet) {
  dra::StalenessPolicy p;
  dra::ObstacleFrame f;  // stamp < 0
  dra::MaterializedSnapshot s;
  dra::Materialize(f, 123.0, p, &s);
  EXPECT_EQ(s.state, dra::StalenessState::kNoData);
  EXPECT_TRUE(s.obstacles.empty());
  EXPECT_FALSE(s.fresh);
  EXPECT_DOUBLE_EQ(s.command_scale, 0.0);
  EXPECT_TRUE(std::isinf(s.age_s));
}

TEST(UidToId, Edges) {
  EXPECT_EQ(dra::UidToId(0), 0);
  EXPECT_EQ(dra::UidToId(41), 41);
  EXPECT_EQ(dra::UidToId(0x7fffffffull), 0x7fffffff);
  EXPECT_EQ(dra::UidToId(0x80000000ull), 0);          // wraps, stays >= 0
  EXPECT_EQ(dra::UidToId(0x80000001ull), 1);
  EXPECT_EQ(dra::UidToId(0xffffffffffffffffull), 0x7fffffff);
  EXPECT_GE(dra::UidToId(0xdeadbeefcafebabeull), 0);
}

TEST(ObstacleBuffer, ReadBeforeAnyPublish) {
  dra::ObstacleBuffer buf;
  dra::ObstacleFrame out;
  EXPECT_FALSE(buf.Read(&out));
  EXPECT_LT(out.stamp, 0.0);
}

TEST(ObstacleBuffer, PublishReadRoundTrip) {
  dra::ObstacleBuffer buf;
  buf.Publish(MakeFrame(1.5, {{1, 2, 0.3, 4, 5, 6}, {7, 8, 0.9, 1, 2, 3}}));
  dra::ObstacleFrame out;
  ASSERT_TRUE(buf.Read(&out));
  EXPECT_DOUBLE_EQ(out.stamp, 1.5);
  ASSERT_EQ(out.count, 2u);
  EXPECT_DOUBLE_EQ(out.obstacles[1].x, 7.0);
  EXPECT_EQ(out.obstacles[1].id, 3);
  // Newest frame wins.
  buf.Publish(MakeFrame(2.0, {{9, 9, 0.1, 0, 0, 1}}));
  ASSERT_TRUE(buf.Read(&out));
  EXPECT_DOUBLE_EQ(out.stamp, 2.0);
  EXPECT_EQ(out.count, 1u);
}

// Frame-consistency invariant under concurrent pub/query: every read must
// return exactly one published frame (stamp k encodes payload k — torn reads
// would mix stamps/payloads and fail the checks).
TEST(ObstacleBuffer, ConcurrentStress) {
  dra::ObstacleBuffer buf;
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> reads{0};
  std::thread writer([&] {
    for (std::uint64_t k = 1; !stop.load(std::memory_order_relaxed); ++k) {
      dra::ObstacleFrame f;
      f.stamp = static_cast<double>(k);
      f.count = 1 + (k % (dra::kMaxObstacles - 1));
      for (std::size_t i = 0; i < f.count; ++i) {
        f.obstacles[i].x = static_cast<double>(k);
        f.obstacles[i].y = static_cast<double>(i);
        f.obstacles[i].id = static_cast<int>(k % 1000);
      }
      buf.Publish(f);
    }
  });
  std::thread reader([&] {
    dra::ObstacleFrame out;
    while (!stop.load(std::memory_order_relaxed)) {
      if (!buf.Read(&out)) continue;
      const auto k = static_cast<std::uint64_t>(out.stamp);
      ASSERT_EQ(out.count, 1 + (k % (dra::kMaxObstacles - 1)));
      for (std::size_t i = 0; i < out.count; i += 17) {
        ASSERT_DOUBLE_EQ(out.obstacles[i].x, static_cast<double>(k));
        ASSERT_DOUBLE_EQ(out.obstacles[i].y, static_cast<double>(i));
        ASSERT_EQ(out.obstacles[i].id, static_cast<int>(k % 1000));
      }
      reads.fetch_add(1, std::memory_order_relaxed);
    }
  });
  std::this_thread::sleep_for(std::chrono::seconds(2));
  stop.store(true);
  writer.join();
  reader.join();
  EXPECT_GT(reads.load(), 10000u);
}

TEST(AxisCommandMap, RoundTripMatchesBaselineSemantics) {
  // Same asymmetric bounds as dpcbf_config.yaml v_s: [-1, 2].
  const double vmin = -1.0, vmax = 2.0;
  EXPECT_DOUBLE_EQ(dra::AxisToCommand(0.5, vmin, vmax), 1.0);
  EXPECT_DOUBLE_EQ(dra::AxisToCommand(-0.5, vmin, vmax), -0.5);
  EXPECT_DOUBLE_EQ(dra::AxisToCommand(1.5, vmin, vmax), 2.0);   // clamped
  EXPECT_FLOAT_EQ(dra::CommandToAxis(1.0, vmin, vmax), 0.5f);
  EXPECT_FLOAT_EQ(dra::CommandToAxis(-0.5, vmin, vmax), -0.5f);
  EXPECT_FLOAT_EQ(dra::CommandToAxis(0.7, 0.0, 0.0), 0.0f);     // degenerate
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
