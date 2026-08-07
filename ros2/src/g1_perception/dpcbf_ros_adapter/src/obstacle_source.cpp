#include "dpcbf_ros_adapter/obstacle_source.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dpcbf_ros_adapter {

namespace {

double StampToSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

const char* StateName(StalenessState s) {
  switch (s) {
    case StalenessState::kFresh: return "fresh";
    case StalenessState::kDegrade: return "degrade";
    case StalenessState::kStop: return "stop";
    case StalenessState::kNoData: return "no_data";
  }
  return "?";
}

const char* ModeName(ObstacleSource::Mode m) {
  switch (m) {
    case ObstacleSource::Mode::kOracle: return "oracle";
    case ObstacleSource::Mode::kShadow: return "shadow";
    case ObstacleSource::Mode::kEstimated: return "estimated";
  }
  return "?";
}

}  // namespace

struct ObstacleSource::Impl {
  Config config;

  // Estimated-stream state (kShadow / kEstimated).
  ObstacleBuffer buffer;
  std::atomic<std::uint64_t> dropped_circles{0};

  // Query-side instrumentation (written by the 1 kHz caller only).
  std::atomic<std::uint64_t> latency_buckets[kNumLatencyBuckets] = {};
  std::atomic<std::uint64_t> query_count{0};
  std::atomic<double> last_age{-1.0};
  std::atomic<double> last_scale{1.0};
  std::atomic<std::uint32_t> last_state{
      static_cast<std::uint32_t>(StalenessState::kFresh)};
  std::atomic<double> robot_x{0.0}, robot_y{0.0};

  // Shadow deltas: computed on the bridge thread only when a new estimated
  // frame has arrived (≈10 Hz); the diagnostics thread copies under a mutex
  // the bridge thread only try_locks — never a blocking wait at 1 kHz.
  std::uint64_t shadow_seen_frames = 0;
  ShadowDeltaStats shadow;      // guarded by shadow_mutex
  ShadowDeltaStats shadow_pending;  // bridge-thread local accumulator
  std::mutex shadow_mutex;
  std::atomic<std::uint64_t> shadow_flush_misses{0};

  // Scratch (bridge thread only — GetObstacles is single-caller by design).
  ObstacleFrame scratch_frame;
  MaterializedSnapshot scratch_snapshot;

  // ROS plumbing.
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
  rclcpp::TimerBase::SharedPtr diag_timer;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;
  std::atomic<bool> shutdown{false};

  void OnObstacles(const obstacle_detector::msg::Obstacles& msg) {
    ObstacleFrame frame;
    frame.stamp = StampToSec(msg.header.stamp);
    std::size_t n = 0;
    for (const auto& c : msg.circles) {
      if (n >= kMaxObstacles) {
        dropped_circles.fetch_add(msg.circles.size() - n,
                                  std::memory_order_relaxed);
        break;
      }
      auto& o = frame.obstacles[n++];
      o.x = c.center.x;
      o.y = c.center.y;
      o.radius = c.radius;
      o.velocity_x = c.velocity.x;
      o.velocity_y = c.velocity.y;
      o.id = UidToId(c.uid);
    }
    frame.count = n;
    buffer.Publish(frame);
  }

  void PublishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = node->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "dpcbf_ros_adapter";
    st.hardware_id = ModeName(config.mode);
    const auto state =
        static_cast<StalenessState>(last_state.load(std::memory_order_relaxed));
    const double age = last_age.load(std::memory_order_relaxed);
    if (config.mode == Mode::kOracle || state == StalenessState::kFresh) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (state == StalenessState::kDegrade) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    st.message = StateName(state);
    auto add = [&st](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    };
    auto num = [](double v) {
      std::ostringstream os;
      os << v;
      return os.str();
    };
    add("mode", ModeName(config.mode));
    add("staleness_state", StateName(state));
    add("age_s", num(age));
    add("command_scale", num(last_scale.load(std::memory_order_relaxed)));
    add("frames_received", num(static_cast<double>(buffer.frames_published())));
    add("dropped_circles",
        num(static_cast<double>(dropped_circles.load(std::memory_order_relaxed))));
    add("query_count",
        num(static_cast<double>(query_count.load(std::memory_order_relaxed))));
    // Histogram + coarse percentiles from bucket counts.
    std::uint64_t total = 0;
    std::uint64_t counts[kNumLatencyBuckets];
    for (std::size_t i = 0; i < kNumLatencyBuckets; ++i) {
      counts[i] = latency_buckets[i].load(std::memory_order_relaxed);
      total += counts[i];
    }
    std::ostringstream hist;
    for (std::size_t i = 0; i < kNumLatencyBuckets; ++i) {
      if (i) hist << ' ';
      hist << "le" << kLatencyBucketsUs[i] << "us:" << counts[i];
    }
    add("query_latency_histogram", hist.str());
    auto pct = [&](double q) -> double {
      if (total == 0) return 0.0;
      std::uint64_t need =
          static_cast<std::uint64_t>(q * static_cast<double>(total));
      std::uint64_t acc = 0;
      for (std::size_t i = 0; i < kNumLatencyBuckets; ++i) {
        acc += counts[i];
        if (acc > need) return kLatencyBucketsUs[i];
      }
      return kLatencyBucketsUs[kNumLatencyBuckets - 1];
    };
    add("query_latency_p50_le_us", num(pct(0.50)));
    add("query_latency_p99_le_us", num(pct(0.99)));
    if (config.mode == Mode::kShadow) {
      ShadowDeltaStats s;
      {
        std::lock_guard<std::mutex> lock(shadow_mutex);
        s = shadow;
      }
      add("shadow_frames", num(static_cast<double>(s.frames)));
      add("shadow_matched", num(static_cast<double>(s.matched)));
      add("shadow_oracle_unmatched",
          num(static_cast<double>(s.oracle_unmatched)));
      add("shadow_est_unmatched", num(static_cast<double>(s.est_unmatched)));
      if (s.matched > 0) {
        add("shadow_pos_err_mean_m",
            num(s.pos_err_sum / static_cast<double>(s.matched)));
        add("shadow_pos_err_max_m", num(s.pos_err_max));
        add("shadow_vel_err_mean_mps",
            num(s.vel_err_sum / static_cast<double>(s.matched)));
        add("shadow_vel_err_max_mps", num(s.vel_err_max));
        add("shadow_radius_margin_min_m", num(s.radius_margin_min));
      }
      if (s.frames > 0) {
        add("shadow_age_mean_s", num(s.age_sum / static_cast<double>(s.frames)));
        add("shadow_age_max_s", num(s.age_max));
      }
    }
    arr.status.push_back(st);
    diag_pub->publish(arr);
  }

  // Shadow comparison: oracle (return set) vs estimated (materialized to the
  // same t_query). Greedy nearest-neighbor within 0.5 m (§17.2), restricted
  // to oracle obstacles within p_max of the robot.
  void AccumulateShadowDelta(const std::vector<dpcbf::ObstacleState>& oracle,
                             const MaterializedSnapshot& est) {
    const double px = robot_x.load(std::memory_order_relaxed);
    const double py = robot_y.load(std::memory_order_relaxed);
    const double p_max2 = config.p_max * config.p_max;
    ShadowDeltaStats& s = shadow_pending;
    s.frames += 1;
    s.age_sum += est.age_s;
    s.age_max = std::max(s.age_max, est.age_s);
    std::vector<bool> est_used(est.obstacles.size(), false);
    std::size_t est_in_scope = 0;
    for (const auto& e : est.obstacles) {
      const double dx = e.x - px, dy = e.y - py;
      if (dx * dx + dy * dy <= p_max2) ++est_in_scope;
    }
    std::size_t matched_est = 0;
    for (const auto& o : oracle) {
      const double dxr = o.x - px, dyr = o.y - py;
      if (dxr * dxr + dyr * dyr > p_max2) continue;
      double best = 0.5 * 0.5;
      int best_i = -1;
      for (std::size_t i = 0; i < est.obstacles.size(); ++i) {
        if (est_used[i]) continue;
        const double dx = est.obstacles[i].x - o.x;
        const double dy = est.obstacles[i].y - o.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 <= best) {
          best = d2;
          best_i = static_cast<int>(i);
        }
      }
      if (best_i < 0) {
        s.oracle_unmatched += 1;
        continue;
      }
      est_used[best_i] = true;
      const auto& e = est.obstacles[best_i];
      const double pos_err = std::hypot(e.x - o.x, e.y - o.y);
      const double vel_err =
          std::hypot(e.velocity_x - o.velocity_x, e.velocity_y - o.velocity_y);
      const double margin = e.radius - pos_err - o.radius;
      s.matched += 1;
      s.pos_err_sum += pos_err;
      s.pos_err_max = std::max(s.pos_err_max, pos_err);
      s.vel_err_sum += vel_err;
      s.vel_err_max = std::max(s.vel_err_max, vel_err);
      s.radius_margin_min = std::min(s.radius_margin_min, margin);
      const double dxe = e.x - px, dye = e.y - py;
      if (dxe * dxe + dye * dye <= p_max2) ++matched_est;
    }
    s.est_unmatched += est_in_scope - std::min(est_in_scope, matched_est);
    // Hand off to the diagnostics thread without ever blocking at 1 kHz.
    if (shadow_mutex.try_lock()) {
      MergeShadow(shadow, s);
      shadow_mutex.unlock();
      s = ShadowDeltaStats{};
    } else {
      shadow_flush_misses.fetch_add(1, std::memory_order_relaxed);
    }
  }

  static void MergeShadow(ShadowDeltaStats& into, const ShadowDeltaStats& s) {
    into.frames += s.frames;
    into.matched += s.matched;
    into.oracle_unmatched += s.oracle_unmatched;
    into.est_unmatched += s.est_unmatched;
    into.pos_err_sum += s.pos_err_sum;
    into.pos_err_max = std::max(into.pos_err_max, s.pos_err_max);
    into.vel_err_sum += s.vel_err_sum;
    into.vel_err_max = std::max(into.vel_err_max, s.vel_err_max);
    into.radius_margin_min =
        std::min(into.radius_margin_min, s.radius_margin_min);
    into.age_sum += s.age_sum;
    into.age_max = std::max(into.age_max, s.age_max);
  }
};

ObstacleSource::ObstacleSource(Config config) : impl_(new Impl) {
  impl_->config = std::move(config);
  const auto& cfg = impl_->config;
  if (cfg.mode != Mode::kEstimated && !cfg.oracle) {
    throw std::invalid_argument(
        "ObstacleSource: oracle callback required for oracle/shadow mode");
  }
  if (cfg.mode != Mode::kOracle && !cfg.enable_ros) {
    throw std::invalid_argument(
        "ObstacleSource: shadow/estimated modes require ROS");
  }
  if (!cfg.enable_ros) return;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions opts;
  // §11.2 decision: never consume our own /clock (see header).
  opts.parameter_overrides({{"use_sim_time", false}});
  impl_->node = rclcpp::Node::make_shared("dpcbf_ros_adapter", opts);
  impl_->diag_pub =
      impl_->node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          cfg.diagnostics_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  if (cfg.mode != Mode::kOracle) {
    // /obstacles_safe is Reliable depth 1 — latest wins (§7.1).
    impl_->sub =
        impl_->node->create_subscription<obstacle_detector::msg::Obstacles>(
            cfg.topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            // ConstSharedPtr, not `const Obstacles&`: rclcpp accepts
            // const-reference subscription callbacks only from Galactic on,
            // and the G1's onboard computer runs Foxy. Delivery, QoS and
            // OnObstacles() itself are unchanged.
            [impl = impl_.get()](
                obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
              impl->OnObstacles(*m);
            });
  }
  impl_->diag_timer = impl_->node->create_wall_timer(
      std::chrono::duration<double>(cfg.diagnostics_period_s),
      [impl = impl_.get()] { impl->PublishDiagnostics(); });
  impl_->executor =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor->add_node(impl_->node);
  impl_->spin_thread = std::thread([impl = impl_.get()] {
    while (!impl->shutdown.load(std::memory_order_relaxed) && rclcpp::ok()) {
      impl->executor->spin_some(std::chrono::milliseconds(50));
    }
  });
}

ObstacleSource::~ObstacleSource() {
  impl_->shutdown.store(true);
  if (impl_->spin_thread.joinable()) impl_->spin_thread.join();
}

ObstacleSource::Mode ObstacleSource::mode() const { return impl_->config.mode; }

void ObstacleSource::SetRobotXY(double x, double y) {
  impl_->robot_x.store(x, std::memory_order_relaxed);
  impl_->robot_y.store(y, std::memory_order_relaxed);
}

ObstacleSource::Snapshot ObstacleSource::GetObstacles(double t_query_s) {
  const auto t0 = std::chrono::steady_clock::now();
  Snapshot snap;
  Impl& im = *impl_;
  switch (im.config.mode) {
    case Mode::kOracle:
      // Pass-through: no topic, no extrapolation, no staleness (§10.4). Any
      // transformation here is a T1 failure waiting.
      snap.obstacles = im.config.oracle();
      break;
    case Mode::kShadow: {
      snap.obstacles = im.config.oracle();
      const std::uint64_t frames = im.buffer.frames_published();
      if (frames != im.shadow_seen_frames) {
        im.shadow_seen_frames = frames;
        if (im.buffer.Read(&im.scratch_frame)) {
          Materialize(im.scratch_frame, t_query_s, im.config.staleness,
                      &im.scratch_snapshot);
          im.AccumulateShadowDelta(snap.obstacles, im.scratch_snapshot);
          im.last_age.store(im.scratch_snapshot.age_s,
                            std::memory_order_relaxed);
        }
      }
      break;
    }
    case Mode::kEstimated: {
      im.buffer.Read(&im.scratch_frame);
      Materialize(im.scratch_frame, t_query_s, im.config.staleness,
                  &im.scratch_snapshot);
      snap.obstacles = std::move(im.scratch_snapshot.obstacles);
      im.scratch_snapshot.obstacles = std::vector<dpcbf::ObstacleState>();
      snap.fresh = im.scratch_snapshot.fresh;
      snap.age_s = im.scratch_snapshot.age_s;
      snap.command_scale = im.scratch_snapshot.command_scale;
      snap.state = im.scratch_snapshot.state;
      im.last_age.store(snap.age_s, std::memory_order_relaxed);
      im.last_scale.store(snap.command_scale, std::memory_order_relaxed);
      im.last_state.store(static_cast<std::uint32_t>(snap.state),
                          std::memory_order_relaxed);
      break;
    }
  }
  im.query_count.fetch_add(1, std::memory_order_relaxed);
  const double us = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
  for (std::size_t i = 0; i < kNumLatencyBuckets; ++i) {
    if (us <= kLatencyBucketsUs[i]) {
      im.latency_buckets[i].fetch_add(1, std::memory_order_relaxed);
      break;
    }
  }
  return snap;
}

ShadowDeltaStats ObstacleSource::shadow_stats() const {
  std::lock_guard<std::mutex> lock(impl_->shadow_mutex);
  return impl_->shadow;
}

std::vector<std::uint64_t> ObstacleSource::latency_histogram() const {
  std::vector<std::uint64_t> out(kNumLatencyBuckets);
  for (std::size_t i = 0; i < kNumLatencyBuckets; ++i) {
    out[i] = impl_->latency_buckets[i].load(std::memory_order_relaxed);
  }
  return out;
}

std::uint64_t ObstacleSource::frames_received() const {
  return impl_->buffer.frames_published();
}

}  // namespace dpcbf_ros_adapter
