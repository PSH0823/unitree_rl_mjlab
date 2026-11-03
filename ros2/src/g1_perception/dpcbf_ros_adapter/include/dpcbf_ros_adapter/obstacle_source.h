#pragma once
// ObstacleSource (§10.2): the only bridge between ROS2 topics and
// dpcbf::ObstacleState. Modes (§10.4):
//   kOracle    — pass-through of the injected oracle callback (in simulate:
//                the verbatim baseline DynamicObstacleManager::Snapshot()
//                conversion loop). No topic, no extrapolation, no staleness.
//                Must be bit-identical to baseline behavior (gate T1).
//   kShadow    — oracle feeds Filter(); the /obstacles_safe stream is
//                consumed in parallel and per-query deltas are accumulated
//                and reported (risk-free full-stack evaluation).
//   kEstimated — /obstacles_safe via the wait-free buffer, constant-velocity
//                extrapolation to t_query, §10.3 staleness ladder.
//
// H-9 is enforced structurally: the mode is fixed at construction (no
// setter), kOracle never creates the subscription, kEstimated never invokes
// the oracle callback, and GetObstacles returns exactly one source's set —
// there is no API path that mixes them in one Filter() call.
//
// Time discipline (§11.2 decision, recorded in §21): the adapter node runs
// with use_sim_time=false — inside simulate it must not consume the /clock
// this very process publishes. Wall time paces ONLY the diagnostics timer;
// every safety-relevant age is sim-time-vs-sim-time via GetObstacles'
// t_query argument (caller's d->time) and message header stamps.

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/obstacle_buffer.h"

namespace dpcbf_ros_adapter {

// Query-latency histogram buckets (µs upper bounds; last = overflow).
constexpr double kLatencyBucketsUs[] = {1, 2, 5, 10, 20, 50, 100, 1e9};
constexpr std::size_t kNumLatencyBuckets =
    sizeof(kLatencyBucketsUs) / sizeof(kLatencyBucketsUs[0]);

struct ShadowDeltaStats {
  std::uint64_t frames = 0;          // estimated frames compared
  std::uint64_t matched = 0;         // NN pairs within 0.5 m
  std::uint64_t oracle_unmatched = 0;   // oracle obstacles in p_max w/o pair
  std::uint64_t est_unmatched = 0;      // estimated obstacles w/o pair
  double pos_err_sum = 0.0, pos_err_max = 0.0;
  double vel_err_sum = 0.0, vel_err_max = 0.0;
  double radius_margin_min = 1e9;    // min over pairs of
                                     // (est.radius - pos_err - oracle.radius)
                                     // >0 ⇒ safety circle contains truth
  double age_sum = 0.0, age_max = 0.0;  // estimated-frame age at comparison
};

class ObstacleSource {
 public:
  enum class Mode : std::uint32_t { kOracle = 0, kShadow = 1, kEstimated = 2 };
  using OracleFn = std::function<std::vector<dpcbf::ObstacleState>()>;

  struct Config {
    Mode mode = Mode::kOracle;
    StalenessPolicy staleness;
    std::string topic = "/obstacles_safe";
    std::string diagnostics_topic = "/dpcbf/status";
    double diagnostics_period_s = 0.1;  // 10 Hz (§7.1), wall-clock paced
    OracleFn oracle;                    // required for kOracle / kShadow
    double p_max = 3.0;                 // shadow-delta horizon (dpcbf p_max)
    // When false, no node/executor is created: kOracle works fully offline
    // (T1 replay, unit tests). kShadow/kEstimated require ROS.
    bool enable_ros = true;
  };

  struct Snapshot {
    std::vector<dpcbf::ObstacleState> obstacles;
    bool fresh = true;
    double age_s = 0.0;
    double command_scale = 1.0;
    StalenessState state = StalenessState::kFresh;
  };

  explicit ObstacleSource(Config config);
  ~ObstacleSource();
  ObstacleSource(const ObstacleSource&) = delete;
  ObstacleSource& operator=(const ObstacleSource&) = delete;

  // Wait-free for the 1 kHz caller. t_query_s is SIM time (d->time).
  Snapshot GetObstacles(double t_query_s);

  // Robot position context for shadow deltas / diagnostics (cheap atomic
  // store; call from the seam before GetObstacles).
  void SetRobotXY(double x, double y);

  Mode mode() const;
  // Snapshot of the accumulated shadow statistics (kShadow only).
  ShadowDeltaStats shadow_stats() const;
  // Query-latency histogram counts (kNumLatencyBuckets entries).
  std::vector<std::uint64_t> latency_histogram() const;
  std::uint64_t frames_received() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dpcbf_ros_adapter
