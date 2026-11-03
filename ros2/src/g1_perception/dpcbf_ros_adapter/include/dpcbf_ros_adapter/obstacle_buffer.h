#pragma once
// Pure (ROS-free) core of the adapter: the wait-free single-writer /
// single-reader obstacle frame buffer, constant-velocity extrapolation, the
// §10.3 staleness ladder, and the uid→id mapping (D6 addendum). Everything
// here is unit-testable without rclcpp; ObstacleSource wraps it with the
// subscription and diagnostics.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"  // dpcbf::ObstacleState (header-only)

namespace dpcbf_ros_adapter {

// Tracker load measured in Phase 3 peaked at 22 simultaneous tracks on the
// 90-obstacle live field; 128 is a comfortable hard cap (overflow counted,
// never UB).
constexpr std::size_t kMaxObstacles = 128;

struct ObstacleFrame {
  double stamp = -1.0;  // sim-time seconds from header.stamp; <0 = no data yet
  std::size_t count = 0;
  std::array<dpcbf::ObstacleState, kMaxObstacles> obstacles{};
};

// §10.3 regimes. kNoData = no frame ever received (startup): treated like the
// fail-safe stop, with an EMPTY set — there is no "last known set" to retain,
// and the command scale of 0 keeps the robot stopped until perception is up.
enum class StalenessState : std::uint32_t {
  kFresh = 0,        // age <= max_age
  kDegrade = 1,      // max_age < age <= max_age + fade_out: command ramps to 0
  kStop = 2,         // age beyond fade_out: command 0, retained set inflated
  kNoData = 3,       // never received a frame
};

struct StalenessPolicy {
  double max_age_s = 0.30;
  double fade_out_s = 0.30;
  double hold_after_stale_s = 1.0;  // inflation growth cap (§10.3)

  StalenessState Classify(double age_s) const {
    if (age_s <= max_age_s) return StalenessState::kFresh;
    if (age_s <= max_age_s + fade_out_s) return StalenessState::kDegrade;
    return StalenessState::kStop;
  }

  // 1 while fresh, linear ramp to 0 across the fade-out window, 0 after.
  // Applied by the CALL SITE to the desired command before Filter() (§10.5).
  double CommandScale(double age_s) const {
    if (age_s <= max_age_s) return 1.0;
    if (age_s >= max_age_s + fade_out_s) return 0.0;
    return 1.0 - (age_s - max_age_s) / fade_out_s;
  }
};

// uid → ObstacleState::id (D6 addendum, Phase-4 §0). CircleObstacle.uid is
// uint64 (monotonic per-track counter); id is a 32-bit int that DPCBF treats
// as informational only (§2.3-6). Policy: wrap into the non-negative int31
// range so ids never go negative (−1 means "unset" downstream) and the
// mapping is total. Collisions require 2^31 track births in one session.
inline int UidToId(std::uint64_t uid) {
  return static_cast<int>(uid & 0x7fffffffu);
}

struct MaterializedSnapshot {
  std::vector<dpcbf::ObstacleState> obstacles;
  bool fresh = false;
  double age_s = std::numeric_limits<double>::infinity();
  double command_scale = 0.0;
  StalenessState state = StalenessState::kNoData;
};

// Extrapolation + staleness ladder applied to a frame at query time.
// Deliberate choices (recorded in §21):
//  - ages are sim-time-vs-sim-time: t_query_s is the caller's d->time, stamp
//    comes from header.stamp — no wall clock anywhere in this path;
//  - position extrapolation dt is capped at max_age+fade_out: beyond the
//    stop boundary the last set is RETAINED at its 0.6 s extrapolation point
//    ("retaining the last set", §10.3) instead of chasing a stale velocity;
//  - in the stop regime radii grow by |v|·min(age, hold_after_stale)
//    (k_v = 1, the H-8 velocity-inflation kernel with age as the horizon);
//  - obstacles-empty-on-stale is forbidden: the set is retained indefinitely
//    (inflation capped at hold_after_stale) — the command is already 0.
inline void Materialize(const ObstacleFrame& frame, double t_query_s,
                        const StalenessPolicy& policy,
                        MaterializedSnapshot* out) {
  out->obstacles.clear();
  if (frame.stamp < 0.0) {
    out->fresh = false;
    out->age_s = std::numeric_limits<double>::infinity();
    out->command_scale = 0.0;
    out->state = StalenessState::kNoData;
    return;
  }
  // A stamp minimally ahead of the query (pipeline stages share sim time but
  // sample it asynchronously) is treated as age 0, never negative dt.
  const double age = std::max(0.0, t_query_s - frame.stamp);
  const StalenessState state = policy.Classify(age);
  const double dt_extrap =
      std::min(age, policy.max_age_s + policy.fade_out_s);
  const double inflate_horizon =
      state == StalenessState::kStop
          ? std::min(age, policy.hold_after_stale_s)
          : 0.0;
  out->obstacles.reserve(frame.count);
  for (std::size_t i = 0; i < frame.count; ++i) {
    dpcbf::ObstacleState o = frame.obstacles[i];
    o.x += o.velocity_x * dt_extrap;
    o.y += o.velocity_y * dt_extrap;
    if (inflate_horizon > 0.0) {
      o.radius += std::hypot(o.velocity_x, o.velocity_y) * inflate_horizon;
    }
    out->obstacles.push_back(o);
  }
  out->fresh = state == StalenessState::kFresh;
  out->age_s = age;
  out->command_scale = policy.CommandScale(age);
  out->state = state;
}

// Wait-free single-writer (executor thread, ~10 Hz) / single-reader (1 kHz
// bridge thread) frame exchange: double buffer, each slot seqlock-guarded.
// The reader never blocks; a retry can only happen if the writer laps the
// reader's slot mid-copy, which at 10 Hz writes vs µs reads requires the
// reader to stall >100 ms inside the copy.
class ObstacleBuffer {
 public:
  void Publish(const ObstacleFrame& frame) {
    const std::uint32_t next = 1u - front_.load(std::memory_order_relaxed);
    Slot& slot = slots_[next];
    slot.seq.fetch_add(1, std::memory_order_release);  // odd: writing
    slot.frame.stamp = frame.stamp;
    slot.frame.count = frame.count < kMaxObstacles ? frame.count
                                                   : kMaxObstacles;
    for (std::size_t i = 0; i < slot.frame.count; ++i) {
      slot.frame.obstacles[i] = frame.obstacles[i];
    }
    slot.seq.fetch_add(1, std::memory_order_release);  // even: done
    front_.store(next, std::memory_order_release);
    frames_published_.fetch_add(1, std::memory_order_relaxed);
  }

  // Copies the newest consistent frame into *out. Returns false only if no
  // frame has ever been published (out->stamp stays < 0).
  bool Read(ObstacleFrame* out) const {
    for (;;) {
      const std::uint32_t f = front_.load(std::memory_order_acquire);
      const Slot& slot = slots_[f];
      const std::uint32_t s1 = slot.seq.load(std::memory_order_acquire);
      if (s1 == 0u) return false;      // never written
      if (s1 & 1u) continue;           // writer active on this slot
      out->stamp = slot.frame.stamp;
      const std::size_t n = slot.frame.count < kMaxObstacles
                                ? slot.frame.count : kMaxObstacles;
      for (std::size_t i = 0; i < n; ++i) {
        out->obstacles[i] = slot.frame.obstacles[i];
      }
      out->count = n;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (slot.seq.load(std::memory_order_acquire) == s1 &&
          front_.load(std::memory_order_acquire) == f) {
        return true;
      }
    }
  }

  std::uint64_t frames_published() const {
    return frames_published_.load(std::memory_order_relaxed);
  }

 private:
  struct Slot {
    std::atomic<std::uint32_t> seq{0};
    ObstacleFrame frame;
  };
  Slot slots_[2];
  std::atomic<std::uint32_t> front_{0};
  std::atomic<std::uint64_t> frames_published_{0};
};

}  // namespace dpcbf_ros_adapter
