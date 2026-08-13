#pragma once
// Pure gating + inflation rules (§9.6, H-8) — ROS-message-typed but
// node-free, so every rule is unit-testable without an executor.
//
// Per circle, in order:
//   1. Gate: drop the whole message's circles if now − header.stamp >
//      max_age (all circles in a tracker message share the measurement
//      stamp); drop a circle if true_radius > max_circle_radius (spurious
//      wall arcs); clamp r ← max(true_radius, min_radius).
//   2. Speed sanity: clamp |v| to v_max_obstacle (direction preserved) —
//      BEFORE inflation, so a KF spike can neither aim a constraint wrongly
//      nor inflate a radius absurdly.
//   3. Inflate: radius = r + fixed_inflation + k_sigma·sigma
//                          + |v_clamped|·latency_horizon.
//
// The k_sigma term (H-8, §9.6) is DEFAULT-DISABLED (use_covariance=false) and
// config-gated: the shipped behaviour stays the Phase-4-calibrated fixed
// inflation. It consumes CircleObstacle.covariance = [var_x, var_y, var_r],
// published by the tracker under fork patch 0007 (P-3); producers that are not
// the tracker (the oracle source, hand-built test messages) leave it zero, so
// the term self-disables rather than mis-firing.
//
// true_radius and uid pass through untouched; segments are NOT forwarded
// (DPCBF consumes circles only, §7.3); the output header keeps the INPUT
// stamp so downstream staleness accounting (§10.3) is measured from the
// measurement time, not from this node's processing time.

#include <cmath>

#include <obstacle_detector/msg/obstacles.hpp>

namespace safety_obstacle_filter {

struct Params {
  double max_age = 0.30;            // s
  double min_radius = 0.20;         // m
  double max_circle_radius = 0.60;  // m
  double fixed_inflation = 0.03;    // m (Appendix A start; Phase-4 calibrated)
  double latency_horizon = 0.12;    // s
  double v_max_obstacle = 1.5;      // m/s
  // --- P-3 covariance inflation (§9.6 step 2's sigma term, H-8) ------------
  // OFF by default: k_sigma's only calibrated value (2.748) comes from the
  // abandoned branch and is a placeholder until 5B's residual/covariance
  // capture. Turning this on is a joint recalibration of (fixed_inflation,
  // k_sigma), not a change to k_sigma alone — the fixed term already covers
  // the steady-state circle-fit bias the sigma term would double-count.
  bool use_covariance = false;
  double k_sigma = 2.748;           // multiplier on sigma
  double sigma_max = 0.50;          // m, cap: a diverged track must not inflate
                                    // without bound (it would swallow the arena)
};

// Largest 1-sigma obstacle-surface uncertainty that can plausibly come out of
// a Mid-360 plus a circle fit (gap G1). sqrt(P(0,0)) IS metres — the tracker's
// KF has C = [1 0] and measures centre coordinates in metres, so R, P and this
// sigma are all m^2 / m by construction. What can be wrong is the VALUE:
// `measurement_variance` seeds P(0,0) directly, so the inherited 1.0 asserts a
// 1 m 1-sigma LiDAR measurement and produces a ~0.58 m sigma that k_sigma then
// silently absorbs. Anything past this bound means R was never set from data.
inline constexpr double kSigmaPlausibleMax = 0.25;  // m

// Counters for every path on which this filter changes or discards an obstacle
// without saying so. A dropped obstacle is invisible to DPCBF and nothing
// downstream can compensate for a constraint that was never generated, so the
// node logs these; see also fork patch 0009 for the same rule upstream.
struct Stats {
  long stale_messages = 0;      // whole message gated on max_age
  long dropped_large_radius = 0;
  long sigma_clamped = 0;       // sigma hit sigma_max
  long sigma_implausible = 0;   // sigma > kSigmaPlausibleMax: R not calibrated
  double sigma_max_seen = 0.0;  // m
  double radius_max_dropped = 0.0;  // m
};

// Positional + radial 1-sigma of the reported obstacle SURFACE, from the
// tracker's posterior variances, in METRES. The containment quantity is the
// distance from the true surface to the reported surface along the
// robot→obstacle line: worst-case centre error over direction (max of the two
// axis variances) plus an independent radius error, added in quadrature.
inline double SurfaceSigma(const obstacle_detector::msg::CircleObstacle& c) {
  const double var_pos = std::max(c.covariance[0], c.covariance[1]);
  const double var_r = c.covariance[2];
  const double var = var_pos + var_r;
  return (var > 0.0) ? std::sqrt(var) : 0.0;
}

inline obstacle_detector::msg::Obstacles Apply(
    const obstacle_detector::msg::Obstacles& in, const Params& p,
    double now_s, Stats* stats = nullptr) {
  obstacle_detector::msg::Obstacles out;
  out.header = in.header;
  const double stamp_s = static_cast<double>(in.header.stamp.sec) +
                         1e-9 * static_cast<double>(in.header.stamp.nanosec);
  if (now_s - stamp_s > p.max_age) {
    if (stats) ++stats->stale_messages;
    return out;  // stale: gate every circle (stamp passes through)
  }
  out.circles.reserve(in.circles.size());
  for (const auto& c : in.circles) {
    if (c.true_radius > p.max_circle_radius) {
      if (stats) {
        ++stats->dropped_large_radius;
        stats->radius_max_dropped =
            std::max(stats->radius_max_dropped, c.true_radius);
      }
      continue;
    }
    auto safe = c;
    const double r = std::max(c.true_radius, p.min_radius);
    double vx = c.velocity.x;
    double vy = c.velocity.y;
    const double speed = std::hypot(vx, vy);
    if (speed > p.v_max_obstacle) {
      const double k = p.v_max_obstacle / speed;
      vx *= k;
      vy *= k;
    }
    safe.velocity.x = vx;
    safe.velocity.y = vy;
    double sigma_term = 0.0;
    if (p.use_covariance) {
      const double sigma = SurfaceSigma(c);
      if (stats) {
        stats->sigma_max_seen = std::max(stats->sigma_max_seen, sigma);
        if (sigma > kSigmaPlausibleMax) ++stats->sigma_implausible;
        if (sigma > p.sigma_max) ++stats->sigma_clamped;
      }
      sigma_term = p.k_sigma * std::min(sigma, p.sigma_max);
    }
    safe.radius = 0.75*(r + p.fixed_inflation + sigma_term +
                  std::hypot(vx, vy) * p.latency_horizon);
    out.circles.push_back(safe);
  }
  return out;
}

}  // namespace safety_obstacle_filter
