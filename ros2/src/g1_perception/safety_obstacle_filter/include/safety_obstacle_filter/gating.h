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
//   3. Inflate: radius = r + fixed_inflation + |v_clamped|·latency_horizon.
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
};

inline obstacle_detector::msg::Obstacles Apply(
    const obstacle_detector::msg::Obstacles& in, const Params& p,
    double now_s) {
  obstacle_detector::msg::Obstacles out;
  out.header = in.header;
  const double stamp_s = static_cast<double>(in.header.stamp.sec) +
                         1e-9 * static_cast<double>(in.header.stamp.nanosec);
  if (now_s - stamp_s > p.max_age) {
    return out;  // stale: gate every circle (stamp passes through)
  }
  out.circles.reserve(in.circles.size());
  for (const auto& c : in.circles) {
    if (c.true_radius > p.max_circle_radius) continue;
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
    safe.radius = r + p.fixed_inflation +
                  std::hypot(vx, vy) * p.latency_horizon;
    out.circles.push_back(safe);
  }
  return out;
}

}  // namespace safety_obstacle_filter
