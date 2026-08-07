// Composable node wrapper for the §9.6 rules: /tracked_obstacles (Reliable
// depth 5, matching the tracker publisher) → /obstacles_safe (Reliable
// depth 1 — latest wins, §7.1). With use_sim_time=true (the sim launch
// default) now() is sim time, matching the sim-time header stamps.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "safety_obstacle_filter/gating.h"

namespace safety_obstacle_filter {

class SafetyObstacleFilterNode : public rclcpp::Node {
 public:
  explicit SafetyObstacleFilterNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("safety_obstacle_filter", options) {
    params_.max_age = declare_parameter("max_age", params_.max_age);
    params_.min_radius = declare_parameter("min_radius", params_.min_radius);
    params_.max_circle_radius =
        declare_parameter("max_circle_radius", params_.max_circle_radius);
    params_.fixed_inflation =
        declare_parameter("fixed_inflation", params_.fixed_inflation);
    params_.latency_horizon =
        declare_parameter("latency_horizon", params_.latency_horizon);
    params_.v_max_obstacle =
        declare_parameter("v_max_obstacle", params_.v_max_obstacle);
    // P-3 sigma term. The covariance is now published by the tracker (fork
    // patch 0007), but the term stays OFF by default: k_sigma's only value is
    // the abandoned branch's 2.748, a placeholder until 5B's residual capture.
    params_.use_covariance =
        declare_parameter("use_covariance", params_.use_covariance);
    params_.k_sigma = declare_parameter("k_sigma", params_.k_sigma);
    params_.sigma_max = declare_parameter("sigma_max", params_.sigma_max);

    pub_ = create_publisher<obstacle_detector::msg::Obstacles>(
        "obstacles_safe", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
        "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        // ConstSharedPtr, not `const Msg&`: rclcpp only accepts const-reference
        // subscription callbacks from Galactic on, and the G1's onboard
        // computer runs Foxy. Apply() still takes the message by const
        // reference — nothing about the filtering changes.
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr msg) {
          pub_->publish(Apply(*msg, params_, now().seconds(), &stats_));
          ReportStats();
        });
    RCLCPP_INFO(get_logger(),
                "safety_obstacle_filter: max_age=%.2f min_r=%.2f max_r=%.2f "
                "fixed=%.3f horizon=%.2f v_max=%.2f sigma=%s(k=%.3f cap=%.2f)",
                params_.max_age, params_.min_radius,
                params_.max_circle_radius, params_.fixed_inflation,
                params_.latency_horizon, params_.v_max_obstacle,
                params_.use_covariance ? "ON" : "off", params_.k_sigma,
                params_.sigma_max);
  }

 private:
  // Every path on which this node silently changes or discards an obstacle
  // gets a counter and a throttled warning. Gap G2's whole lesson is that a
  // drop nobody can observe is a defect wherever the threshold sits, and the
  // sigma cap is the same shape of thing: a diverged track quietly pinned at
  // sigma_max looks identical to a confident one.
  void ReportStats() {
    if (stats_.dropped_large_radius != last_.dropped_large_radius) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropped %ld circle(s) with true_radius > max_circle_radius %.2f m "
          "(largest %.3f m) — INVISIBLE to DPCBF",
          stats_.dropped_large_radius, params_.max_circle_radius,
          stats_.radius_max_dropped);
    }
    if (stats_.sigma_implausible != last_.sigma_implausible) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "sigma %.3f m exceeds the plausibility bound %.2f m on %ld circle(s): "
          "the tracker's measurement_variance (m^2) is not set from data, so "
          "k_sigma is a scale factor for the wrong quantity",
          stats_.sigma_max_seen, kSigmaPlausibleMax, stats_.sigma_implausible);
    } else if (stats_.sigma_clamped != last_.sigma_clamped) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "sigma clamped at sigma_max %.2f m on %ld circle(s) "
                           "(max seen %.3f m)",
                           params_.sigma_max, stats_.sigma_clamped,
                           stats_.sigma_max_seen);
    }
    last_ = stats_;
  }

  Params params_;
  Stats stats_;
  Stats last_;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr pub_;
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_;
};

}  // namespace safety_obstacle_filter

RCLCPP_COMPONENTS_REGISTER_NODE(
    safety_obstacle_filter::SafetyObstacleFilterNode)
