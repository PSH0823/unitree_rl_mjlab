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
    // k_sigma (Appendix A) is accepted but inert until P-3 covariance
    // export lands (§20 Q-2).
    declare_parameter("k_sigma", 2.748);

    pub_ = create_publisher<obstacle_detector::msg::Obstacles>(
        "obstacles_safe", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
        "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](const obstacle_detector::msg::Obstacles& msg) {
          pub_->publish(Apply(msg, params_, now().seconds()));
        });
    RCLCPP_INFO(get_logger(),
                "safety_obstacle_filter: max_age=%.2f min_r=%.2f max_r=%.2f "
                "fixed=%.3f horizon=%.2f v_max=%.2f",
                params_.max_age, params_.min_radius,
                params_.max_circle_radius, params_.fixed_inflation,
                params_.latency_horizon, params_.v_max_obstacle);
  }

 private:
  Params params_;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr pub_;
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_;
};

}  // namespace safety_obstacle_filter

RCLCPP_COMPONENTS_REGISTER_NODE(
    safety_obstacle_filter::SafetyObstacleFilterNode)
