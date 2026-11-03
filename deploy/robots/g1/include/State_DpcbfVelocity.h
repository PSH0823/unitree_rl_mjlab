#pragma once

#include "FSM/FSMState.h"
#include "dpcbf/constant_velocity_kalman.h"
#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/viz_publisher.h"
#include "isaaclab/envs/manager_based_rl_env.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

// Joystick velocity command -> DPCBF/OSQP -> locomotion ONNX.  The QP and
// ONNX inference deliberately share one 50 Hz worker, so every joint target
// is generated from the safe command computed in the same control tick.
class State_DpcbfVelocity : public FSMState {
 public:
  State_DpcbfVelocity(int state_mode, std::string state_string);
  ~State_DpcbfVelocity();

  void enter() override;
  void run() override;
  void exit() override;

 private:
  using Clock = std::chrono::steady_clock;

  struct RobotSnapshot {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx_world = 0.0;
    double vy_world = 0.0;
    Clock::time_point received{};
    bool valid = false;
  };

  void OnOdometry(const nav_msgs::msg::Odometry& message);
  void OnObstacles(const obstacle_detector::msg::Obstacles& message);
  void ControlLoop();
  dpcbf::VelocityCommand ReadJoystickCommand() const;
  dpcbf::VelocityCommand ApproachZero(
      const dpcbf::VelocityCommand& command) const;
  void PublishVelocityCommands(const dpcbf::VelocityCommand& nominal,
                               const dpcbf::VelocityCommand& safe);
  void PublishRobotVelocities(
      const builtin_interfaces::msg::Time& stamp, double yaw,
      double raw_vx_world, double raw_vy_world,
      double filtered_vx_world, double filtered_vy_world,
      double yaw_rate);
  void PublishPlot(const RobotSnapshot& robot,
                   const dpcbf::VelocityCommand& nominal,
                   const dpcbf::VelocityCommand& scaled,
                   const dpcbf::VelocityCommand& safe,
                   const dpcbf::SafetyFilterResult& result,
                   double command_scale, double obstacle_age,
                   std::uint8_t staleness_state,
                   std::size_t obstacle_total);

  std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
  dpcbf::DpcbfSafetyFilter filter_;
  dpcbf::ConstantVelocityKalman2D velocity_kalman_;
  std::unique_ptr<dpcbf_ros_adapter::DpcbfVizPublisher> viz_publisher_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      nominal_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      safe_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      raw_robot_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      filtered_robot_velocity_pub_;
  std::thread ros_thread_;
  std::atomic<bool> ros_running_{false};

  std::thread control_thread_;
  std::atomic<bool> control_running_{false};
  std::atomic<bool> low_level_failed_{false};
  std::atomic<float> joystick_lx_{0.0f};
  std::atomic<float> joystick_ly_{0.0f};
  std::atomic<float> joystick_rx_{0.0f};

  mutable std::mutex sensor_mutex_;
  RobotSnapshot robot_;
  std::vector<dpcbf::ObstacleState> obstacles_;
  Clock::time_point obstacles_received_{};
  bool obstacles_received_once_ = false;
  std::int64_t previous_odom_stamp_ns_ = 0;
  double previous_odom_x_ = 0.0;
  double previous_odom_y_ = 0.0;
  double previous_odom_yaw_ = 0.0;
  bool odom_measurement_initialized_ = false;
  int consecutive_kalman_rejections_ = 0;

  std::mutex target_mutex_;
  std::vector<float> joint_targets_;
  bool targets_valid_ = false;

  std::string expected_frame_ = "odom";
  std::string command_frame_ = "base_link";
  double odometry_timeout_ = 0.20;
  double max_odometry_speed_ = 3.0;
  bool enable_kalman_ = true;
  double kalman_position_measurement_std_ = 0.01;
  double kalman_process_acceleration_std_ = 48.0;
  double kalman_initial_velocity_std_ = 0.5;
  double kalman_innovation_gate_sigma_ = 5.0;
  int kalman_max_consecutive_rejections_ = 3;
  double odometry_min_dt_ = 0.001;
  double odometry_max_dt_ = 0.10;
  double joystick_deadzone_ = 0.08;
  double obstacle_max_age_ = 0.30;
  double obstacle_fade_out_ = 0.30;
  double obstacle_hold_after_stale_ = 1.0;
  int qp_failure_hold_cycles_ = 1;
  double fail_safe_linear_deceleration_ = 2.0;
  double fail_safe_yaw_deceleration_ = 2.0;
  double velocity_arrow_seconds_ = 1.0;
  double world_parabola_lateral_limit_ = 1.0;
  double world_parabola_backward_limit_ = 1.5;

  dpcbf::VelocityCommand last_safe_command_{};
  int consecutive_qp_failures_ = 0;
  std::uint64_t tick_ = 0;
  Clock::time_point control_epoch_{};
};

REGISTER_FSM(State_DpcbfVelocity)
