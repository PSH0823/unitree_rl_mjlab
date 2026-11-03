#include "State_DpcbfVelocity.h"

#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "unitree_articulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace {

double AgeSeconds(const std::chrono::steady_clock::time_point& stamp) {
  if (stamp.time_since_epoch().count() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp)
      .count();
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double WrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::int64_t StampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

void ClipVectorMagnitude(double limit, double* x, double* y) {
  const double magnitude = std::hypot(*x, *y);
  if (magnitude > limit) {
    const double scale = limit / magnitude;
    *x *= scale;
    *y *= scale;
  }
}

double Approach(double value, double target, double delta) {
  if (value < target) return std::min(value + delta, target);
  return std::max(value - delta, target);
}

bool AllFinite(const std::vector<float>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

std::filesystem::path ResolveFromProject(const YAML::Node& node) {
  std::filesystem::path path = node.as<std::string>();
  if (path.is_relative()) path = param::proj_dir / path;
  return std::filesystem::weakly_canonical(path);
}

}  // namespace

State_DpcbfVelocity::State_DpcbfVelocity(int state_mode,
                                         std::string state_string)
    : FSMState(state_mode, std::move(state_string)) {
  const YAML::Node state_cfg = param::config["FSM"][getStateString()];
  const auto policy_dir =
      param::parser_policy_dir(state_cfg["policy_dir"].as<std::string>());
  const auto dpcbf_config_path = ResolveFromProject(state_cfg["dpcbf_config"]);

  env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
      YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string()),
      std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
          FSMState::lowstate));
  env_->alg = std::make_unique<isaaclab::OrtRunner>(
      policy_dir / "exported" / "policy.onnx");

  filter_.LoadConfig(dpcbf_config_path);
  filter_.Initialize(env_->step_dt);

  const YAML::Node root = YAML::LoadFile(dpcbf_config_path.string());
  const YAML::Node deployment = root["deployment"];
  if (!deployment) {
    throw std::runtime_error("missing 'deployment' in " +
                             dpcbf_config_path.string());
  }
  expected_frame_ = deployment["expected_frame"].as<std::string>("odom");
  command_frame_ =
      deployment["command_frame"].as<std::string>("base_link");
  odometry_timeout_ = deployment["odometry_timeout"].as<double>(0.20);
  max_odometry_speed_ = deployment["max_odometry_speed"].as<double>(3.0);
  const YAML::Node velocity_estimator = deployment["velocity_estimator"];
  if (velocity_estimator) {
    enable_kalman_ =
        velocity_estimator["enable_kalman"].as<bool>(true);
    kalman_position_measurement_std_ =
        velocity_estimator["position_measurement_std"].as<double>(0.01);
    kalman_process_acceleration_std_ =
        velocity_estimator["process_acceleration_std"].as<double>(48.0);
    kalman_initial_velocity_std_ =
        velocity_estimator["initial_velocity_std"].as<double>(0.5);
    kalman_innovation_gate_sigma_ =
        velocity_estimator["innovation_gate_sigma"].as<double>(5.0);
    kalman_max_consecutive_rejections_ =
        velocity_estimator["max_consecutive_rejections"].as<int>(3);
    odometry_min_dt_ = velocity_estimator["min_dt"].as<double>(0.001);
    odometry_max_dt_ = velocity_estimator["max_dt"].as<double>(0.10);
  }
  joystick_deadzone_ = deployment["joystick_deadzone"].as<double>(0.08);
  obstacle_max_age_ = deployment["obstacle_max_age"].as<double>(0.30);
  obstacle_fade_out_ = deployment["obstacle_fade_out"].as<double>(0.30);
  obstacle_hold_after_stale_ =
      deployment["obstacle_hold_after_stale"].as<double>(1.0);
  qp_failure_hold_cycles_ =
      deployment["qp_failure_hold_cycles"].as<int>(1);
  fail_safe_linear_deceleration_ =
      deployment["fail_safe_linear_deceleration"].as<double>(2.0);
  fail_safe_yaw_deceleration_ =
      deployment["fail_safe_yaw_deceleration"].as<double>(2.0);
  if (odometry_timeout_ <= 0.0 || max_odometry_speed_ <= 0.0 ||
      kalman_position_measurement_std_ <= 0.0 ||
      kalman_process_acceleration_std_ <= 0.0 ||
      kalman_initial_velocity_std_ <= 0.0 ||
      kalman_innovation_gate_sigma_ <= 0.0 ||
      kalman_max_consecutive_rejections_ <= 0 ||
      odometry_min_dt_ <= 0.0 || odometry_max_dt_ <= odometry_min_dt_ ||
      joystick_deadzone_ < 0.0 || joystick_deadzone_ >= 1.0 ||
      obstacle_max_age_ <= 0.0 || obstacle_fade_out_ < 0.0 ||
      obstacle_hold_after_stale_ < 0.0 || qp_failure_hold_cycles_ < 0 ||
      fail_safe_linear_deceleration_ <= 0.0 ||
      fail_safe_yaw_deceleration_ <= 0.0) {
    throw std::runtime_error("invalid DpcbfVelocity deployment parameters");
  }
  velocity_kalman_.Configure(
      {kalman_position_measurement_std_,
       kalman_process_acceleration_std_,
       kalman_initial_velocity_std_,
       kalman_innovation_gate_sigma_});

  node_ = rclcpp::Node::make_shared("g1_dpcbf_velocity");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      deployment["odometry_topic"].as<std::string>("/odom"), qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        OnOdometry(*message);
      });
  obstacle_sub_ = node_->create_subscription<obstacle_detector::msg::Obstacles>(
      deployment["obstacle_topic"].as<std::string>("/obstacles_safe"), qos,
      [this](obstacle_detector::msg::Obstacles::ConstSharedPtr message) {
        OnObstacles(*message);
      });
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  nominal_command_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          deployment["nominal_command_topic"].as<std::string>(
              "/dpcbf/nominal_velocity_command"),
          command_qos);
  safe_command_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          deployment["safe_command_topic"].as<std::string>(
              "/dpcbf/safe_velocity_command"),
          command_qos);
  raw_robot_velocity_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          velocity_estimator && velocity_estimator["raw_velocity_topic"]
              ? velocity_estimator["raw_velocity_topic"].as<std::string>()
              : "/dpcbf/raw_robot_velocity",
          command_qos);
  filtered_robot_velocity_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          velocity_estimator && velocity_estimator["filtered_velocity_topic"]
              ? velocity_estimator["filtered_velocity_topic"].as<std::string>()
              : "/dpcbf/filtered_robot_velocity",
          command_qos);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  ros_running_.store(true);
  ros_thread_ = std::thread([this] {
    while (ros_running_.load() && rclcpp::ok()) {
      executor_->spin_some(std::chrono::milliseconds(20));
    }
  });

  const YAML::Node visualization = root["visualization"];
  if (visualization) {
    velocity_arrow_seconds_ =
        visualization["velocity_arrow_seconds"].as<double>(1.0);
    world_parabola_lateral_limit_ =
        visualization["world_parabola_lateral_limit"].as<double>(1.0);
    world_parabola_backward_limit_ =
        visualization["world_parabola_backward_limit"].as<double>(1.5);
  }
  if (visualization && visualization["enabled"].as<bool>(true)) {
    dpcbf_ros_adapter::DpcbfVizPublisher::Config config;
    config.topic = deployment["plot_topic"].as<std::string>("/dpcbf/plot");
    config.rate_hz = visualization["refresh_hz"].as<double>(30.0);
    config.frame_id = expected_frame_;
    viz_publisher_ =
        std::make_unique<dpcbf_ros_adapter::DpcbfVizPublisher>(config);
  }

  registered_checks.emplace_back(std::make_pair(
      [this] { return low_level_failed_.load(); },
      FSMStringMap.right.at("Passive")));
  registered_checks.emplace_back(std::make_pair(
      [this] { return isaaclab::mdp::bad_orientation(env_.get(), 1.0f); },
      FSMStringMap.right.at("Passive")));
}

State_DpcbfVelocity::~State_DpcbfVelocity() {
  exit();
  ros_running_.store(false);
  if (executor_) executor_->cancel();
  if (ros_thread_.joinable()) ros_thread_.join();
}

void State_DpcbfVelocity::enter() {
  for (int i = 0; i < env_->robot->data.joint_stiffness.size(); ++i) {
    lowcmd->msg_.motor_cmd()[i].kp() = env_->robot->data.joint_stiffness[i];
    lowcmd->msg_.motor_cmd()[i].kd() = env_->robot->data.joint_damping[i];
    lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
    lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
  }
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    joint_targets_.clear();
    targets_valid_ = false;
  }
  last_safe_command_ = {};
  consecutive_qp_failures_ = 0;
  tick_ = 0;
  low_level_failed_.store(false);
  joystick_lx_.store(0.0f);
  joystick_ly_.store(0.0f);
  joystick_rx_.store(0.0f);
  control_epoch_ = Clock::now();
  env_->set_external_velocity_command({0.0f, 0.0f, 0.0f});
  env_->reset();
  control_running_.store(true);
  control_thread_ = std::thread(&State_DpcbfVelocity::ControlLoop, this);
  spdlog::info("DpcbfVelocity: one 50 Hz QP -> ONNX worker started");
}

void State_DpcbfVelocity::run() {
  // The 1 kHz FSM thread owns joystick polling. Hand a coherent-enough
  // lock-free snapshot to the 50 Hz QP worker instead of reading the device
  // object concurrently from two threads.
  auto& joystick = env_->robot->data.joystick;
  joystick_lx_.store(joystick->lx(), std::memory_order_relaxed);
  joystick_ly_.store(joystick->ly(), std::memory_order_relaxed);
  joystick_rx_.store(joystick->rx(), std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (!targets_valid_) return;
  for (int i = 0; i < env_->robot->data.joint_ids_map.size(); ++i) {
    lowcmd->msg_.motor_cmd()[env_->robot->data.joint_ids_map[i]].q() =
        joint_targets_[i];
  }
}

void State_DpcbfVelocity::exit() {
  control_running_.store(false);
  if (control_thread_.joinable()) control_thread_.join();
  env_->set_external_velocity_command({0.0f, 0.0f, 0.0f});
  last_safe_command_ = {};
  std::lock_guard<std::mutex> lock(target_mutex_);
  targets_valid_ = false;
}

void State_DpcbfVelocity::OnOdometry(
    const nav_msgs::msg::Odometry& message) {
  if (!message.header.frame_id.empty() &&
      message.header.frame_id != expected_frame_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring odometry frame '%s'; expected '%s'",
                         message.header.frame_id.c_str(),
                         expected_frame_.c_str());
    return;
  }
  const auto received_now = Clock::now();
  const double x = message.pose.pose.position.x;
  const double y = message.pose.pose.position.y;
  const double yaw = QuaternionYaw(message.pose.pose.orientation);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return;

  builtin_interfaces::msg::Time publish_stamp = message.header.stamp;
  std::int64_t measurement_stamp_ns = StampNanoseconds(message.header.stamp);
  if (measurement_stamp_ns <= 0) {
    measurement_stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               received_now.time_since_epoch())
                               .count();
    publish_stamp = node_->get_clock()->now();
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Odometry header stamp is zero; falling back to callback time");
  }

  double raw_vx_world = 0.0;
  double raw_vy_world = 0.0;
  double filtered_vx_world = 0.0;
  double filtered_vy_world = 0.0;
  double yaw_rate = 0.0;
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // Freshness of the control input is intentionally based on callback
    // receipt time (robot_.received), whereas velocity differentiation is
    // based exclusively on the sensor's header timestamp.  Do not reset the
    // estimator from callback timing here: delayed DDS delivery can still
    // contain correctly spaced measurements and control-side staleness is
    // already handled independently in ControlLoop().
    bool reset_estimator = !odom_measurement_initialized_;
    double dt = 0.0;
    if (odom_measurement_initialized_) {
      dt = static_cast<double>(measurement_stamp_ns -
                               previous_odom_stamp_ns_) *
           1.0e-9;
      // A large positive gap is a sensor interruption; a negative gap is a
      // simulation/estimator epoch reset. Both require a clean state reset.
      // Tiny non-positive deltas remain duplicate/out-of-order samples and
      // are ignored below without advancing the time base.
      if (dt > odometry_max_dt_ || dt < -odometry_min_dt_) {
        reset_estimator = true;
      }
    }

    if (reset_estimator) {
      velocity_kalman_.Reset(x, y);
      consecutive_kalman_rejections_ = 0;
      previous_odom_stamp_ns_ = measurement_stamp_ns;
      previous_odom_x_ = x;
      previous_odom_y_ = y;
      previous_odom_yaw_ = yaw;
      odom_measurement_initialized_ = true;
    } else if (dt >= odometry_min_dt_) {
      raw_vx_world = (x - previous_odom_x_) / dt;
      raw_vy_world = (y - previous_odom_y_) / dt;
      yaw_rate = WrapAngle(yaw - previous_odom_yaw_) / dt;
      const double raw_speed = std::hypot(raw_vx_world, raw_vy_world);
      ClipVectorMagnitude(max_odometry_speed_, &raw_vx_world,
                          &raw_vy_world);
      if (raw_speed > max_odometry_speed_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Clipping raw odometry speed %.3f to %.3f m/s", raw_speed,
            max_odometry_speed_);
      }

      if (enable_kalman_) {
        if (velocity_kalman_.Update(x, y, dt)) {
          consecutive_kalman_rejections_ = 0;
        } else {
          ++consecutive_kalman_rejections_;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 2000,
              "Kalman innovation rejected (%d/%d consecutive)",
              consecutive_kalman_rejections_,
              kalman_max_consecutive_rejections_);
          if (consecutive_kalman_rejections_ >=
              kalman_max_consecutive_rejections_) {
            velocity_kalman_.Reset(x, y);
            consecutive_kalman_rejections_ = 0;
          }
        }
        filtered_vx_world = velocity_kalman_.vx();
        filtered_vy_world = velocity_kalman_.vy();
      } else {
        filtered_vx_world = raw_vx_world;
        filtered_vy_world = raw_vy_world;
      }
      ClipVectorMagnitude(max_odometry_speed_, &filtered_vx_world,
                          &filtered_vy_world);

      previous_odom_stamp_ns_ = measurement_stamp_ns;
      previous_odom_x_ = x;
      previous_odom_y_ = y;
      previous_odom_yaw_ = yaw;
    } else {
      // A duplicate/out-of-order stamp must not advance the differencing or
      // Kalman time base. Keep the last velocity but refresh receive age.
      filtered_vx_world = robot_.vx_world;
      filtered_vy_world = robot_.vy_world;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Ignoring odometry velocity update with dt=%.6f s", dt);
    }

    robot_ = {x, y, yaw, filtered_vx_world, filtered_vy_world,
              received_now, true};
  }

  PublishRobotVelocities(publish_stamp, yaw, raw_vx_world, raw_vy_world,
                         filtered_vx_world, filtered_vy_world, yaw_rate);
}

void State_DpcbfVelocity::OnObstacles(
    const obstacle_detector::msg::Obstacles& message) {
  if (!message.header.frame_id.empty() &&
      message.header.frame_id != expected_frame_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring obstacle frame '%s'; expected '%s'",
                         message.header.frame_id.c_str(),
                         expected_frame_.c_str());
    return;
  }
  std::vector<dpcbf::ObstacleState> next;
  next.reserve(message.circles.size());
  for (const auto& circle : message.circles) {
    if (!std::isfinite(circle.center.x) ||
        !std::isfinite(circle.center.y) || !std::isfinite(circle.radius) ||
        !std::isfinite(circle.velocity.x) ||
        !std::isfinite(circle.velocity.y) || circle.radius <= 0.0) {
      continue;
    }
    next.push_back({circle.center.x, circle.center.y, circle.radius,
                    circle.velocity.x, circle.velocity.y,
                    static_cast<int>(circle.uid & 0x7fffffffu)});
  }
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  obstacles_ = std::move(next);
  obstacles_received_ = Clock::now();
  obstacles_received_once_ = true;
}

dpcbf::VelocityCommand State_DpcbfVelocity::ReadJoystickCommand() const {
  const auto axis_to_command = [this](double axis, double lower,
                                      double upper) {
    axis = std::clamp(axis, -1.0, 1.0);
    const double magnitude = std::abs(axis);
    if (magnitude <= joystick_deadzone_) return 0.0;
    axis = std::copysign(
        (magnitude - joystick_deadzone_) / (1.0 - joystick_deadzone_), axis);
    return axis >= 0.0 ? axis * upper : -axis * lower;
  };
  dpcbf::VelocityCommand command;
  command.sagittal = axis_to_command(
      joystick_ly_.load(std::memory_order_relaxed),
      filter_.sagittal_velocity_min(),
      filter_.sagittal_velocity_max());
  command.lateral = axis_to_command(
      -joystick_lx_.load(std::memory_order_relaxed),
      filter_.lateral_velocity_min(),
      filter_.lateral_velocity_max());
  command.yaw_rate = axis_to_command(
      -joystick_rx_.load(std::memory_order_relaxed),
      filter_.yaw_rate_min(), filter_.yaw_rate_max());
  return command;
}

dpcbf::VelocityCommand State_DpcbfVelocity::ApproachZero(
    const dpcbf::VelocityCommand& command) const {
  const double dt = env_->step_dt;
  return {Approach(command.sagittal, 0.0,
                   fail_safe_linear_deceleration_ * dt),
          Approach(command.lateral, 0.0,
                   fail_safe_linear_deceleration_ * dt),
          Approach(command.yaw_rate, 0.0,
                   fail_safe_yaw_deceleration_ * dt)};
}

void State_DpcbfVelocity::PublishVelocityCommands(
    const dpcbf::VelocityCommand& nominal,
    const dpcbf::VelocityCommand& safe) {
  const auto stamp = node_->get_clock()->now();
  const auto make_message = [this, &stamp](
                                const dpcbf::VelocityCommand& command) {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = command_frame_;
    message.twist.linear.x = command.sagittal;
    message.twist.linear.y = command.lateral;
    message.twist.angular.z = command.yaw_rate;
    return message;
  };
  nominal_command_pub_->publish(make_message(nominal));
  safe_command_pub_->publish(make_message(safe));
}

void State_DpcbfVelocity::PublishRobotVelocities(
    const builtin_interfaces::msg::Time& stamp, double yaw,
    double raw_vx_world, double raw_vy_world,
    double filtered_vx_world, double filtered_vy_world,
    double yaw_rate) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const auto make_message = [&](double vx_world, double vy_world) {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = command_frame_;
    message.twist.linear.x = cosine * vx_world + sine * vy_world;
    message.twist.linear.y = -sine * vx_world + cosine * vy_world;
    message.twist.angular.z = yaw_rate;
    return message;
  };
  raw_robot_velocity_pub_->publish(
      make_message(raw_vx_world, raw_vy_world));
  filtered_robot_velocity_pub_->publish(
      make_message(filtered_vx_world, filtered_vy_world));
}

void State_DpcbfVelocity::ControlLoop() {
  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(env_->step_dt));
  auto next = Clock::now();
  int invalid_outputs = 0;
  while (control_running_.load()) {
    const auto started = Clock::now();
    RobotSnapshot robot;
    std::vector<dpcbf::ObstacleState> obstacles;
    Clock::time_point obstacle_stamp;
    bool received_obstacles = false;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      robot = robot_;
      obstacles = obstacles_;
      obstacle_stamp = obstacles_received_;
      received_obstacles = obstacles_received_once_;
    }

    const dpcbf::VelocityCommand nominal = ReadJoystickCommand();
    const double obstacle_age =
        received_obstacles ? AgeSeconds(obstacle_stamp)
                           : std::numeric_limits<double>::infinity();
    double command_scale = 0.0;
    std::uint8_t staleness = 3;
    if (received_obstacles) {
      if (obstacle_age <= obstacle_max_age_) {
        command_scale = 1.0;
        staleness = 0;
      } else if (obstacle_fade_out_ > 0.0 &&
                 obstacle_age <= obstacle_max_age_ + obstacle_fade_out_) {
        command_scale = 1.0 -
            (obstacle_age - obstacle_max_age_) / obstacle_fade_out_;
        staleness = 1;
      } else {
        staleness = 2;
      }
      const double extrapolation =
          std::min(obstacle_age, obstacle_max_age_ + obstacle_fade_out_);
      const double inflate_horizon =
          staleness == 2
              ? std::min(obstacle_age, obstacle_hold_after_stale_)
              : 0.0;
      for (auto& obstacle : obstacles) {
        obstacle.x += obstacle.velocity_x * extrapolation;
        obstacle.y += obstacle.velocity_y * extrapolation;
        obstacle.radius += std::hypot(obstacle.velocity_x,
                                      obstacle.velocity_y) * inflate_horizon;
      }
      if (staleness == 2) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "DpcbfVelocity command gated to zero: /obstacles_safe stale "
            "(age=%.3f s, fresh limit=%.3f s, fade=%.3f s)",
            obstacle_age, obstacle_max_age_, obstacle_fade_out_);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "DpcbfVelocity command gated to zero: waiting for the first "
          "/obstacles_safe message (an empty message is valid)");
    }
    const dpcbf::VelocityCommand scaled{
        nominal.sagittal * command_scale,
        nominal.lateral * command_scale,
        nominal.yaw_rate * command_scale};

    dpcbf::SafetyFilterResult result;
    dpcbf::VelocityCommand safe = last_safe_command_;
    const bool odometry_fresh =
        robot.valid && AgeSeconds(robot.received) <= odometry_timeout_;
    const bool command_gate_open = command_scale > 1.0e-6;
    if (odometry_fresh && command_gate_open) {
      const double c = std::cos(robot.yaw);
      const double s = std::sin(robot.yaw);
      dpcbf::RobotState state;
      state.x = robot.x;
      state.y = robot.y;
      state.phi = robot.yaw;
      state.sagittal_velocity = c * robot.vx_world + s * robot.vy_world;
      state.lateral_velocity = -s * robot.vx_world + c * robot.vy_world;
      result = filter_.Filter(state, scaled, obstacles, last_safe_command_);
    }

    if (odometry_fresh && command_gate_open && result.solved) {
      safe = result.command;
      last_safe_command_ = safe;
      consecutive_qp_failures_ = 0;
    } else if (odometry_fresh && !command_gate_open) {
      // A missing/stale obstacle stream is a command gate, not a QP request.
      // Calling the QP with a zero desired command would still integrate its
      // delta-v output into the previous command and could wind it up to a
      // velocity limit while the gate is closed.
      safe = ApproachZero(last_safe_command_);
      last_safe_command_ = safe;
      consecutive_qp_failures_ = 0;
    } else {
      ++consecutive_qp_failures_;
      if (consecutive_qp_failures_ > qp_failure_hold_cycles_) {
        safe = ApproachZero(last_safe_command_);
        last_safe_command_ = safe;
      }
      if (!odometry_fresh) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "DpcbfVelocity: /odom missing or stale; "
                             "skipping QP and decelerating command");
      } else if (command_gate_open) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "DpcbfVelocity: OSQP failed; holding once then "
                              "decelerating command");
      }
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "DpcbfVelocity status: raw_axes=(%.2f %.2f %.2f), "
        "nominal=(%.2f %.2f %.2f), scale=%.2f, safe=(%.2f %.2f %.2f), "
        "odom=%s age=%.3f, obstacles=%s age=%.3f count=%zu, qp=%s",
        joystick_lx_.load(std::memory_order_relaxed),
        joystick_ly_.load(std::memory_order_relaxed),
        joystick_rx_.load(std::memory_order_relaxed), nominal.sagittal,
        nominal.lateral, nominal.yaw_rate, command_scale, safe.sagittal,
        safe.lateral, safe.yaw_rate, odometry_fresh ? "fresh" : "STALE",
        AgeSeconds(robot.received), received_obstacles ? "seen" : "MISSING",
        obstacle_age, obstacles.size(),
        !command_gate_open ? "gated"
                           : (result.solved ? "solved" : "FAILED"));

    // Full-rate (50 Hz) command seam for rosbag analysis. Both messages use
    // the same timestamp and body frame: nominal is the joystick intent;
    // safe is the command actually passed to the locomotion policy.
    PublishVelocityCommands(nominal, safe);

    env_->set_external_velocity_command(
        {static_cast<float>(safe.sagittal), static_cast<float>(safe.lateral),
         static_cast<float>(safe.yaw_rate)});
    try {
      env_->step();
      const auto targets = env_->action_manager->processed_actions();
      if (targets.size() != env_->robot->data.joint_ids_map.size() ||
          !AllFinite(targets)) {
        throw std::runtime_error("invalid low-level policy output");
      }
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        joint_targets_ = targets;
        targets_valid_ = true;
      }
      invalid_outputs = 0;
    } catch (const std::exception& error) {
      ++invalid_outputs;
      spdlog::error("DpcbfVelocity low-level inference failed: {}",
                    error.what());
      if (invalid_outputs >= 3) low_level_failed_.store(true);
    }

    PublishPlot(robot, nominal, scaled, safe, result, command_scale,
                received_obstacles ? obstacle_age : -1.0, staleness,
                obstacles.size());
    ++tick_;

    next += period;
    const auto finished = Clock::now();
    if (finished > next) {
      const double overrun =
          std::chrono::duration<double>(finished - next).count();
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "DpcbfVelocity 50 Hz worker overran by %.3f ms",
                           overrun * 1000.0);
      next = std::max(finished, started + period);
    }
    std::this_thread::sleep_until(next);
  }
}

void State_DpcbfVelocity::PublishPlot(
    const RobotSnapshot& robot, const dpcbf::VelocityCommand& nominal,
    const dpcbf::VelocityCommand& scaled,
    const dpcbf::VelocityCommand& safe,
    const dpcbf::SafetyFilterResult& result, double command_scale,
    double obstacle_age, std::uint8_t staleness_state,
    std::size_t obstacle_total) {
  if (!viz_publisher_) return;
  dpcbf_ros_adapter::PlotSample sample;
  sample.tick = tick_;
  sample.t_ctrl =
      std::chrono::duration<double>(Clock::now() - control_epoch_).count();
  sample.mode = 2;  // estimated /obstacles_safe
  sample.robot_x = robot.x;
  sample.robot_y = robot.y;
  sample.robot_phi = robot.yaw;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  sample.robot_sagittal_velocity = c * robot.vx_world + s * robot.vy_world;
  sample.robot_lateral_velocity = -s * robot.vx_world + c * robot.vy_world;
  sample.nominal = {nominal.sagittal, nominal.lateral, nominal.yaw_rate};
  sample.scaled = {scaled.sagittal, scaled.lateral, scaled.yaw_rate};
  sample.safe = {safe.sagittal, safe.lateral, safe.yaw_rate};
  sample.command_scale = command_scale;
  sample.solved = result.solved;
  sample.active_constraints = result.active_constraints;
  sample.active_dpcbf_constraints = result.active_dpcbf_constraints;
  sample.active_ecbf_constraints = result.active_ecbf_constraints;
  sample.acceleration = result.acceleration;
  sample.staleness_state = staleness_state;
  sample.obstacle_age_s = obstacle_age;
  sample.velocity_arrow_seconds = velocity_arrow_seconds_;
  sample.world_parabola_lateral_limit = world_parabola_lateral_limit_;
  sample.world_parabola_backward_limit = world_parabola_backward_limit_;
  sample.obstacle_total = static_cast<std::uint32_t>(obstacle_total);
  sample.obstacle_count = std::min(
      result.selected_obstacles.size(),
      dpcbf_ros_adapter::PlotSample::kMaxObstacles);
  for (std::size_t i = 0; i < sample.obstacle_count; ++i) {
    const auto& selected = result.selected_obstacles[i];
    auto& out = sample.obstacles[i];
    out.id = selected.obstacle.id;
    out.x = selected.obstacle.x;
    out.y = selected.obstacle.y;
    out.radius = selected.obstacle.radius;
    out.velocity_x = selected.obstacle.velocity_x;
    out.velocity_y = selected.obstacle.velocity_y;
    out.distance = selected.distance;
    out.h = selected.constraint.h;
    out.los_angle = std::atan2(selected.obstacle.y - robot.y,
                               selected.obstacle.x - robot.x);
    out.relative_velocity_x = selected.relative_velocity_world[0];
    out.relative_velocity_y = selected.relative_velocity_world[1];
    out.boundary_vertex_x = selected.boundary_vertex_x;
    out.boundary_curvature = selected.boundary_curvature;
  }
  viz_publisher_->Push(sample);
}
