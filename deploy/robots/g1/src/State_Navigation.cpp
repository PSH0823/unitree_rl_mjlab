#include "State_Navigation.h"

#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "unitree_articulation.h"
#include "dpcbf_ros_adapter/dpcbf_boundary_config.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double AgeSeconds(std::chrono::steady_clock::time_point stamp) {
    if (stamp.time_since_epoch().count() == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stamp).count();
}

bool AllFinite(const std::vector<float>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](float value) { return std::isfinite(value); });
}

std_msgs::msg::ColorRGBA Color(int rank, float alpha = 1.0f) {
    static constexpr float colors[][3] = {
        {1.00f, 0.55f, 0.10f}, {0.15f, 0.75f, 1.00f},
        {0.80f, 0.35f, 1.00f}, {0.20f, 0.90f, 0.45f},
        {1.00f, 0.30f, 0.45f}, {0.95f, 0.85f, 0.20f},
        {0.25f, 0.55f, 1.00f}, {0.70f, 0.85f, 0.25f},
        {1.00f, 0.45f, 0.75f}, {0.30f, 0.90f, 0.90f}};
    std_msgs::msg::ColorRGBA out;
    const auto& c = colors[rank % 10];
    out.r = c[0]; out.g = c[1]; out.b = c[2]; out.a = alpha;
    return out;
}

std_msgs::msg::ColorRGBA Color(const std::array<float, 4>& values) {
    std_msgs::msg::ColorRGBA out;
    out.r = values[0]; out.g = values[1];
    out.b = values[2]; out.a = values[3];
    return out;
}

std::array<float, 2> ConfigRange(
    const YAML::Node& node, const char* key,
    const std::array<float, 2>& fallback) {
    if (!node || !node[key]) return fallback;
    if (!node[key].IsSequence() || node[key].size() != 2) {
        throw std::runtime_error(
            std::string("range '") + key + "' must be a [min, max] pair");
    }
    std::array<float, 2> value{node[key][0].as<float>(),
                               node[key][1].as<float>()};
    if (!(value[0] < value[1])) {
        throw std::runtime_error(
            std::string("range '") + key + "' must satisfy min < max");
    }
    return value;
}

std::array<float, 4> ConfigColor(
    const YAML::Node& node, const char* key,
    const std::array<float, 4>& fallback) {
    if (!node[key] || !node[key].IsSequence() || node[key].size() != 4) {
        return fallback;
    }
    std::array<float, 4> value{};
    for (std::size_t i = 0; i < value.size(); ++i) {
        value[i] = std::clamp(node[key][i].as<float>(), 0.0f, 1.0f);
    }
    return value;
}

geometry_msgs::msg::Point Point(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

}  // namespace

class NavigationOrtRunner {
public:
    NavigationOrtRunner(const std::filesystem::path& encoder_path,
                        const std::filesystem::path& policy_head_path)
        : env_(ORT_LOGGING_LEVEL_WARNING, "navigation_onnx") {
        options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        encoder_ = std::make_unique<Ort::Session>(
            env_, encoder_path.c_str(), options_);
        policy_head_ = std::make_unique<Ort::Session>(
            env_, policy_head_path.c_str(), options_);

        if (encoder_->GetInputCount() != 1 || encoder_->GetOutputCount() != 1) {
            throw std::runtime_error(
                "GAT encoder ONNX must have one input and one output");
        }
        encoder_input_name_ = encoder_->GetInputNameAllocated(0, allocator_).get();
        encoder_output_name_ = encoder_->GetOutputNameAllocated(0, allocator_).get();
        const auto encoder_input_shape = encoder_->GetInputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        const auto encoder_output_shape = encoder_->GetOutputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        if (encoder_input_name_ != "nodes" ||
            encoder_output_name_ != "robot_embedding" ||
            encoder_input_shape.size() != 3 || encoder_input_shape.back() != 8 ||
            encoder_output_shape.size() != 2 || encoder_output_shape.back() != 16) {
            throw std::runtime_error(
                "GAT encoder ONNX must be nodes[batch,N,8] -> "
                "robot_embedding[batch,16]");
        }

        if (policy_head_->GetInputCount() != 2 ||
            policy_head_->GetOutputCount() != 1) {
            throw std::runtime_error(
                "navigation policy-head ONNX must have two inputs and one output");
        }
        head_embedding_input_name_ =
            policy_head_->GetInputNameAllocated(0, allocator_).get();
        head_local_input_name_ =
            policy_head_->GetInputNameAllocated(1, allocator_).get();
        head_output_name_ =
            policy_head_->GetOutputNameAllocated(0, allocator_).get();
        const auto embedding_shape = policy_head_->GetInputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        const auto local_shape = policy_head_->GetInputTypeInfo(1)
            .GetTensorTypeAndShapeInfo().GetShape();
        const auto action_shape = policy_head_->GetOutputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        if (head_embedding_input_name_ != "robot_embedding" ||
            head_local_input_name_ != "local_state" ||
            head_output_name_ != "policy_action" ||
            embedding_shape.size() != 2 || embedding_shape.back() != 16 ||
            local_shape.size() != 2 || local_shape.back() != 13 ||
            action_shape.size() != 2 || action_shape.back() != 3) {
            throw std::runtime_error(
                "navigation policy-head ONNX must be "
                "robot_embedding[batch,16] + local_state[batch,13] -> "
                "policy_action[batch,3]");
        }
    }

    std::array<float, 3> Act(const std::vector<float>& nodes,
                             const std::vector<float>& local_state) {
        constexpr std::size_t kNodeDimension = 8;
        constexpr std::size_t kMaximumNodes = 12;
        if (nodes.size() % kNodeDimension != 0 || !AllFinite(nodes)) {
            throw std::runtime_error("invalid dynamic GAT node observation");
        }
        const std::size_t node_count = nodes.size() / kNodeDimension;
        if (node_count < 2 || node_count > kMaximumNodes) {
            throw std::runtime_error("GAT node count must be in [2,12]");
        }
        if (local_state.size() != 13 || !AllFinite(local_state)) {
            throw std::runtime_error("invalid 13D navigation local state");
        }

        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        const std::array<int64_t, 3> node_shape{
            1, static_cast<int64_t>(node_count),
            static_cast<int64_t>(kNodeDimension)};
        auto node_tensor = Ort::Value::CreateTensor<float>(
            memory, const_cast<float*>(nodes.data()), nodes.size(),
            node_shape.data(), node_shape.size());
        const char* encoder_input_names[] = {encoder_input_name_.c_str()};
        const char* encoder_output_names[] = {encoder_output_name_.c_str()};
        auto encoder_outputs = encoder_->Run(
            Ort::RunOptions{nullptr}, encoder_input_names, &node_tensor, 1,
            encoder_output_names, 1);
        if (encoder_outputs.size() != 1 ||
            encoder_outputs.front().GetTensorTypeAndShapeInfo().GetElementCount() != 16) {
            throw std::runtime_error("GAT encoder returned a non-16D embedding");
        }

        const float* embedding = encoder_outputs.front().GetTensorData<float>();
        for (std::size_t i = 0; i < 16; ++i) {
            if (!std::isfinite(embedding[i])) {
                throw std::runtime_error("GAT encoder returned NaN/Inf");
            }
        }

        const std::array<int64_t, 2> local_shape{1, 13};
        auto local_tensor = Ort::Value::CreateTensor<float>(
            memory, const_cast<float*>(local_state.data()), local_state.size(),
            local_shape.data(), local_shape.size());
        std::array<Ort::Value, 2> head_inputs{
            std::move(encoder_outputs.front()), std::move(local_tensor)};
        const char* head_input_names[] = {
            head_embedding_input_name_.c_str(), head_local_input_name_.c_str()};
        const char* head_output_names[] = {head_output_name_.c_str()};
        auto outputs = policy_head_->Run(
            Ort::RunOptions{nullptr}, head_input_names, head_inputs.data(),
            head_inputs.size(), head_output_names, 1);
        if (outputs.size() != 1 ||
            outputs.front().GetTensorTypeAndShapeInfo().GetElementCount() != 3) {
            throw std::runtime_error(
                "navigation policy head returned a non-3D action");
        }
        const float* value = outputs.front().GetTensorData<float>();
        std::array<float, 3> action{value[0], value[1], value[2]};
        for (float item : action) {
            if (!std::isfinite(item)) {
                throw std::runtime_error(
                    "navigation policy head returned NaN/Inf");
            }
        }
        return action;
    }

private:
    Ort::Env env_;
    Ort::SessionOptions options_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::unique_ptr<Ort::Session> encoder_;
    std::unique_ptr<Ort::Session> policy_head_;
    std::string encoder_input_name_, encoder_output_name_;
    std::string head_embedding_input_name_, head_local_input_name_;
    std::string head_output_name_;
};

State_Navigation::State_Navigation(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string) {
    const auto state_cfg = param::config["FSM"][state_string];
    const auto policy_dir = param::parser_policy_dir(
        state_cfg["policy_dir"].as<std::string>());
    const auto nav_cfg = YAML::LoadFile(
        (policy_dir / "params/navigation.yaml").string());
    const auto repo_root = param::proj_dir.parent_path().parent_path().parent_path();
    const auto dpcbf_path = repo_root / "dpcbf/config/dpcbf_config.yaml";
    const auto dpcbf_cfg = YAML::LoadFile(dpcbf_path.string());
    boundary_params_ = dpcbf_ros_adapter::LoadBoundaryParams(dpcbf_path);
    if (boundary_params_.max_constraints > 10) {
        throw std::runtime_error(
            "Navigation supports at most 10 prioritized obstacle nodes");
    }

    high_level_dt_ = nav_cfg["high_level_dt"].as<double>(0.1);
    goal_radius_ = nav_cfg["goal_radius"].as<double>(0.3);
    goal_heading_tolerance_ = nav_cfg["goal_heading_tolerance_deg"]
        .as<double>(10.0) * kPi / 180.0;
    enable_random_goal_ = nav_cfg["enable_random_goal"].as<bool>(false) &&
                          param::is_simulation;
    hold_goal_after_reaching_ =
        nav_cfg["hold_goal_after_reaching"].as<bool>(true);
    goal_hold_obstacle_trigger_distance_ =
        nav_cfg["goal_hold_obstacle_trigger_distance"].as<double>(1.0);
    goal_hold_min_closing_speed_ =
        nav_cfg["goal_hold_min_closing_speed"].as<double>(0.05);
    random_goal_margin_ = nav_cfg["random_goal_margin"].as<double>(0.6);
    odom_velocity_filter_tau_ =
        nav_cfg["odometry_velocity_filter_tau"].as<double>(0.15);
    if (odom_velocity_filter_tau_ <= 0.0) {
        throw std::runtime_error(
            "odometry_velocity_filter_tau must be positive");
    }
    random_engine_.seed(nav_cfg["random_seed"].as<unsigned int>(42));
    const auto safety = nav_cfg["safety"];
    command_timeout_ = safety["command_timeout"].as<double>(0.25);
    obstacle_timeout_ = safety["obstacle_timeout"].as<double>(0.5);
    odometry_timeout_ = safety["odometry_timeout"].as<double>(0.2);
    const auto collision = safety["collision_stop"];
    collision_stop_enabled_ = collision["enabled"].as<bool>(true);
    collision_stop_distance_ = collision["surface_distance"].as<double>(0.15);
    const auto tilt = safety["tilt_protection"];
    tilt_enabled_ = tilt["enabled"].as<bool>(true);
    max_tilt_angle_ = tilt["max_angle"].as<double>(1.0);
    tilt_duration_ = tilt["duration"].as<double>(0.1);
    invalid_low_level_output_limit_ =
        safety["invalid_low_level_output_limit"].as<int>(3);

    const auto action_cfg = nav_cfg["actions"]["base_command"];
    action_range_[0] = ConfigRange(action_cfg, "lin_acc_x", action_range_[0]);
    action_range_[1] = ConfigRange(action_cfg, "lin_acc_y", action_range_[1]);
    action_range_[2] = ConfigRange(action_cfg, "ang_vel_z", action_range_[2]);

    const auto arena = dpcbf_cfg["dynamic_obstacles"]["arena"];
    if (arena["size"] && arena["size"].size() == 2) {
        arena_width_ = arena["size"][0].as<double>();
        arena_height_ = arena["size"][1].as<double>();
    }
    if (arena["center"] && arena["center"].size() == 2) {
        arena_center_x_ = arena["center"][0].as<double>();
        arena_center_y_ = arena["center"][1].as<double>();
    }
    const auto viz = nav_cfg["visualization"];
    visualization_enabled_ = viz["enabled"].as<bool>(true);
    relative_velocity_arrow_seconds_ =
        viz["relative_velocity_arrow_seconds"].as<double>(1.0);
    parabola_lateral_limit_ = viz["parabola_lateral_limit"].as<double>(1.0);
    parabola_backward_limit_ = viz["parabola_backward_limit"].as<double>(1.5);
    pulse_min_ = viz["goal_pulse_min_radius"].as<double>(0.08);
    pulse_max_ = viz["goal_pulse_max_radius"].as<double>(0.12);
    pulse_period_ = viz["goal_pulse_period"].as<double>(1.2);
    center_pulse_min_scale_ =
        viz["goal_center_pulse_min_percent"].as<double>(85.0) / 100.0;
    center_pulse_max_scale_ =
        viz["goal_center_pulse_max_percent"].as<double>(115.0) / 100.0;
    goal_heading_line_width_ =
        viz["goal_heading_line_width"].as<double>(0.025);
    goal_fill_color_ = ConfigColor(viz, "goal_fill_rgba", goal_fill_color_);
    goal_outline_color_ = ConfigColor(viz, "goal_outline_rgba", goal_outline_color_);
    goal_pulse_color_ = ConfigColor(viz, "goal_pulse_rgba", goal_pulse_color_);
    goal_center_color_ = ConfigColor(viz, "goal_center_rgba", goal_center_color_);
    goal_cone_tip_height_ = viz["goal_cone_tip_height"].as<double>(0.10);
    goal_cone_height_ = viz["goal_cone_height"].as<double>(0.34);
    goal_cone_top_radius_ = viz["goal_cone_top_radius"].as<double>(0.035);
    goal_cone_slices_ = viz["goal_cone_slices"].as<int>(12);
    goal_sphere_center_height_ =
        viz["goal_sphere_center_height"].as<double>(0.60);
    goal_sphere_radius_ = viz["goal_sphere_radius"].as<double>(0.05);
    if (!(goal_heading_tolerance_ >= 0.0 && goal_heading_tolerance_ <= kPi) ||
        goal_hold_obstacle_trigger_distance_ < 0.0 ||
        goal_hold_min_closing_speed_ < 0.0 ||
        pulse_period_ <= 0.0 || pulse_min_ <= 0.0 || pulse_max_ < pulse_min_ ||
        center_pulse_min_scale_ <= 0.0 ||
        center_pulse_max_scale_ < center_pulse_min_scale_ ||
        goal_heading_line_width_ <= 0.0 || goal_cone_tip_height_ <= 0.0 ||
        goal_cone_height_ <= 0.0 || goal_cone_top_radius_ <= 0.0 ||
        goal_cone_slices_ < 3 || goal_cone_slices_ > 32 ||
        goal_sphere_center_height_ <= 0.0 || goal_sphere_radius_ <= 0.0 ||
        goal_cone_top_radius_ >= goal_sphere_radius_ ||
        goal_cone_tip_height_ + goal_cone_height_ * center_pulse_max_scale_ >=
            goal_sphere_center_height_ -
                goal_sphere_radius_ * center_pulse_max_scale_) {
        throw std::runtime_error("invalid Navigation goal tolerance or visualization scale");
    }

    low_env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile((policy_dir / "params/low_level_deploy.yaml").string()),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
            FSMState::lowstate));
    low_env_->alg = std::make_unique<isaaclab::OrtRunner>(
        policy_dir / "exported/low_level_locomotion_policy.onnx");
    high_policy_ = std::make_unique<NavigationOrtRunner>(
        policy_dir / "exported/high_level_navigation_policy/gat_encoder.onnx",
        policy_dir / "exported/high_level_navigation_policy/policy_head.onnx");
    const auto command_ranges =
        low_env_->cfg["commands"]["base_velocity"]["ranges"];
    velocity_command_range_[0] = ConfigRange(
        command_ranges, "lin_vel_x", velocity_command_range_[0]);
    velocity_command_range_[1] = ConfigRange(
        command_ranges, "lin_vel_y", velocity_command_range_[1]);
    velocity_command_range_[2] = ConfigRange(
        command_ranges, "ang_vel_z", velocity_command_range_[2]);
    joint_targets_.assign(29, 0.0f);

    node_ = std::make_shared<rclcpp::Node>("g1_navigation_controller");
    const auto topics = nav_cfg["topics"];
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topics["odometry"].as<std::string>(), sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { OnOdometry(*msg); });
    obstacle_sub_ = node_->create_subscription<obstacle_detector::msg::Obstacles>(
        topics["obstacles"].as<std::string>(), sensor_qos,
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr msg) {
            OnObstacles(*msg);
        });
    // A goal is an operator command and must not be replayed to a later FSM
    // entry. Reliable + volatile delivers live commands without latching an
    // earlier goal in DDS.
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topics["goal"].as<std::string>(), goal_qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnGoal(*msg); });
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        topics["stop"].as<std::string>(), rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Empty::ConstSharedPtr msg) { OnStop(*msg); });
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        topics["markers"].as<std::string>(), rclcpp::QoS(1).best_effort());
    command_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        topics["command"].as<std::string>("/navigation/cmd_vel"),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    ros_thread_ = std::thread([this] { executor_->spin(); });

    registered_checks.emplace(registered_checks.begin() + 1,
        std::make_pair([this]() {
            if (!low_level_failed_.load()) return false;
            return true;
        }, FSMStringMap.right.at("Passive")));
    registered_checks.emplace(registered_checks.begin() + 1,
        std::make_pair([this]() {
            if (!tilt_enabled_) return false;
            const bool tilted = isaaclab::mdp::bad_orientation(
                low_env_.get(), static_cast<float>(max_tilt_angle_));
            if (!tilted) {
                tilt_started_ = {};
                return false;
            }
            if (tilt_started_.time_since_epoch().count() == 0) {
                tilt_started_ = SteadyClock::now();
                return false;
            }
            return AgeSeconds(tilt_started_) >= tilt_duration_;
        }, FSMStringMap.right.at("Passive")));
}

State_Navigation::~State_Navigation() {
    exit();
    if (executor_) executor_->cancel();
    if (ros_thread_.joinable()) ros_thread_.join();
}

void State_Navigation::enter() {
    accept_goal_commands_.store(false);
    for (int i = 0; i < static_cast<int>(low_env_->robot->data.joint_stiffness.size()); ++i) {
        lowcmd->msg_.motor_cmd()[i].kp() = low_env_->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = low_env_->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
    }
    low_level_failed_.store(false);
    tilt_started_ = {};
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_ = {};
        velocity_command_ = {0.0f, 0.0f, 0.0f};
        previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
    }
    low_env_->set_external_velocity_command({0.0f, 0.0f, 0.0f});
    PublishCommand({0.0f, 0.0f, 0.0f});
    low_env_->reset();
    accept_goal_commands_.store(true);
    policy_thread_running_.store(true);
    policy_thread_ = std::thread(&State_Navigation::PolicyLoop, this);
}

void State_Navigation::exit() {
    accept_goal_commands_.store(false);
    policy_thread_running_.store(false);
    if (policy_thread_.joinable()) policy_thread_.join();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_ = {};
        previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
    }
    SetZeroCommand();
}

void State_Navigation::run() {
    std::vector<float> targets;
    {
        std::lock_guard<std::mutex> lock(joint_target_mutex_);
        targets = joint_targets_;
    }
    if (targets.size() != 29 || !AllFinite(targets)) return;
    for (std::size_t i = 0; i < targets.size(); ++i) {
        lowcmd->msg_.motor_cmd()[low_env_->robot->data.joint_ids_map[i]].q() = targets[i];
    }
}

void State_Navigation::OnOdometry(const nav_msgs::msg::Odometry& msg) {
    const auto now = SteadyClock::now();
    const double yaw = QuaternionYaw(msg.pose.pose.orientation);
    std::lock_guard<std::mutex> lock(data_mutex_);
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double wz = msg.twist.twist.angular.z;
    if (robot_.valid) {
        const double dt = std::chrono::duration<double>(now - robot_.received).count();
        if (dt > 1.0e-3 && dt < 0.5) {
            const double raw_vx = (msg.pose.pose.position.x - robot_.x) / dt;
            const double raw_vy = (msg.pose.pose.position.y - robot_.y) / dt;
            const double raw_wz = WrapAngle(yaw - robot_.yaw) / dt;
            const double alpha =
                std::clamp(dt / (odom_velocity_filter_tau_ + dt), 0.0, 1.0);
            vx = (1.0 - alpha) * robot_.vx_world + alpha * raw_vx;
            vy = (1.0 - alpha) * robot_.vy_world + alpha * raw_vy;
            wz = (1.0 - alpha) * robot_.yaw_rate + alpha * raw_wz;
        }
    }
    robot_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
              vx, vy, wz, now, true};
}

void State_Navigation::OnObstacles(const obstacle_detector::msg::Obstacles& msg) {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != "odom") {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Ignoring /obstacles_safe in frame '%s'; expected odom",
            msg.header.frame_id.c_str());
        return;
    }
    std::vector<dpcbf::ObstacleState> next;
    next.reserve(msg.circles.size());
    for (const auto& circle : msg.circles) {
        if (!std::isfinite(circle.center.x) || !std::isfinite(circle.center.y) ||
            !std::isfinite(circle.radius) || circle.radius <= 0.0) continue;
        dpcbf::ObstacleState obstacle;
        obstacle.x = circle.center.x;
        obstacle.y = circle.center.y;
        obstacle.radius = circle.radius;
        obstacle.velocity_x = circle.velocity.x;
        obstacle.velocity_y = circle.velocity.y;
        obstacle.id = static_cast<int>(circle.uid);
        next.push_back(obstacle);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacles_ = std::move(next);
    obstacles_received_ = SteadyClock::now();
    obstacles_received_once_ = true;
}

void State_Navigation::OnGoal(const geometry_msgs::msg::PoseStamped& msg) {
    if (!accept_goal_commands_.load()) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Ignoring /navigation/goal while Navigation is inactive");
        return;
    }
    if (!msg.header.frame_id.empty() && msg.header.frame_id != "odom") {
        RCLCPP_WARN(node_->get_logger(),
                    "Ignoring goal in frame '%s'; expected odom",
                    msg.header.frame_id.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_ = {msg.pose.position.x, msg.pose.position.y,
             QuaternionYaw(msg.pose.orientation), true, true};
    velocity_command_ = {0.0f, 0.0f, 0.0f};
    previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
    RCLCPP_INFO(node_->get_logger(),
                "Navigation goal received: x=%.2f y=%.2f yaw=%.2f",
                goal_.x, goal_.y, goal_.yaw);
}

void State_Navigation::OnStop(const std_msgs::msg::Empty&) {
    ClearGoalCommandState();
}

void State_Navigation::ClearGoalCommandState() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_.active = false;
    goal_.external = false;
    velocity_command_ = {0.0f, 0.0f, 0.0f};
    previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
}

void State_Navigation::SetZeroCommand() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    velocity_command_ = {0.0f, 0.0f, 0.0f};
    low_env_->set_external_velocity_command(velocity_command_);
    PublishCommand(velocity_command_);
}

// The command is expressed in the robot heading frame: linear.x/y are the
// sagittal/lateral velocities fed to the low-level policy, angular.z the
// yaw rate.
void State_Navigation::PublishCommand(const std::array<float, 3>& command) {
    if (!command_pub_) return;
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = command[0];
    msg.twist.linear.y = command[1];
    msg.twist.angular.z = command[2];
    command_pub_->publish(msg);
}

void State_Navigation::CreateRandomGoal(const RobotSnapshot& robot) {
    std::uniform_real_distribution<double> x_dist(
        arena_center_x_ - 0.5 * arena_width_ + random_goal_margin_,
        arena_center_x_ + 0.5 * arena_width_ - random_goal_margin_);
    std::uniform_real_distribution<double> y_dist(
        arena_center_y_ - 0.5 * arena_height_ + random_goal_margin_,
        arena_center_y_ + 0.5 * arena_height_ - random_goal_margin_);
    std::uniform_real_distribution<double> yaw_dist(-kPi, kPi);
    for (int attempt = 0; attempt < 100; ++attempt) {
        const double x = x_dist(random_engine_);
        const double y = y_dist(random_engine_);
        if (std::hypot(x - robot.x, y - robot.y) < 1.0) continue;
        bool clear = true;
        for (const auto& obstacle : obstacles_) {
            if (std::hypot(x - obstacle.x, y - obstacle.y) <
                goal_radius_ + obstacle.radius + 0.2) {
                clear = false;
                break;
            }
        }
        if (clear) {
            goal_ = {x, y, yaw_dist(random_engine_), true, false};
            RCLCPP_INFO(node_->get_logger(),
                        "Random navigation goal: x=%.2f y=%.2f yaw=%.2f",
                        goal_.x, goal_.y, goal_.yaw);
            return;
        }
    }
    goal_.active = false;
}

bool State_Navigation::HasApproachingGoalHoldThreat(
    const RobotSnapshot& robot,
    const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected) const {
    for (const auto& item : selected) {
        const double dx = item.obstacle.x - robot.x;
        const double dy = item.obstacle.y - robot.y;
        const double center_distance = std::hypot(dx, dy);
        if (center_distance <= 1.0e-9) {
            continue;
        }
        const double surface_distance = center_distance -
            (boundary_params_.robot_radius + item.obstacle.radius);
        const double closing_speed =
            -(dx * item.relative_velocity_world[0] +
              dy * item.relative_velocity_world[1]) / center_distance;
        if (surface_distance <= goal_hold_obstacle_trigger_distance_ &&
            closing_speed >= goal_hold_min_closing_speed_) {
            return true;
        }
    }
    return false;
}

std::vector<float> State_Navigation::BuildNodeObservation(
    const RobotSnapshot& robot, const GoalSnapshot& goal,
    const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected) const {
    constexpr std::size_t kMaximumObstacles = 10;
    constexpr std::size_t kNodeDimension = 8;
    const std::size_t obstacle_count =
        std::min(selected.size(), kMaximumObstacles);
    std::vector<float> nodes;
    nodes.reserve((2 + obstacle_count) * kNodeDimension);
    const auto append_node = [&nodes](float is_robot, float is_obstacle,
                                      float is_goal, float x, float y,
                                      float radius, float vx, float vy) {
        nodes.insert(nodes.end(), {is_robot, is_obstacle, is_goal,
                                   x, y, radius, vx, vy});
    };

    // The encoder returns only the first node's latent, so robot must remain
    // node 0. All coordinates and velocities are robot-relative in body frame.
    append_node(1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                static_cast<float>(boundary_params_.robot_radius),
                0.0f, 0.0f);

    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    const double robot_vx_b = c * robot.vx_world + s * robot.vy_world;
    const double robot_vy_b = -s * robot.vx_world + c * robot.vy_world;
    const double goal_dx = goal.x - robot.x;
    const double goal_dy = goal.y - robot.y;
    const double goal_bx = c * goal_dx + s * goal_dy;
    const double goal_by = -s * goal_dx + c * goal_dy;

    // Goal is node 1. It is stationary in the world, hence relative velocity
    // is the negative robot body velocity.
    append_node(0.0f, 0.0f, 1.0f,
                static_cast<float>(goal_bx),
                static_cast<float>(goal_by),
                static_cast<float>(goal_radius_),
                static_cast<float>(-robot_vx_b),
                static_cast<float>(-robot_vy_b));

    // No dummy nodes and no valid mask: append only the prioritized obstacles
    // actually received from perception. Their radius is circle.radius from
    // /obstacles_safe; the GAT computes pairwise surface distance internally.
    for (std::size_t i = 0; i < obstacle_count; ++i) {
        const auto& item = selected[i];
        const double dx = item.obstacle.x - robot.x;
        const double dy = item.obstacle.y - robot.y;
        append_node(
            0.0f, 1.0f, 0.0f,
            static_cast<float>(c * dx + s * dy),
            static_cast<float>(-s * dx + c * dy),
            static_cast<float>(item.obstacle.radius),
            static_cast<float>(c * item.relative_velocity_world[0] +
                               s * item.relative_velocity_world[1]),
            static_cast<float>(-s * item.relative_velocity_world[0] +
                               c * item.relative_velocity_world[1]));
    }
    return nodes;
}

std::vector<float> State_Navigation::BuildLocalState(
    const RobotSnapshot& robot, const GoalSnapshot& goal) const {
    std::vector<float> local_state(13, 0.0f);
    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    const double goal_dx = goal.x - robot.x;
    const double goal_dy = goal.y - robot.y;
    const double goal_bx = c * goal_dx + s * goal_dy;
    const double goal_by = -s * goal_dx + c * goal_dy;
    const double robot_vx_b = c * robot.vx_world + s * robot.vy_world;
    const double robot_vy_b = -s * robot.vx_world + c * robot.vy_world;
    const double heading_error = WrapAngle(goal.yaw - robot.yaw);
    local_state[0] = static_cast<float>(goal_bx);
    local_state[1] = static_cast<float>(goal_by);
    local_state[2] = static_cast<float>(std::sin(heading_error));
    local_state[3] = static_cast<float>(std::cos(heading_error));
    local_state[4] = static_cast<float>(robot_vx_b);
    local_state[5] = static_cast<float>(robot_vy_b);
    local_state[6] = static_cast<float>(robot.yaw_rate);
    std::array<float, 3> command;
    std::array<float, 3> previous_action;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        command = velocity_command_;
        previous_action = previous_normalized_action_;
    }
    for (int i = 0; i < 3; ++i) {
        local_state[7 + i] = command[i];
        local_state[10 + i] = previous_action[i];
    }
    return local_state;
}

bool State_Navigation::UpdateHighLevel() {
    RobotSnapshot robot;
    GoalSnapshot goal;
    std::vector<dpcbf::ObstacleState> obstacles;
    bool obstacles_seen;
    SteadyClock::time_point obstacle_stamp;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot = robot_;
        goal = goal_;
        obstacles = obstacles_;
        obstacles_seen = obstacles_received_once_;
        obstacle_stamp = obstacles_received_;
    }
    {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        const double gyro_z =
            FSMState::lowstate->msg_.imu_state().gyroscope()[2];
        if (std::isfinite(gyro_z)) robot.yaw_rate = gyro_z;
    }
    const double odometry_age = AgeSeconds(robot.received);
    const double obstacle_age = AgeSeconds(obstacle_stamp);
    if (!robot.valid) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Navigation stopped and goal cleared: waiting for /odom");
        ClearGoalCommandState();
        SetZeroCommand();
        return false;
    }
    if (odometry_age > odometry_timeout_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Navigation stopped and goal cleared: /odom stale "
            "(%.3f s > %.3f s); a new goal is required",
            odometry_age, odometry_timeout_);
        ClearGoalCommandState();
        SetZeroCommand();
        return false;
    }
    if (!obstacles_seen) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Navigation stopped and goal cleared: waiting for "
            "/obstacles_safe");
        ClearGoalCommandState();
        SetZeroCommand();
        return false;
    }
    if (obstacle_age > obstacle_timeout_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Navigation stopped and goal cleared: /obstacles_safe stale "
            "(%.3f s > %.3f s); a new goal is required",
            obstacle_age, obstacle_timeout_);
        // The same command-state reset used by a right-click Stop. Clearing
        // the goal latches the stop across sensor recovery: inference cannot
        // resume until the operator explicitly sends a new goal.
        ClearGoalCommandState();
        SetZeroCommand();
        return false;
    }
    if (!goal.active) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Navigation stopped: waiting for a goal");
        SetZeroCommand();
        return false;
    }
    dpcbf::RobotState dpcbf_robot;
    dpcbf_robot.x = robot.x; dpcbf_robot.y = robot.y; dpcbf_robot.phi = robot.yaw;
    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    dpcbf_robot.sagittal_velocity = c * robot.vx_world + s * robot.vy_world;
    dpcbf_robot.lateral_velocity = -s * robot.vx_world + c * robot.vy_world;
    const auto selected = dpcbf_ros_adapter::SelectAndEvaluate(
        boundary_params_, dpcbf_robot, obstacles);

    const double goal_distance = std::hypot(goal.x - robot.x, goal.y - robot.y);
    const double goal_heading_error = std::abs(WrapAngle(goal.yaw - robot.yaw));
    if (goal_distance <= goal_radius_ &&
        goal_heading_error <= goal_heading_tolerance_) {
        if (enable_random_goal_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            velocity_command_ = {0.0f, 0.0f, 0.0f};
            previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
            CreateRandomGoal(robot);
            low_env_->set_external_velocity_command(velocity_command_);
            PublishCommand(velocity_command_);
            return false;
        }

        const bool approaching_threat = hold_goal_after_reaching_ &&
            HasApproachingGoalHoldThreat(robot, selected);
        if (!approaching_threat) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                velocity_command_ = {0.0f, 0.0f, 0.0f};
                previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
                if (!hold_goal_after_reaching_) {
                    goal_.active = false;
                }
                low_env_->set_external_velocity_command(velocity_command_);
                PublishCommand(velocity_command_);
            }
            PublishMarkers(robot, goal, selected);
            return false;
        }
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Goal hold avoidance active: an obstacle is approaching");
    } else if (goal_distance <= goal_radius_) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Goal position reached; aligning heading (error %.1f deg, tolerance %.1f deg)",
            goal_heading_error * 180.0 / kPi,
            goal_heading_tolerance_ * 180.0 / kPi);
    }

    if (collision_stop_enabled_) {
        for (const auto& obstacle : obstacles) {
            const double clearance =
                std::hypot(obstacle.x - robot.x, obstacle.y - robot.y) -
                (boundary_params_.robot_radius + obstacle.radius);
            if (clearance < collision_stop_distance_) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Navigation emergency stop: obstacle %d clearance %.3f m",
                    obstacle.id, clearance);
                SetZeroCommand();
                PublishMarkers(robot, goal, selected);
                return false;
            }
        }
    }
    try {
        const auto nodes = BuildNodeObservation(robot, goal, selected);
        const auto local_state = BuildLocalState(robot, goal);
        auto action = high_policy_->Act(nodes, local_state);
        for (int i = 0; i < 3; ++i) {
            action[i] = std::clamp(
                action[i], action_range_[i][0], action_range_[i][1]);
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            velocity_command_[0] = std::clamp(
                velocity_command_[0] + action[0] * static_cast<float>(high_level_dt_),
                velocity_command_range_[0][0], velocity_command_range_[0][1]);
            velocity_command_[1] = std::clamp(
                velocity_command_[1] + action[1] * static_cast<float>(high_level_dt_),
                velocity_command_range_[1][0], velocity_command_range_[1][1]);
            velocity_command_[2] = std::clamp(
                action[2],
                velocity_command_range_[2][0], velocity_command_range_[2][1]);
            for (int i = 0; i < 3; ++i) {
                const float center =
                    0.5f * (action_range_[i][0] + action_range_[i][1]);
                const float half_range =
                    0.5f * (action_range_[i][1] - action_range_[i][0]);
                previous_normalized_action_[i] = std::clamp(
                    (action[i] - center) / half_range, -1.0f, 1.0f);
            }
            low_env_->set_external_velocity_command(velocity_command_);
            PublishCommand(velocity_command_);
            last_high_success_ = SteadyClock::now();
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "Navigation command: vx=%.3f vy=%.3f wz=%.3f (obstacles=%zu)",
                velocity_command_[0], velocity_command_[1], velocity_command_[2],
                selected.size());
        }
        PublishMarkers(robot, goal, selected);
        return true;
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "Navigation inference stopped command: %s", error.what());
        SetZeroCommand();
        return false;
    }
}

void State_Navigation::PolicyLoop() {
    using namespace std::chrono_literals;
    const auto low_period = std::chrono::duration<double>(low_env_->step_dt);
    const int high_stride = std::max(1, static_cast<int>(
        std::lround(high_level_dt_ / low_env_->step_dt)));
    int tick = 0, invalid = 0;
    auto next = SteadyClock::now();
    while (policy_thread_running_.load()) {
        if (tick % high_stride == 0) UpdateHighLevel();
        if (AgeSeconds(last_high_success_) > command_timeout_) SetZeroCommand();
        try {
            low_env_->step();
            const auto targets = low_env_->action_manager->processed_actions();
            if (targets.size() != 29 || !AllFinite(targets)) {
                throw std::runtime_error("invalid low-level joint target");
            }
            {
                std::lock_guard<std::mutex> lock(joint_target_mutex_);
                joint_targets_ = targets;
            }
            invalid = 0;
        } catch (const std::exception& error) {
            ++invalid;
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                  "Low-level policy failure: %s", error.what());
            if (invalid >= invalid_low_level_output_limit_) {
                low_level_failed_.store(true);
                SetZeroCommand();
            }
        }
        ++tick;
        next += std::chrono::duration_cast<SteadyClock::duration>(low_period);
        std::this_thread::sleep_until(next);
    }
}

void State_Navigation::PublishMarkers(
    const RobotSnapshot& robot, const GoalSnapshot& goal,
    const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected) {
    if (!visualization_enabled_ || !marker_pub_) return;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;
    const auto stamp = node_->now();
    auto base_marker = [&](const std::string& ns, int type) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id++;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);
        return marker;
    };
    auto flat_box = [&](const std::string& ns, double center_x, double center_y,
                        double yaw, double length, double width, double height,
                        double z, const std_msgs::msg::ColorRGBA& color) {
        auto box = base_marker(ns, visualization_msgs::msg::Marker::CUBE);
        box.pose.position = Point(center_x, center_y, z);
        box.pose.orientation.z = std::sin(0.5 * yaw);
        box.pose.orientation.w = std::cos(0.5 * yaw);
        box.scale.x = length; box.scale.y = width; box.scale.z = height;
        box.color = color;
        array.markers.push_back(std::move(box));
    };
    for (const auto& item : selected) {
        auto curve = base_marker("dpcbf_boundary", visualization_msgs::msg::Marker::LINE_STRIP);
        curve.scale.x = 0.025;
        curve.color = Color(item.rank);
        constexpr int samples = 80;
        for (int j = 0; j <= samples; ++j) {
            const double local_y = -parabola_lateral_limit_ +
                2.0 * parabola_lateral_limit_ * j / samples;
            const double local_x = item.boundary_vertex_x -
                item.boundary_curvature * local_y * local_y;
            if (local_x < -parabola_backward_limit_) continue;
            const double ca = std::cos(item.los_angle), sa = std::sin(item.los_angle);
            curve.points.push_back(Point(robot.x + ca * local_x - sa * local_y,
                                         robot.y + sa * local_x + ca * local_y, 0.008));
        }
        array.markers.push_back(curve);
        const double arrow_x = item.relative_velocity_world[0] *
                               relative_velocity_arrow_seconds_;
        const double arrow_y = item.relative_velocity_world[1] *
                               relative_velocity_arrow_seconds_;
        const double arrow_length = std::hypot(arrow_x, arrow_y);
        if (arrow_length > 1.0e-3) {
            const double yaw = std::atan2(arrow_y, arrow_x);
            const double ca = std::cos(yaw), sa = std::sin(yaw);
            const auto color = Color(item.rank, 0.9f);
            const double head_length = std::min(0.16, std::max(0.06, 0.30 * arrow_length));
            const double shaft_length = std::max(0.01, arrow_length - head_length);
            flat_box("relative_velocity_flat_shaft",
                     robot.x + 0.5 * shaft_length * ca,
                     robot.y + 0.5 * shaft_length * sa,
                     yaw, shaft_length, 0.028, 0.006, 0.009, color);
            // Six shallow boxes form a filled triangular-prism arrowhead
            // without using MuJoCo's cylindrical/conical ARROW primitive.
            constexpr int head_slices = 6;
            for (int slice = 0; slice < head_slices; ++slice) {
                const double fraction = (slice + 0.5) / head_slices;
                const double slice_length = head_length / head_slices;
                const double slice_width = std::max(0.008, 0.10 * (1.0 - fraction));
                const double along = shaft_length + (slice + 0.5) * slice_length;
                flat_box("relative_velocity_flat_head",
                         robot.x + along * ca, robot.y + along * sa,
                         yaw, slice_length * 1.05, slice_width, 0.006, 0.009, color);
            }
            // A negligible-width vector preserves the existing 2-D operator
            // UI representation; the MuJoCo bridge intentionally skips it.
            auto vector = base_marker("relative_velocity_vector",
                                      visualization_msgs::msg::Marker::LINE_STRIP);
            vector.scale.x = 0.001;
            vector.color = color;
            vector.points.push_back(Point(robot.x, robot.y, 0.009));
            vector.points.push_back(Point(robot.x + arrow_x,
                                          robot.y + arrow_y, 0.009));
            array.markers.push_back(std::move(vector));
        }
    }
    if (goal.active) {
        auto disk = base_marker("goal", visualization_msgs::msg::Marker::CYLINDER);
        disk.pose.position = Point(goal.x, goal.y, 0.0025);
        disk.scale.x = 2.0 * goal_radius_; disk.scale.y = 2.0 * goal_radius_;
        disk.scale.z = 0.004;
        disk.color = Color(goal_fill_color_);
        array.markers.push_back(disk);
        auto outline = base_marker("goal_outline", visualization_msgs::msg::Marker::LINE_STRIP);
        outline.scale.x = 0.018;
        outline.color = Color(goal_outline_color_);
        for (int j = 0; j <= 64; ++j) {
            const double a = 2.0 * kPi * j / 64.0;
            outline.points.push_back(Point(
                goal.x + goal_radius_ * std::cos(a),
                goal.y + goal_radius_ * std::sin(a), 0.006));
        }
        array.markers.push_back(outline);
        const double heading_length = 0.92 * goal_radius_;
        flat_box("goal_heading",
                 goal.x + 0.5 * heading_length * std::cos(goal.yaw),
                 goal.y + 0.5 * heading_length * std::sin(goal.yaw),
                 goal.yaw, heading_length, goal_heading_line_width_, 0.004,
                 0.0095, Color(goal_center_color_));
        const double phase = std::fmod(node_->now().seconds(), pulse_period_) / pulse_period_;
        const double pulse = 0.5 - 0.5 * std::cos(2.0 * kPi * phase);
        const double radius = pulse_min_ + (pulse_max_ - pulse_min_) * pulse;
        auto ring = base_marker("goal_pulse", visualization_msgs::msg::Marker::LINE_STRIP);
        ring.scale.x = 0.020;
        ring.color = Color(goal_pulse_color_);
        for (int j = 0; j <= 48; ++j) {
            const double a = 2.0 * kPi * j / 48.0;
            ring.points.push_back(Point(goal.x + radius * std::cos(a),
                                        goal.y + radius * std::sin(a), 0.014));
        }
        array.markers.push_back(ring);
        const double center_scale = center_pulse_min_scale_ +
            (center_pulse_max_scale_ - center_pulse_min_scale_) * pulse;
        const double cone_height = goal_cone_height_ * center_scale;
        const double cone_slice_height = cone_height / goal_cone_slices_;
        for (int slice = 0; slice < goal_cone_slices_; ++slice) {
            const double fraction = (slice + 0.5) / goal_cone_slices_;
            const double cone_radius = goal_cone_top_radius_ *
                                       center_scale * fraction;
            auto cone_slice = base_marker(
                "goal_center_cone", visualization_msgs::msg::Marker::CYLINDER);
            cone_slice.pose.position = Point(
                goal.x, goal.y,
                goal_cone_tip_height_ + (slice + 0.5) * cone_slice_height);
            cone_slice.scale.x = 2.0 * cone_radius;
            cone_slice.scale.y = 2.0 * cone_radius;
            cone_slice.scale.z = 1.04 * cone_slice_height;
            cone_slice.color = Color(goal_center_color_);
            array.markers.push_back(std::move(cone_slice));
        }
        const double sphere_radius = goal_sphere_radius_ * center_scale;
        auto sphere = base_marker("goal_center", visualization_msgs::msg::Marker::SPHERE);
        sphere.pose.position = Point(goal.x, goal.y, goal_sphere_center_height_);
        sphere.scale.x = 2.0 * sphere_radius; sphere.scale.y = 2.0 * sphere_radius;
        sphere.scale.z = 2.0 * sphere_radius;
        sphere.color = Color(goal_center_color_);
        array.markers.push_back(sphere);
    }
    std::array<float, 3> command;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        command = velocity_command_;
    }
    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    auto linear = base_marker("navigation_command", visualization_msgs::msg::Marker::ARROW);
    linear.scale.x = 0.045; linear.scale.y = 0.09; linear.scale.z = 0.12;
    linear.color.r = 1.0f; linear.color.a = 0.7f;
    linear.points.push_back(Point(robot.x, robot.y, 1.55));
    linear.points.push_back(Point(robot.x + c * command[0] - s * command[1],
                                  robot.y + s * command[0] + c * command[1], 1.55));
    array.markers.push_back(linear);
    if (std::abs(command[2]) > 1.0e-3f) {
        auto angular = base_marker("navigation_command", visualization_msgs::msg::Marker::ARROW);
        angular.scale.x = 0.045; angular.scale.y = 0.09; angular.scale.z = 0.12;
        angular.color.r = 1.0f; angular.color.a = 0.7f;
        angular.points.push_back(Point(robot.x, robot.y, 1.55));
        angular.points.push_back(Point(robot.x, robot.y, 1.55 + 0.5 * command[2]));
        array.markers.push_back(angular);
    }
    marker_pub_->publish(array);
}
