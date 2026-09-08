#include "State_Navigation.h"

#include "isaaclab/algorithms/algorithms.h"
#include "onnxruntime_session_options_config_keys.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "unitree_articulation.h"
#include "dpcbf_ros_adapter/dpcbf_boundary_config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kPi = 3.14159265358979323846;

// exit() runs on the 1 kHz FSM thread, where a blocked cycle publishes no
// motor command at all, so the high-level loop waits in slices instead of one
// 100 ms sleep: stopping it never costs the FSM more than this.
constexpr auto kHighLevelStopPoll = std::chrono::milliseconds(5);

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

std::int64_t SteadyNanoseconds(std::chrono::steady_clock::time_point stamp) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
}

// The high-level result is timestamped across threads, so it is carried as a
// plain integer the low-level loop can read without taking data_mutex_.
double AgeSeconds(std::int64_t nanoseconds) {
    if (nanoseconds == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch() -
        std::chrono::nanoseconds(nanoseconds)).count();
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
                        const std::filesystem::path& policy_head_path,
                        int intra_op_num_threads)
        : env_(ORT_LOGGING_LEVEL_WARNING, "navigation_onnx") {
        options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        // CreateSession copies the options, so configuring them once here
        // reaches both sessions. 0 keeps ONNX Runtime's own pool sizing.
        if (intra_op_num_threads > 0) {
            options_.SetIntraOpNumThreads(intra_op_num_threads);
            options_.SetInterOpNumThreads(1);
            options_.SetExecutionMode(ORT_SEQUENTIAL);
            options_.AddConfigEntry(
                kOrtSessionOptionsConfigAllowIntraOpSpinning, "0");
            options_.AddConfigEntry(
                kOrtSessionOptionsConfigAllowInterOpSpinning, "0");
        }
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
    if (!(high_level_dt_ > 0.0) || !std::isfinite(high_level_dt_)) {
        // PolicyLoop used to clamp a bad period through std::max(1, stride).
        // The high-level tick now paces its own thread, where a non-positive
        // period free-runs ONNX inference against the 1 kHz FSM thread.
        throw std::runtime_error("high_level_dt must be positive");
    }
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
    // Absent from an older navigation.yaml. yaml-cpp returns a zombie node
    // for a missing key and throws InvalidNode on the next operator[], so
    // the block has to be probed before its entries are read. The defaults
    // reproduce the previous behaviour exactly: warn on a missed deadline,
    // act on nothing.
    const auto policy_deadline = safety["policy_deadline"];
    if (policy_deadline && policy_deadline.IsMap()) {
        policy_deadline_tolerance_ =
            policy_deadline["tolerance"].as<double>(policy_deadline_tolerance_);
        policy_deadline_warmup_steps_ = policy_deadline["warmup_steps"]
            .as<int>(policy_deadline_warmup_steps_);
        policy_deadline_stop_streak_ = policy_deadline["stop_streak"]
            .as<int>(policy_deadline_stop_streak_);
        policy_deadline_fault_streak_ = policy_deadline["fault_streak"]
            .as<int>(policy_deadline_fault_streak_);
    }
    if (!std::isfinite(policy_deadline_tolerance_) ||
        policy_deadline_tolerance_ < 0.0 ||
        policy_deadline_warmup_steps_ < 0 ||
        policy_deadline_stop_streak_ < 0 ||
        policy_deadline_fault_streak_ < 0 ||
        (policy_deadline_stop_streak_ > 0 &&
         policy_deadline_fault_streak_ > 0 &&
         policy_deadline_fault_streak_ < policy_deadline_stop_streak_)) {
        throw std::runtime_error(
            "invalid safety.policy_deadline: entries must be finite and "
            "non-negative and fault_streak must not precede stop_streak");
    }
    const auto watchdog = safety["joint_target_watchdog"];
    if (watchdog && watchdog.IsMap()) {
        watchdog_enabled_ = watchdog["enabled"].as<bool>(watchdog_enabled_);
        watchdog_warn_age_ = watchdog["warn_age"].as<double>(0.06);
        watchdog_fade_age_ = watchdog["fade_age"].as<double>(0.20);
        watchdog_fault_age_ = watchdog["fault_age"].as<double>(0.35);
        watchdog_startup_age_ = watchdog["startup_age"].as<double>(1.0);
        watchdog_restore_seconds_ =
            watchdog["restore_seconds"].as<double>(0.5);
        watchdog_fade_stiffness_scale_ =
            watchdog["fade_stiffness_scale"].as<float>(0.0f);
        watchdog_fade_damping_scale_ =
            watchdog["fade_damping_scale"].as<float>(1.0f);
        watchdog_passive_on_fault_ =
            watchdog["passive_on_fault"].as<bool>(true);
    }
    if (!(watchdog_warn_age_ > 0.0 &&
          watchdog_warn_age_ <= watchdog_fade_age_ &&
          watchdog_fade_age_ < watchdog_fault_age_ &&
          watchdog_startup_age_ >= watchdog_fade_age_ &&
          watchdog_restore_seconds_ > 0.0) ||
        !std::isfinite(watchdog_fade_stiffness_scale_) ||
        !std::isfinite(watchdog_fade_damping_scale_) ||
        watchdog_fade_stiffness_scale_ < 0.0f ||
        watchdog_fade_stiffness_scale_ > 1.0f ||
        watchdog_fade_damping_scale_ < 0.0f ||
        watchdog_fade_damping_scale_ > 1.0f) {
        throw std::runtime_error(
            "safety.joint_target_watchdog must satisfy 0 < warn_age <= "
            "fade_age < fault_age, startup_age >= fade_age, "
            "restore_seconds > 0, and fade scales in [0, 1]");
    }

    // Each session otherwise builds an intra-op pool sized to the core count
    // whose workers spin before blocking; three of them on the eight-core
    // Orin NX delay the 1 kHz control thread. 0 restores that default.
    const auto runtime = nav_cfg["runtime"];
    const int onnx_intra_op_threads = (runtime && runtime.IsMap())
        ? runtime["onnx_intra_op_threads"].as<int>(1) : 1;
    if (onnx_intra_op_threads < 0 || onnx_intra_op_threads > 8) {
        throw std::runtime_error(
            "runtime.onnx_intra_op_threads must be between 0 and 8");
    }
    spdlog::info("Navigation: ONNX intra-op threads {}",
                 onnx_intra_op_threads == 0
                     ? std::string("default (ONNX Runtime)")
                     : std::to_string(onnx_intra_op_threads));

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
    marker_rate_hz_ = viz["rate_hz"].as<double>(20.0);
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
    command_color_ = ConfigColor(viz, "command_rgba", command_color_);
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
    // A zero rate makes the period infinite and silently suppresses every
    // marker rather than failing.
    if (!(marker_rate_hz_ > 0.0)) {
        throw std::runtime_error("visualization.rate_hz must be positive");
    }

    low_env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile((policy_dir / "params/low_level_deploy.yaml").string()),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
            FSMState::lowstate));
    // The 1 kHz thread re-sends every 50 Hz target about twenty times, so an
    // age below two policy periods is healthy operation, not a stall.
    if (watchdog_enabled_ &&
        watchdog_warn_age_ < 2.0 * low_env_->step_dt) {
        throw std::runtime_error(
            "safety.joint_target_watchdog.warn_age must exceed two "
            "low-level policy periods");
    }
    low_env_->alg = std::make_unique<isaaclab::OrtRunner>(
        policy_dir / "exported/low_level_locomotion_policy.onnx",
        onnx_intra_op_threads);
    high_policy_ = std::make_unique<NavigationOrtRunner>(
        policy_dir / "exported/high_level_navigation_policy/gat_encoder.onnx",
        policy_dir / "exported/high_level_navigation_policy/policy_head.onnx",
        onnx_intra_op_threads);
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
            if (low_level_failed_.load()) return true;
            // run() has already published the damping command; the state
            // change only exists to stop the stalled policy thread and to
            // hand the robot to a state that keeps publishing.
            return watchdog_passive_on_fault_ &&
                   joint_target_stale_failed_.load();
        }, FSMStringMap.right.at("Passive")));
    registered_checks.emplace(registered_checks.begin() + 1,
        std::make_pair([this]() {
            if (policy_deadline_fault_streak_ <= 0) return false;
            return policy_deadline_failed_.load();
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

    StartMarkerThread();
}

State_Navigation::~State_Navigation() {
    exit();
    // The render thread publishes on marker_pub_, so it must be gone before
    // the executor is cancelled and the node is dropped. CtrlFSM::shutdown()
    // resets fsm_thread_ first, so this join is off the 1 kHz thread.
    StopMarkerThread();
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
    joint_target_stale_failed_.store(false);
    policy_deadline_failed_.store(false);
    policy_deadline_misses_.store(0);
    policy_deadline_streak_.store(0);
    policy_deadline_last_overrun_.store(0.0);
    policy_deadline_worst_overrun_.store(
        -std::numeric_limits<double>::infinity());
    watchdog_fault_logged_ = false;
    watchdog_fault_age_seen_ = 0.0;
    applied_gain_blend_ = 0.0f;
    last_run_ = {};
    {
        // joint_targets_ still holds the previous entry's pose. Arming the
        // stamp here makes the same ladder bound the wait for the first
        // target of this entry.
        std::lock_guard<std::mutex> lock(joint_target_mutex_);
        joint_targets_produced_ = false;
        joint_targets_stamp_ = SteadyClock::now();
    }
    tilt_started_ = {};
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_ = {};
        odometry_recovery_pending_ = false;
        odometry_recovery_samples_ = 0;
        velocity_command_ = {0.0f, 0.0f, 0.0f};
        previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
    }
    low_env_->set_external_velocity_command({0.0f, 0.0f, 0.0f});
    PublishCommand({0.0f, 0.0f, 0.0f});
    low_env_->reset();
    // No high-level result carries across an entry: the command must read as
    // stale until this entry's own first successful inference.
    last_high_success_ns_.store(0);
    accept_goal_commands_.store(true);
    marker_frames_accepted_.store(true);
    // The low-level loop starts first. run() re-sends joint_targets_ every
    // millisecond and enter() does not reset them, so nothing may delay the
    // first step() that produces this entry's targets; the first high-level
    // tick cannot command anything anyway, because enter() just cleared the
    // goal.
    policy_thread_running_.store(true);
    policy_thread_ = std::thread(&State_Navigation::PolicyLoop, this);
    high_level_thread_running_.store(true);
    high_level_thread_ = std::thread(&State_Navigation::HighLevelLoop, this);
}

void State_Navigation::exit() {
    accept_goal_commands_.store(false);
    // The Passive transition is evaluated on the same 1 kHz tick that latches
    // the fault, so this is the only place the event can still be reported.
    if (joint_target_stale_failed_.load() && !watchdog_fault_logged_) {
        watchdog_fault_logged_ = true;
        RCLCPP_ERROR(node_->get_logger(),
                     "Navigation low-level target stale %.3f s: "
                     "stiffness released, %s",
                     watchdog_fault_age_seen_,
                     watchdog_passive_on_fault_
                         ? "requesting Passive"
                         : "holding damping in Navigation");
    }
    // One store, not a join: exit() runs on the 1 kHz FSM thread and is the
    // path to Passive damping, so it must never wait on a marker build.
    marker_frames_accepted_.store(false);
    // Both loops are told to stop before either is joined so their stop
    // latencies overlap instead of adding. This runs on the 1 kHz FSM thread
    // for an FSM transition, and while it blocks, post_run() does not publish
    // any motor command at all.
    high_level_thread_running_.store(false);
    policy_thread_running_.store(false);
    if (high_level_thread_.joinable()) high_level_thread_.join();
    if (policy_thread_.joinable()) policy_thread_.join();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_ = {};
        odometry_recovery_pending_ = false;
        odometry_recovery_samples_ = 0;
        previous_normalized_action_ = {0.0f, 0.0f, 0.0f};
    }
    SetZeroCommand();
}

void State_Navigation::run() {
    std::vector<float> targets;
    SteadyClock::time_point stamp;
    bool produced;
    {
        std::lock_guard<std::mutex> lock(joint_target_mutex_);
        targets = joint_targets_;
        stamp = joint_targets_stamp_;
        produced = joint_targets_produced_;
    }
    const auto now = SteadyClock::now();
    // Slew rates below are per second of wall time, not per tick, so an FSM
    // thread that was preempted cannot turn one late tick into a gain step.
    // The clamp bounds the first tick of an entry and any long preemption.
    const double tick_dt = std::clamp(
        std::chrono::duration<double>(now - last_run_).count(), 0.0, 0.02);
    last_run_ = now;
    const double age = std::chrono::duration<double>(now - stamp).count();
    // The first target of an entry waits on three cold ONNX sessions, so the
    // whole ladder is shifted until one exists. Shifting it, rather than
    // giving the startup case its own hard limit, keeps the stiffness ramp in
    // front of the fault on both paths.
    const double shift =
        produced ? 0.0 : watchdog_startup_age_ - watchdog_fade_age_;

    float blend = 0.0f;
    if (!watchdog_enabled_) {
        blend = 0.0f;
    } else if (joint_target_stale_failed_.load()) {
        // Terminal for this entry. A policy that returns after this long is
        // looking at a state far outside its training distribution, and
        // restoring stiffness onto the errors the fade has already allowed
        // would be the largest torque step of the whole event.
        blend = 1.0f;
    } else if (age > watchdog_fade_age_ + shift) {
        blend = std::clamp(static_cast<float>(
            (age - watchdog_fade_age_ - shift) /
            (watchdog_fault_age_ - watchdog_fade_age_)), 0.0f, 1.0f);
    }
    ApplyGainBlend(blend, tick_dt);

    if (blend >= 1.0f && applied_gain_blend_ >= 1.0f) {
        // exit() joins the policy thread that is stalled and post_run() is the
        // only thing that reaches the wire, so the safe command has to be
        // written on this tick rather than after the transition. Only now is
        // the stiffness actually at the floor, so replacing the frozen target
        // with the measured posture cannot step the torque.
        if (watchdog_fade_stiffness_scale_ <= 0.0f) HoldMeasuredPosture();
        const bool first = !joint_target_stale_failed_.exchange(true);
        // With passive_on_fault the FSM evaluates the transition on this same
        // tick, so run() is never called again and only exit() can report the
        // event. Keep the age for it.
        if (first) watchdog_fault_age_seen_ = age;
        // Deferred one tick on purpose: the default logger flushes every
        // record to its rotating file sink, and post_run() cannot publish
        // while run() is blocked inside it. Only reachable with
        // passive_on_fault false, where run() keeps ticking.
        if (!first && !watchdog_fault_logged_) {
            watchdog_fault_logged_ = true;
            RCLCPP_ERROR(node_->get_logger(),
                         "Navigation low-level target stale %.3f s: "
                         "stiffness released, %s",
                         age,
                         watchdog_passive_on_fault_
                             ? "requesting Passive"
                             : "holding damping in Navigation");
        }
        return;
    }
    // Before the first target of this entry, leave the previous state's
    // command in place: joint_targets_ is either the 29 zeros the constructor
    // wrote or the pose the last entry froze at, and both are a step away
    // from whatever the previous state was holding.
    // Above the !produced return: the startup rung is the one an operator is
    // most likely to hit, and it has no other diagnostic.
    if (watchdog_enabled_ && age > watchdog_warn_age_ + shift) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             produced
                                 ? "Navigation low-level target stale %.3f s: "
                                   "stiffness at %.0f%%"
                                 : "Navigation waiting for its first "
                                   "low-level target, %.3f s: "
                                   "stiffness at %.0f%%",
                             age, 100.0 * (1.0f - applied_gain_blend_));
    }
    if (!produced || targets.size() != 29 || !AllFinite(targets)) return;
    for (std::size_t i = 0; i < targets.size(); ++i) {
        lowcmd->msg_.motor_cmd()[low_env_->robot->data.joint_ids_map[i]].q() = targets[i];
    }
}

// blend 0 is the gain set enter() wrote, blend 1 the configured fade floor.
// The healthy 1 kHz path returns before the body, so a fresh target produces
// exactly the command it produced before this change; the float compare is
// safe because both sides come from the same variable. Both directions slew:
// the age-derived target can jump, and a kp step at 1 kHz against an existing
// position error is the one thing this whole ladder exists to avoid.
void State_Navigation::ApplyGainBlend(float target, double dt) {
    if (target == applied_gain_blend_) return;
    const bool fading = target > applied_gain_blend_;
    const double rate = fading
        ? 1.0 / (watchdog_fault_age_ - watchdog_fade_age_)
        : 1.0 / watchdog_restore_seconds_;
    const float step = static_cast<float>(rate * dt);
    applied_gain_blend_ = fading
        ? std::min(target, applied_gain_blend_ + step)
        : std::max(target, applied_gain_blend_ - step);
    const float kp_scale =
        1.0f + applied_gain_blend_ * (watchdog_fade_stiffness_scale_ - 1.0f);
    const float kd_scale =
        1.0f + applied_gain_blend_ * (watchdog_fade_damping_scale_ - 1.0f);
    // Nothing downstream of lowcmd rejects a non-finite gain, so refuse to
    // write one no matter how it was arrived at.
    if (!std::isfinite(kp_scale) || !std::isfinite(kd_scale)) return;
    const auto& stiffness = low_env_->robot->data.joint_stiffness;
    const auto& damping = low_env_->robot->data.joint_damping;
    for (int i = 0; i < static_cast<int>(stiffness.size()); ++i) {
        lowcmd->msg_.motor_cmd()[i].kp() = kp_scale * stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = kd_scale * damping[i];
    }
}

// With the stiffness already at zero this is the command State_Passive::run()
// publishes: only the damping term survives. mode() is already 1 because
// State_Passive's constructor set it on the shared lowcmd.
void State_Navigation::HoldMeasuredPosture() {
    std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
    for (int i = 0; i < static_cast<int>(
             low_env_->robot->data.joint_stiffness.size()); ++i) {
        const float measured = FSMState::lowstate->msg_.motor_state()[i].q();
        if (std::isfinite(measured)) {
            lowcmd->msg_.motor_cmd()[i].q() = measured;
        }
    }
}

void State_Navigation::OnOdometry(const nav_msgs::msg::Odometry& msg) {
    const auto now = SteadyClock::now();
    const double yaw = QuaternionYaw(msg.pose.pose.orientation);
    bool recovered = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double vx = msg.twist.twist.linear.x;
        double vy = msg.twist.twist.linear.y;
        double wz = msg.twist.twist.angular.z;
        double dt = 0.0;
        if (robot_.valid) {
            dt = std::chrono::duration<double>(now - robot_.received).count();
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

        if (odometry_recovery_pending_) {
            // The first message after a gap starts a new run. Subsequent
            // messages must remain inside the configured stale timeout.
            if (dt > 0.0 && dt <= odometry_timeout_) {
                ++odometry_recovery_samples_;
            } else {
                odometry_recovery_samples_ = 1;
            }
            if (odometry_recovery_samples_ >= 3) {
                odometry_recovery_pending_ = false;
                odometry_recovery_samples_ = 0;
                recovered = true;
            }
        }
        robot_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                  vx, vy, wz, now, true};
    }
    if (recovered) {
        RCLCPP_INFO(node_->get_logger(),
                    "Navigation odometry recovered: 3 consecutive valid "
                    "samples; waiting for a new goal");
    }
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

// The stamp is written inside the same data_mutex_ critical section as the
// command it vouches for, so a stamp that moved while this thread waited for
// the lock means a fresh command landed and must not be discarded.
void State_Navigation::SetZeroCommandIfStale(std::int64_t observed_ns) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_high_success_ns_.load() != observed_ns) return;
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
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!odometry_recovery_pending_) {
                odometry_recovery_pending_ = true;
                odometry_recovery_samples_ = 0;
            }
        }
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
            QueueMarkers(robot, goal, selected, {0.0f, 0.0f, 0.0f});
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
                QueueMarkers(robot, goal, selected, {0.0f, 0.0f, 0.0f});
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
        std::array<float, 3> command{0.0f, 0.0f, 0.0f};
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
            last_high_success_ns_.store(SteadyNanoseconds(SteadyClock::now()));
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "Navigation command: vx=%.3f vy=%.3f wz=%.3f (obstacles=%zu)",
                velocity_command_[0], velocity_command_[1], velocity_command_[2],
                selected.size());
            command = velocity_command_;
        }
        QueueMarkers(robot, goal, selected, command);
        return true;
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "Navigation inference stopped command: %s", error.what());
        SetZeroCommand();
        return false;
    }
}

State_Navigation::PolicyLoopHealth
State_Navigation::GetPolicyLoopHealth() const {
    PolicyLoopHealth health;
    health.missed_deadlines =
        policy_deadline_misses_.load(std::memory_order_relaxed);
    health.consecutive_misses =
        policy_deadline_streak_.load(std::memory_order_relaxed);
    health.last_overrun =
        policy_deadline_last_overrun_.load(std::memory_order_relaxed);
    health.worst_overrun =
        policy_deadline_worst_overrun_.load(std::memory_order_relaxed);
    return health;
}

// Called once per policy iteration with how late that iteration finished
// against its 50 Hz slot; overrun is negative when the iteration finished
// early. Scheduler wake-up jitter of a few hundred microseconds is normal on
// this non-preemptible kernel, so only lateness above the configured
// tolerance counts as a miss. Only the policy thread calls this, which is why
// the running maximum needs no compare-exchange.
void State_Navigation::RecordPolicyDeadline(double overrun) {
    policy_deadline_last_overrun_.store(overrun, std::memory_order_relaxed);
    if (overrun >
        policy_deadline_worst_overrun_.load(std::memory_order_relaxed)) {
        policy_deadline_worst_overrun_.store(
            overrun, std::memory_order_relaxed);
    }
    if (overrun <= policy_deadline_tolerance_) {
        policy_deadline_streak_.store(0, std::memory_order_relaxed);
        return;
    }
    const std::uint64_t total =
        policy_deadline_misses_.fetch_add(1, std::memory_order_relaxed) + 1;
    const int streak =
        policy_deadline_streak_.fetch_add(1, std::memory_order_relaxed) + 1;
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Navigation policy loop missed its %.0f ms deadline by %.1f ms "
        "(streak %d, total %llu, worst %.1f ms); the 1 kHz output is "
        "holding the previous joint target",
        1.0e3 * low_env_->step_dt, 1.0e3 * overrun, streak,
        static_cast<unsigned long long>(total),
        1.0e3 * policy_deadline_worst_overrun_.load(
            std::memory_order_relaxed));
    if (policy_deadline_stop_streak_ > 0 &&
        streak == policy_deadline_stop_streak_) {
        // The same latched stop a stale sensor produces: inference cannot
        // resume until the operator sends a new goal. The zero is a step and
        // not a ramp, and unlike the sensor paths it lands while the joint
        // targets are themselves late, which is why this tier ships off.
        RCLCPP_ERROR(node_->get_logger(),
                     "Navigation stopped and goal cleared: policy loop late "
                     "for %d consecutive steps; a new goal is required",
                     streak);
        ClearGoalCommandState();
        SetZeroCommand();
    }
    if (policy_deadline_fault_streak_ > 0 &&
        streak >= policy_deadline_fault_streak_) {
        policy_deadline_failed_.store(true);
    }
}

void State_Navigation::PolicyLoop() {
    using namespace std::chrono_literals;
    const auto low_period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(low_env_->step_dt));
    int tick = 0, invalid = 0;
    auto next = SteadyClock::now();
    while (policy_thread_running_.load()) {
        const auto started = SteadyClock::now();
        // The high-level tick has its own thread now, so this is the only
        // check left that notices one that has stalled or died.
        const auto seen = last_high_success_ns_.load();
        if (AgeSeconds(seen) > command_timeout_) SetZeroCommandIfStale(seen);
        try {
            low_env_->step();
            const auto targets = low_env_->action_manager->processed_actions();
            if (targets.size() != 29 || !AllFinite(targets)) {
                throw std::runtime_error("invalid low-level joint target");
            }
            const auto stamp = SteadyClock::now();
            {
                std::lock_guard<std::mutex> lock(joint_target_mutex_);
                joint_targets_ = targets;
                joint_targets_stamp_ = stamp;
                joint_targets_produced_ = true;
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
        next += low_period;
        const auto now = SteadyClock::now();
        const double overrun = std::chrono::duration<double>(now - next).count();
        if (overrun > 0.0) {
            // Drop the accumulated debt rather than repay it: each step also
            // advances the gait phase by a fixed increment, so running steps
            // back to back would feed the motors a time-compressed gait.
            // Clamping against this iteration's own start is what keeps the
            // drop from also costing an idle period -- rebasing on `now`
            // alone would turn a 1 ms overshoot into a whole extra 20 ms
            // slot and halve the achieved rate.
            next = std::max(now, started + low_period);
        }
        // The first steps after entry pay for reset(), the first Run on each
        // ONNX graph and the first DDS write, so they are late by
        // construction; counting them would warn on every entry and leave
        // worst_overrun useless as the number the thresholds are tuned from.
        if (tick > policy_deadline_warmup_steps_) RecordPolicyDeadline(overrun);
        std::this_thread::sleep_until(next);
    }
}

void State_Navigation::HighLevelLoop() {
    const auto high_period = std::chrono::duration<double>(high_level_dt_);
    const auto stop_poll =
        std::chrono::duration_cast<SteadyClock::duration>(kHighLevelStopPoll);
    auto next = SteadyClock::now();
    while (high_level_thread_running_.load()) {
        try {
            UpdateHighLevel();
        } catch (const std::exception& error) {
            // The low-level loop no longer runs behind this call, so a throw
            // here would otherwise reach std::terminate and take the 1 kHz
            // FSM with it. Stop now rather than after command_timeout_, which
            // is what the inference-failure path inside UpdateHighLevel()
            // already does.
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                  "Navigation high-level tick failed: %s",
                                  error.what());
            SetZeroCommand();
        }
        next += std::chrono::duration_cast<SteadyClock::duration>(high_period);
        const auto finished = SteadyClock::now();
        if (next < finished) {
            // Without this the loop would fire a burst of catch-up ticks once
            // the overrun clears, and each one integrates a full
            // high_level_dt_ of acceleration into the velocity command.
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "High-level tick overran %.3f s by %.3f s; resynchronising",
                high_level_dt_,
                std::chrono::duration<double>(finished - next).count());
            next = finished;
        }
        while (high_level_thread_running_.load()) {
            const auto now = SteadyClock::now();
            if (now >= next) break;
            const auto slice = now + stop_poll;
            std::this_thread::sleep_until(slice < next ? slice : next);
        }
    }
}

void State_Navigation::QueueMarkers(
    const RobotSnapshot& robot, const GoalSnapshot& goal,
    const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected,
    const std::array<float, 3>& command) {
    if (!visualization_enabled_ || !marker_frames_accepted_.load()) return;
    MarkerFrame frame;
    frame.robot = robot;
    frame.goal = goal;
    frame.command = command;
    frame.obstacle_count = std::min(selected.size(), kMaximumMarkerObstacles);
    for (std::size_t i = 0; i < frame.obstacle_count; ++i) {
        const auto& item = selected[i];
        frame.obstacles[i] = {item.los_angle, item.boundary_vertex_x,
                              item.boundary_curvature,
                              item.relative_velocity_world[0],
                              item.relative_velocity_world[1], item.rank};
    }
    // Visualization must never extend a high-level tick. If the render thread
    // still holds the slot, drop this frame rather than wait for it.
    std::unique_lock<std::mutex> lock(marker_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    marker_frame_ = frame;
    marker_frame_pending_ = true;
    lock.unlock();
    marker_cv_.notify_one();
}

void State_Navigation::StartMarkerThread() {
    if (!visualization_enabled_ || marker_thread_.joinable()) return;
    marker_thread_running_.store(true);
    marker_thread_ = std::thread(&State_Navigation::MarkerLoop, this);
}

void State_Navigation::StopMarkerThread() {
    if (!marker_thread_.joinable()) return;
    {
        // Setting the flag under the lock is what makes the wait predicates
        // race-free; a frame queued during shutdown is discarded.
        std::lock_guard<std::mutex> lock(marker_mutex_);
        marker_thread_running_.store(false);
        marker_frame_pending_ = false;
    }
    marker_cv_.notify_all();
    marker_thread_.join();
}

void State_Navigation::MarkerLoop() {
    const auto minimum_interval =
        std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(1.0 / marker_rate_hz_));
    auto last_publish = SteadyClock::now() - minimum_interval;
    while (true) {
        MarkerFrame frame;
        {
            std::unique_lock<std::mutex> lock(marker_mutex_);
            marker_cv_.wait(lock, [this] {
                return marker_frame_pending_ ||
                       !marker_thread_running_.load();
            });
            if (!marker_thread_running_.load()) return;
            // rate_hz is a ceiling, not a decimator: hold the slot until the
            // interval elapses so what is finally drawn is the newest state,
            // never an arbitrary older sample. The producer overwrites the
            // slot freely while this wait has the lock released.
            const auto earliest = last_publish + minimum_interval;
            if (SteadyClock::now() < earliest) {
                marker_cv_.wait_until(lock, earliest, [this] {
                    return !marker_thread_running_.load();
                });
                if (!marker_thread_running_.load()) return;
            }
            frame = marker_frame_;
            marker_frame_pending_ = false;
        }
        last_publish = SteadyClock::now();
        // A frame queued just before exit() must not be drawn after the stop.
        if (!marker_frames_accepted_.load()) continue;
        PublishMarkers(frame);
    }
}

// Runs on marker_thread_. Everything below allocates roughly a hundred
// markers and serializes them for DDS, which has no business inside a
// policy or inference thread's budget.
void State_Navigation::PublishMarkers(const MarkerFrame& frame) {
    if (!marker_pub_) return;
    const auto& robot = frame.robot;
    const auto& goal = frame.goal;
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
    for (std::size_t i = 0; i < frame.obstacle_count; ++i) {
        const auto& item = frame.obstacles[i];
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
        const double arrow_x = item.relative_velocity_x *
                               relative_velocity_arrow_seconds_;
        const double arrow_y = item.relative_velocity_y *
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
        // Sharing the header stamp keeps the ring and its 0.25 s lifetime on
        // one clock read; the pulse is a free-running function of absolute
        // time, so which thread samples it does not change its shape.
        const double phase = std::fmod(stamp.seconds(), pulse_period_) / pulse_period_;
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
    const auto& command = frame.command;
    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    auto linear = base_marker("navigation_command", visualization_msgs::msg::Marker::ARROW);
    linear.scale.x = 0.045; linear.scale.y = 0.09; linear.scale.z = 0.12;
    linear.color = Color(command_color_);
    linear.points.push_back(Point(robot.x, robot.y, 1.55));
    linear.points.push_back(Point(robot.x + c * command[0] - s * command[1],
                                  robot.y + s * command[0] + c * command[1], 1.55));
    array.markers.push_back(linear);
    if (std::abs(command[2]) > 1.0e-3f) {
        auto angular = base_marker("navigation_command", visualization_msgs::msg::Marker::ARROW);
        angular.scale.x = 0.045; angular.scale.y = 0.09; angular.scale.z = 0.12;
        angular.color = Color(command_color_);
        angular.points.push_back(Point(robot.x, robot.y, 1.55));
        angular.points.push_back(Point(robot.x, robot.y, 1.55 + 0.5 * command[2]));
        array.markers.push_back(angular);
    }
    marker_pub_->publish(array);
}
