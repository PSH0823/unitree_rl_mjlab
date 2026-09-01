#pragma once

#include "FSM/FSMState.h"

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dpcbf_ros_adapter/dpcbf_boundary.h"
#include "isaaclab/envs/manager_based_rl_env.h"

class NavigationOrtRunner;

class State_Navigation : public FSMState
{
public:
    State_Navigation(int state_mode, std::string state_string);
    ~State_Navigation();

    void enter() override;
    void run() override;
    void exit() override;

private:
    using SteadyClock = std::chrono::steady_clock;

    struct RobotSnapshot {
        double x = 0.0, y = 0.0, yaw = 0.0;
        double vx_world = 0.0, vy_world = 0.0, yaw_rate = 0.0;
        SteadyClock::time_point received{};
        bool valid = false;
    };
    struct GoalSnapshot {
        double x = 0.0, y = 0.0, yaw = 0.0;
        bool active = false;
        bool external = false;
    };

    void OnOdometry(const nav_msgs::msg::Odometry& message);
    void OnObstacles(const obstacle_detector::msg::Obstacles& message);
    void OnGoal(const geometry_msgs::msg::PoseStamped& message);
    void OnStop(const std_msgs::msg::Empty&);
    void PolicyLoop();
    bool UpdateHighLevel();
    void SetZeroCommand();
    void PublishCommand(const std::array<float, 3>& command);
    void CreateRandomGoal(const RobotSnapshot& robot);
    bool HasApproachingGoalHoldThreat(
        const RobotSnapshot& robot,
        const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected) const;
    std::vector<float> BuildNodeObservation(
        const RobotSnapshot& robot,
        const GoalSnapshot& goal,
        const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected) const;
    std::vector<float> BuildLocalState(
        const RobotSnapshot& robot,
        const GoalSnapshot& goal) const;
    void PublishMarkers(
        const RobotSnapshot& robot,
        const GoalSnapshot& goal,
        const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected);

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> low_env_;
    std::unique_ptr<NavigationOrtRunner> high_policy_;
    std::thread policy_thread_;
    std::atomic<bool> policy_thread_running_{false};
    std::atomic<bool> low_level_failed_{false};
    // Goals are commands, not persistent configuration. Accept them only
    // while Navigation is active so an old UI command cannot move the robot
    // immediately on the next state entry.
    std::atomic<bool> accept_goal_commands_{false};
    std::mutex joint_target_mutex_;
    std::vector<float> joint_targets_;

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_thread_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;

    mutable std::mutex data_mutex_;
    RobotSnapshot robot_;
    GoalSnapshot goal_;
    std::vector<dpcbf::ObstacleState> obstacles_;
    SteadyClock::time_point obstacles_received_{};
    bool obstacles_received_once_ = false;
    std::array<float, 3> velocity_command_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> previous_normalized_action_{0.0f, 0.0f, 0.0f};
    std::array<std::array<float, 2>, 3> action_range_{
        {{-2.0f, 4.0f}, {-2.0f, 2.0f}, {-1.0f, 1.0f}}};
    std::array<std::array<float, 2>, 3> velocity_command_range_{
        {{-1.0f, 2.0f}, {-1.0f, 1.0f}, {-1.0f, 1.0f}}};
    SteadyClock::time_point last_high_success_{};

    dpcbf_ros_adapter::BoundaryParams boundary_params_;
    double arena_center_x_ = 0.0, arena_center_y_ = 0.0;
    double arena_width_ = 10.0, arena_height_ = 10.0;
    double high_level_dt_ = 0.1, goal_radius_ = 0.3;
    double goal_heading_tolerance_ = 0.17453292519943295;
    double obstacle_timeout_ = 0.5, odometry_timeout_ = 0.2;
    double odom_velocity_filter_tau_ = 0.15;
    double command_timeout_ = 0.25;
    bool enable_random_goal_ = false;
    bool hold_goal_after_reaching_ = true;
    double goal_hold_obstacle_trigger_distance_ = 1.0;
    double goal_hold_min_closing_speed_ = 0.05;
    double random_goal_margin_ = 0.6;
    bool collision_stop_enabled_ = true;
    double collision_stop_distance_ = 0.15;
    int invalid_low_level_output_limit_ = 3;
    double max_tilt_angle_ = 1.0, tilt_duration_ = 0.1;
    bool tilt_enabled_ = true;
    SteadyClock::time_point tilt_started_{};
    std::mt19937 random_engine_;

    bool visualization_enabled_ = true;
    double relative_velocity_arrow_seconds_ = 1.0;
    double parabola_lateral_limit_ = 1.0;
    double parabola_backward_limit_ = 1.5;
    double pulse_min_ = 0.08, pulse_max_ = 0.12, pulse_period_ = 1.2;
    double center_pulse_min_scale_ = 0.85, center_pulse_max_scale_ = 1.15;
    double goal_heading_line_width_ = 0.025;
    std::array<float, 4> goal_fill_color_{0.58f, 0.88f, 0.68f, 0.22f};
    std::array<float, 4> goal_outline_color_{0.37f, 0.50f, 0.41f, 0.95f};
    std::array<float, 4> goal_pulse_color_{0.61f, 0.94f, 0.70f, 1.0f};
    std::array<float, 4> goal_center_color_{0.82f, 0.89f, 0.85f, 1.0f};
    double goal_cone_tip_height_ = 0.10, goal_cone_height_ = 0.34;
    double goal_cone_top_radius_ = 0.035, goal_sphere_center_height_ = 0.60;
    double goal_sphere_radius_ = 0.05;
    int goal_cone_slices_ = 12;
};

REGISTER_FSM(State_Navigation)
