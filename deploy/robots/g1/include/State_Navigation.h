#pragma once

#include "FSM/FSMState.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <limits>
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

    // Deadline health of the 50 Hz PolicyLoop. The loop drops missed slots
    // instead of catching up, so lateness surfaces here instead of as a
    // burst of back-to-back policy steps. last_overrun is negative while the
    // loop is on time, so worst_overrun is the true running maximum and is
    // the margin number to tune the thresholds from; it reads -inf until the
    // first iteration past the entry warm-up. Safe to call from any thread.
    struct PolicyLoopHealth {
        std::uint64_t missed_deadlines = 0;
        int consecutive_misses = 0;
        double last_overrun = 0.0;
        double worst_overrun = -std::numeric_limits<double>::infinity();
    };
    PolicyLoopHealth GetPolicyLoopHealth() const;

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
    // The DPCBF boundary is drawn from its line-of-sight coefficients, so
    // the render side never needs the obstacle state itself.
    struct MarkerObstacle {
        double los_angle = 0.0;
        double boundary_vertex_x = 0.0;
        double boundary_curvature = 0.0;
        double relative_velocity_x = 0.0;
        double relative_velocity_y = 0.0;
        int rank = 0;
    };
    // The constructor rejects max_constraints > 10, so a fixed array keeps
    // the producer's handoff free of allocation.
    static constexpr std::size_t kMaximumMarkerObstacles = 10;
    struct MarkerFrame {
        RobotSnapshot robot;
        GoalSnapshot goal;
        std::array<float, 3> command{0.0f, 0.0f, 0.0f};
        std::array<MarkerObstacle, kMaximumMarkerObstacles> obstacles{};
        std::size_t obstacle_count = 0;
    };

    void OnOdometry(const nav_msgs::msg::Odometry& message);
    void OnObstacles(const obstacle_detector::msg::Obstacles& message);
    void OnGoal(const geometry_msgs::msg::PoseStamped& message);
    void OnStop(const std_msgs::msg::Empty&);
    void ClearGoalCommandState();
    void PolicyLoop();
    void HighLevelLoop();
    void RecordPolicyDeadline(double overrun);
    bool UpdateHighLevel();
    void SetZeroCommand();
    void SetZeroCommandIfStale(std::int64_t observed_ns);
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
    void QueueMarkers(
        const RobotSnapshot& robot,
        const GoalSnapshot& goal,
        const std::vector<dpcbf_ros_adapter::BoundaryObstacle>& selected,
        const std::array<float, 3>& command);
    void StartMarkerThread();
    void StopMarkerThread();
    void MarkerLoop();
    void PublishMarkers(const MarkerFrame& frame);
    void ApplyGainBlend(float target, double dt);
    void HoldMeasuredPosture();

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> low_env_;
    std::unique_ptr<NavigationOrtRunner> high_policy_;
    std::thread policy_thread_;
    std::atomic<bool> policy_thread_running_{false};
    std::thread high_level_thread_;
    std::atomic<bool> high_level_thread_running_{false};
    std::atomic<bool> low_level_failed_{false};
    std::atomic<bool> joint_target_stale_failed_{false};
    std::atomic<bool> policy_deadline_failed_{false};
    // Written only by the policy thread and read by the FSM thread and by
    // GetPolicyLoopHealth(), so relaxed load/store pairs are sufficient and
    // the running maximum below needs no compare-exchange.
    std::atomic<std::uint64_t> policy_deadline_misses_{0};
    std::atomic<int> policy_deadline_streak_{0};
    std::atomic<double> policy_deadline_last_overrun_{0.0};
    std::atomic<double> policy_deadline_worst_overrun_{
        -std::numeric_limits<double>::infinity()};
    // Goals are commands, not persistent configuration. Accept them only
    // while Navigation is active so an old UI command cannot move the robot
    // immediately on the next state entry.
    std::atomic<bool> accept_goal_commands_{false};
    std::mutex joint_target_mutex_;
    std::vector<float> joint_targets_;
    // PolicyLoop stamps the target it stores; enter() re-arms the stamp and
    // clears the flag, so the 1 kHz thread can neither replay the previous
    // entry's pose nor mistake it for a fresh one.
    SteadyClock::time_point joint_targets_stamp_{};
    bool joint_targets_produced_ = false;

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
    // Recovery is informational: it never restores the cleared goal or
    // resumes high-level inference automatically.
    bool odometry_recovery_pending_ = false;
    int odometry_recovery_samples_ = 0;
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
    // Written by the high-level thread, read by the low-level loop every
    // step. An atomic keeps that read off data_mutex_, which the high-level
    // thread holds across a DDS publish.
    std::atomic<std::int64_t> last_high_success_ns_{0};

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
    double policy_deadline_tolerance_ = 0.002;
    int policy_deadline_warmup_steps_ = 5;
    int policy_deadline_stop_streak_ = 0;
    int policy_deadline_fault_streak_ = 0;
    // Default off: a config tree without the block keeps today's actuation
    // rather than silently acquiring a state transition it never had.
    bool watchdog_enabled_ = false;
    double watchdog_warn_age_ = 0.06;
    double watchdog_fade_age_ = 0.20;
    double watchdog_fault_age_ = 0.35;
    double watchdog_startup_age_ = 1.0;
    double watchdog_restore_seconds_ = 0.5;
    float watchdog_fade_stiffness_scale_ = 0.0f;
    float watchdog_fade_damping_scale_ = 1.0f;
    bool watchdog_passive_on_fault_ = true;
    // Touched only by the FSM thread (run() and enter()).
    float applied_gain_blend_ = 0.0f;
    bool watchdog_fault_logged_ = false;
    double watchdog_fault_age_seen_ = 0.0;
    SteadyClock::time_point last_run_{};
    double max_tilt_angle_ = 1.0, tilt_duration_ = 0.1;
    bool tilt_enabled_ = true;
    SteadyClock::time_point tilt_started_{};
    std::mt19937 random_engine_;

    bool visualization_enabled_ = true;
    double marker_rate_hz_ = 20.0;
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
    std::array<float, 4> command_color_{1.0f, 0.0f, 0.0f, 0.7f};
    double goal_cone_tip_height_ = 0.10, goal_cone_height_ = 0.34;
    double goal_cone_top_radius_ = 0.035, goal_sphere_center_height_ = 0.60;
    double goal_sphere_radius_ = 0.05;
    int goal_cone_slices_ = 12;

    // Single-slot latest-frame handoff to a render thread that lives for the
    // object, not for the state: no FSM transition may ever wait on RViz.
    // The producer only try_lock()s marker_mutex_ and never blocks.
    std::mutex marker_mutex_;
    std::condition_variable marker_cv_;
    MarkerFrame marker_frame_;
    bool marker_frame_pending_ = false;
    std::atomic<bool> marker_thread_running_{false};
    // Whether Navigation is the active state. Separate from the thread's own
    // lifetime so entering and leaving the state costs one atomic store.
    std::atomic<bool> marker_frames_accepted_{false};
    // Declared last so it is destroyed first, before the mutex and condition
    // variable it waits on.
    std::thread marker_thread_;
};

REGISTER_FSM(State_Navigation)
