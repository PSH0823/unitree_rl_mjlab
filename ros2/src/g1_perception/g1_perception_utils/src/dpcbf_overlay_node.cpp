// dpcbf_overlay: live 2-D overlay of estimated-vs-ground-truth obstacles and
// the DPCBF constraint geometry, as one MarkerArray (§14.4, runbook §4.6).
//
// WHY THERE ARE TWO FRAMES. The DPCBF barrier's h=0 boundary is a parabola in
// the per-obstacle line-of-sight RELATIVE VELOCITY plane (see
// dpcbf_ros_adapter/dpcbf_boundary.h and dpcbf_safety_filter.h:47), not an
// envelope in the world. Drawing it in `odom` would put an m/s curve on a
// metre grid -- a confidently wrong picture. So:
//
//   odom                    the obstacle layers, the p_max horizon and the
//                           eCBF distance-barrier circles. All honest metres.
//   dpcbf_velocity_plane    a visualization-only TF frame parked beside the
//                           robot, holding the velocity-space cards. Axes are
//                           m/s, scaled by `vel_scale` metres per (m/s), and
//                           every card says so.
//
// The node is READ-ONLY with respect to the running system: it subscribes and
// publishes markers (plus one visualization TF and an optional JSONL log). It
// republishes nothing the stack consumes.
//
// GT is sim-only (D4): on hardware and on bag replays without
// /sim/gt_obstacles the GT-derived layers are dropped and a banner says so,
// because an overlay that silently drops its reference layer looks like
// perfect agreement.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dpcbf_ros_adapter/dpcbf_boundary.h"
#include "dpcbf_ros_adapter/dpcbf_boundary_config.h"

namespace dra = dpcbf_ros_adapter;
using Marker = visualization_msgs::msg::Marker;

namespace {

struct Circle {
  double x = 0.0;
  double y = 0.0;
  double r = 0.0;      // radius as published (tracked/safe carry the margin)
  double vx = 0.0;
  double vy = 0.0;
  std::uint64_t uid = 0;
};

std::vector<Circle> ToCircles(const obstacle_detector::msg::Obstacles& m) {
  std::vector<Circle> out;
  out.reserve(m.circles.size());
  for (const auto& c : m.circles) {
    out.push_back({c.center.x, c.center.y,
                   c.true_radius > 0.0 ? c.true_radius : c.radius,
                   c.velocity.x, c.velocity.y, c.uid});
  }
  return out;
}

double Sec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

geometry_msgs::msg::Point Pt(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::msg::ColorRGBA Col(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

}  // namespace

class DpcbfOverlay : public rclcpp::Node {
 public:
  DpcbfOverlay() : Node("dpcbf_overlay") {
    const auto config_path =
        declare_parameter<std::string>("dpcbf_config", "");
    frame_ = declare_parameter<std::string>("frame_id", "odom");
    vel_frame_ = declare_parameter<std::string>("velocity_frame_id",
                                                "dpcbf_velocity_plane");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    gt_timeout_ = declare_parameter<double>("gt_timeout_s", 2.0);
    nn_gate_ = declare_parameter<double>("nn_gate_m", 0.5);
    max_cards_ = declare_parameter<int>("max_velocity_cards", 3);
    vel_scale_ = declare_parameter<double>("vel_scale", 0.35);
    vel_axis_limit_ = declare_parameter<double>("vel_axis_limit", 3.0);
    panel_offset_ = declare_parameter<double>("panel_offset_m", 4.6);
    error_magnify_ = declare_parameter<double>("error_magnify", 1.0);
    // Pose-difference smoothing. Swept offline against the 1 kHz capture's
    // exact body velocity over a walking window (runbook §4.6): mean |dv|
    // 0.032 m/s at tau=0, 0.032 at 0.02, 0.049 at 0.05, 0.077 at 0.15 --
    // i.e. smoothing COSTS accuracy here, because /odom pose is exact and
    // the lag is the dominant error, not sample noise. 0.02 s keeps tau=0's
    // mean while trimming its p95 (0.055 vs 0.094 m/s).
    vel_tau_ = declare_parameter<double>("vel_tau_s", 0.02);
    log_path_ = declare_parameter<std::string>("log_path", "");

    if (config_path.empty()) {
      throw std::runtime_error(
          "dpcbf_overlay requires the 'dpcbf_config' parameter (path to the "
          "dpcbf_config.yaml the simulator loaded) -- re-deriving the "
          "constraint geometry from ROS defaults would let the overlay drift "
          "from the filter silently");
    }
    params_ = dra::LoadBoundaryParams(config_path);
    RCLCPP_INFO(get_logger(),
                "boundary params from %s: r_rob=%.3f p_max=%.2f s=%.3f "
                "k_mu=%.3f k_lambda=%.3f priority=%d n_max=%d ecbf=%d",
                config_path.c_str(), params_.robot_radius,
                params_.detection_radius, params_.safety_factor, params_.k_mu,
                params_.k_lambda, params_.obstacle_priority,
                params_.max_constraints, static_cast<int>(params_.ecbf_enabled));
    if (error_magnify_ != 1.0) {
      RCLCPP_WARN(get_logger(),
                  "error_magnify=%.2f -- position errors are FALSIFIED for "
                  "legibility; the factor is drawn on screen",
                  error_magnify_);
    }

    if (!log_path_.empty()) {
      log_ = std::fopen(log_path_.c_str(), "w");
      if (!log_) {
        RCLCPP_ERROR(get_logger(), "cannot open log_path %s",
                     log_path_.c_str());
      }
    }

    const rclcpp::QoS qos(5);
    sub_gt_ = create_subscription<obstacle_detector::msg::Obstacles>(
        "/sim/gt_obstacles", qos,
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
          gt_ = ToCircles(*m);
          gt_stamp_ = Sec(m->header.stamp);
        });
    sub_tracked_ = create_subscription<obstacle_detector::msg::Obstacles>(
        "/tracked_obstacles", qos,
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
          tracked_ = ToCircles(*m);
        });
    sub_safe_ = create_subscription<obstacle_detector::msg::Obstacles>(
        "/obstacles_safe", qos,
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
          safe_ = ToCircles(*m);
          safe_stamp_ = Sec(m->header.stamp);
        });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { OnOdom(*m); });
    sub_status_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/dpcbf/status", qos,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          for (const auto& s : m->status) {
            if (s.name != "dpcbf_ros_adapter") continue;
            status_level_ = s.level;
            status_state_ = s.message;
            for (const auto& kv : s.values) {
              if (kv.key == "mode") status_mode_ = kv.value;
              if (kv.key == "age_s") status_age_ = kv.value;
            }
            status_stamp_ = now().seconds();
          }
        });

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/dpcbf_overlay/markers", rclcpp::QoS(1));
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_), [this] { Tick(); });
  }

  ~DpcbfOverlay() override {
    if (log_) std::fclose(log_);
  }

 private:
  // ---- inputs ------------------------------------------------------------

  // /odom carries ground-truth POSE only -- sim_mjlidar_bridge does not
  // populate twist (MjState has no qvel by design, bridge_node.py:9). The
  // DPCBF barrier needs body-frame (sagittal, lateral) velocity, so it is
  // finite-differenced from the 100 Hz pose here and rotated into the body
  // frame. That is the one input the overlay ESTIMATES rather than reads;
  // boundary_check's `join` mode prices the resulting error against the
  // 1 kHz capture, and the banner says the velocity is differentiated.
  void OnOdom(const nav_msgs::msg::Odometry& m) {
    const double t = Sec(m.header.stamp);
    const auto& q = m.pose.pose.orientation;
    const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double x = m.pose.pose.position.x;
    const double y = m.pose.pose.position.y;

    if (have_prev_ && t > prev_t_) {
      const double dt = t - prev_t_;
      // One-pole smoothing over ~`vel_tau_` of pose differences: raw 100 Hz
      // differences of a walking pelvis are dominated by per-step bob.
      const double vx = (x - prev_x_) / dt;
      const double vy = (y - prev_y_) / dt;
      const double a = std::min(1.0, dt / vel_tau_);
      world_vx_ += a * (vx - world_vx_);
      world_vy_ += a * (vy - world_vy_);
      vel_valid_ = true;
    }
    prev_t_ = t;
    prev_x_ = x;
    prev_y_ = y;
    have_prev_ = true;

    robot_.x = x;
    robot_.y = y;
    robot_.phi = yaw;
    robot_.sagittal_velocity =
        world_vx_ * std::cos(yaw) + world_vy_ * std::sin(yaw);
    robot_.lateral_velocity =
        -world_vx_ * std::sin(yaw) + world_vy_ * std::cos(yaw);
    odom_stamp_ = t;
    have_odom_ = true;
  }

  // ---- rendering ---------------------------------------------------------

  void Tick() {
    if (!have_odom_) return;
    const rclcpp::Time stamp = now();
    const double t_now = stamp.seconds();
    const bool gt_live =
        !gt_.empty() && (t_now - gt_stamp_) < gt_timeout_ && gt_stamp_ > 0.0;

    // The obstacle set the filter is actually given is /obstacles_safe --
    // that is the seam's input, so the recomputation must use it and not the
    // tracked stream one stage upstream.
    std::vector<dpcbf::ObstacleState> filter_input;
    filter_input.reserve(safe_.size());
    for (const auto& c : safe_) {
      filter_input.push_back({c.x, c.y, c.r, c.vx, c.vy,
                              static_cast<int>(c.uid & 0x7fffffffu)});
    }
    const auto selected = dra::SelectAndEvaluate(params_, robot_, filter_input);

    visualization_msgs::msg::MarkerArray out;
    Marker clear;
    clear.header.frame_id = frame_;
    clear.header.stamp = stamp;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);

    DrawHorizon(out, stamp);
    if (gt_live) DrawRings(out, stamp, gt_, "gt", 0.02, Col(0.15, 0.75, 0.2, 0.9), 0.02);
    DrawDiscs(out, stamp, tracked_, "tracked", 0.04, Col(1.0, 0.55, 0.1, 0.35));
    DrawRings(out, stamp, safe_, "safe", 0.06, Col(0.9, 0.15, 0.15, 0.8), 0.02);
    if (gt_live) DrawPairing(out, stamp);
    DrawEcbf(out, stamp, selected);
    DrawSelection(out, stamp, selected);
    DrawVelocityPlane(out, stamp, selected);
    DrawBanner(out, stamp, gt_live, selected.size());

    pub_->publish(out);
    LogTick(t_now, selected);
  }

  void DrawHorizon(visualization_msgs::msg::MarkerArray& out,
                   const rclcpp::Time& stamp) const {
    Marker m = Base(stamp, "pmax", 0, Marker::LINE_STRIP, frame_);
    m.scale.x = 0.02;
    m.color = Col(0.35, 0.75, 0.35, 0.65);
    for (int i = 0; i <= 72; ++i) {
      const double a = 2.0 * M_PI * i / 72.0;
      m.points.push_back(Pt(robot_.x + params_.detection_radius * std::cos(a),
                            robot_.y + params_.detection_radius * std::sin(a),
                            0.01));
    }
    out.markers.push_back(m);

    Marker r = Base(stamp, "pmax", 1, Marker::LINE_STRIP, frame_);
    r.scale.x = 0.02;
    r.color = Col(0.35, 0.75, 0.35, 0.9);
    for (int i = 0; i <= 36; ++i) {
      const double a = 2.0 * M_PI * i / 36.0;
      r.points.push_back(Pt(robot_.x + params_.robot_radius * std::cos(a),
                            robot_.y + params_.robot_radius * std::sin(a),
                            0.01));
    }
    out.markers.push_back(r);

    Marker h = Base(stamp, "pmax", 2, Marker::ARROW, frame_);
    h.scale.x = 0.04;
    h.scale.y = 0.09;
    h.color = Col(0.35, 0.75, 0.35, 0.95);
    h.points.push_back(Pt(robot_.x, robot_.y, 0.01));
    h.points.push_back(Pt(robot_.x + 0.6 * std::cos(robot_.phi),
                          robot_.y + 0.6 * std::sin(robot_.phi), 0.01));
    out.markers.push_back(h);
  }

  // Unfilled ring: the reference layer must never occlude what is measured.
  void DrawRings(visualization_msgs::msg::MarkerArray& out,
                 const rclcpp::Time& stamp, const std::vector<Circle>& cs,
                 const std::string& ns, double z,
                 const std_msgs::msg::ColorRGBA& col, double width) const {
    int id = 0;
    for (const auto& c : cs) {
      if (!InScope(c)) continue;
      Marker m = Base(stamp, ns, id++, Marker::LINE_STRIP, frame_);
      m.scale.x = width;
      m.color = col;
      for (int i = 0; i <= 36; ++i) {
        const double a = 2.0 * M_PI * i / 36.0;
        m.points.push_back(
            Pt(c.x + c.r * std::cos(a), c.y + c.r * std::sin(a), z));
      }
      out.markers.push_back(m);
    }
  }

  // Translucent filled disc: reads as "the estimate covers the truth", and
  // the offset stays visible through it.
  void DrawDiscs(visualization_msgs::msg::MarkerArray& out,
                 const rclcpp::Time& stamp, const std::vector<Circle>& cs,
                 const std::string& ns, double z,
                 std_msgs::msg::ColorRGBA col) const {
    if (Stale()) {  // staleness must be visible, not inferred
      col.r = 0.55f;
      col.g = 0.2f;
      col.b = 0.75f;
    }
    int id = 0;
    for (const auto& c : cs) {
      if (!InScope(c)) continue;
      Marker m = Base(stamp, ns, id++, Marker::CYLINDER, frame_);
      m.pose.position = Pt(c.x, c.y, z);
      m.scale.x = 2.0 * c.r;
      m.scale.y = 2.0 * c.r;
      m.scale.z = 0.01;
      m.color = col;
      out.markers.push_back(m);
    }
  }

  // Association is walk_ab_probe.py's rule verbatim so the two views cannot
  // disagree about the same run: scope by p_max, tracked -> nearest GT, gate
  // at nn_gate_ (the Phase-4 0.5 m NN cap). Non-exclusive, deliberately --
  // making it exclusive here would change the reported mm numbers.
  void DrawPairing(visualization_msgs::msg::MarkerArray& out,
                   const rclcpp::Time& stamp) {
    int seg = 0, txt = 0, miss = 0, fp = 0;
    Marker lines = Base(stamp, "error", 0, Marker::LINE_LIST, frame_);
    lines.scale.x = 0.015;
    lines.color = Col(0.1, 0.1, 0.1, 0.9);

    for (const auto& t : tracked_) {
      if (!InScope(t)) continue;
      const Circle* best = nullptr;
      double bd = std::numeric_limits<double>::infinity();
      for (const auto& g : gt_) {
        const double d = std::hypot(t.x - g.x, t.y - g.y);
        if (d < bd) {
          bd = d;
          best = &g;
        }
      }
      if (!best || bd > nn_gate_) {
        Marker m = Base(stamp, "unpaired_tracked", fp++, Marker::LINE_STRIP,
                        frame_);
        m.scale.x = 0.035;
        m.color = Col(0.95, 0.1, 0.85, 0.95);  // magenta = false positive
        for (int i = 0; i <= 36; ++i) {
          const double a = 2.0 * M_PI * i / 36.0;
          m.points.push_back(Pt(t.x + (t.r + 0.06) * std::cos(a),
                                t.y + (t.r + 0.06) * std::sin(a), 0.07));
        }
        out.markers.push_back(m);
        out.markers.push_back(
            Text(stamp, "unpaired_tracked", 1000 + fp, t.x, t.y + t.r + 0.18,
                 0.12, "FP?", Col(0.95, 0.1, 0.85, 1.0)));
        continue;
      }
      // True geometry, never magnified unless explicitly asked; the mm text
      // is the number to quote.
      const double ex = best->x + error_magnify_ * (t.x - best->x);
      const double ey = best->y + error_magnify_ * (t.y - best->y);
      lines.points.push_back(Pt(best->x, best->y, 0.08));
      lines.points.push_back(Pt(ex, ey, 0.08));
      ++seg;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.0f mm", 1000.0 * bd);
      out.markers.push_back(Text(stamp, "error", 500 + txt++,
                                 0.5 * (best->x + ex), 0.5 * (best->y + ey),
                                 0.10, buf, Col(0.05, 0.05, 0.05, 1.0)));
    }
    if (seg > 0) out.markers.push_back(lines);

    for (const auto& g : gt_) {
      if (!InScope(g)) continue;
      double bd = std::numeric_limits<double>::infinity();
      for (const auto& t : tracked_) {
        bd = std::min(bd, std::hypot(t.x - g.x, t.y - g.y));
      }
      if (bd <= nn_gate_) continue;
      Marker m = Base(stamp, "unpaired_gt", miss++, Marker::LINE_STRIP, frame_);
      m.scale.x = 0.035;
      m.color = Col(0.1, 0.9, 0.95, 0.95);  // cyan = missed detection
      for (int i = 0; i <= 36; ++i) {
        const double a = 2.0 * M_PI * i / 36.0;
        m.points.push_back(Pt(g.x + (g.r + 0.06) * std::cos(a),
                              g.y + (g.r + 0.06) * std::sin(a), 0.03));
      }
      out.markers.push_back(m);
      out.markers.push_back(
          Text(stamp, "unpaired_gt", 2000 + miss, g.x, g.y + g.r + 0.18, 0.12,
               "MISS", Col(0.1, 0.9, 0.95, 1.0)));
    }
  }

  // The eCBF family's zero set IS world geometry: |p| = r_rob + r_obs. The
  // shipped config runs ecbf_enabled with slack, so this is half of what the
  // QP actually solves and the only half that belongs on a metre grid.
  void DrawEcbf(visualization_msgs::msg::MarkerArray& out,
                const rclcpp::Time& stamp,
                const std::vector<dra::BoundaryObstacle>& sel) const {
    if (!params_.ecbf_enabled) return;
    int id = 0;
    for (const auto& b : sel) {
      Marker m = Base(stamp, "ecbf_barrier", id++, Marker::LINE_STRIP, frame_);
      m.scale.x = 0.02;
      m.color = b.ecbf_h <= 0.0 ? Col(1.0, 0.0, 0.0, 1.0)
                                : Col(0.85, 0.4, 0.0, 0.75);
      for (int i = 0; i <= 48; ++i) {
        const double a = 2.0 * M_PI * i / 48.0;
        m.points.push_back(Pt(b.obstacle.x + b.ecbf_radius * std::cos(a),
                              b.obstacle.y + b.ecbf_radius * std::sin(a),
                              0.09));
      }
      out.markers.push_back(m);
    }
    if (!sel.empty()) {
      out.markers.push_back(
          Text(stamp, "ecbf_barrier", 900, robot_.x, robot_.y - 0.5, 0.11,
               params_.slack_enabled
                   ? "eCBF |p|=r_rob+r_obs (soft: slack on)"
                   : "eCBF |p|=r_rob+r_obs",
               Col(0.85, 0.4, 0.0, 0.9)));
    }
  }

  // Which obstacles the filter would actually constrain. Drawing envelopes
  // around obstacles the filter ignored is the second way an overlay lies,
  // so the selected set is marked explicitly, in QP row order.
  void DrawSelection(visualization_msgs::msg::MarkerArray& out,
                     const rclcpp::Time& stamp,
                     const std::vector<dra::BoundaryObstacle>& sel) const {
    int id = 0;
    for (const auto& b : sel) {
      Marker m = Base(stamp, "selected", id++, Marker::LINE_STRIP, frame_);
      m.scale.x = 0.03;
      m.color = Col(0.2, 0.35, 0.95, 0.95);
      for (int i = 0; i <= 48; ++i) {
        const double a = 2.0 * M_PI * i / 48.0;
        m.points.push_back(Pt(b.obstacle.x + b.safe_radius * std::cos(a),
                              b.obstacle.y + b.safe_radius * std::sin(a),
                              0.10));
      }
      out.markers.push_back(m);

      char buf[64];
      std::snprintf(buf, sizeof(buf), "#%d h=%.2f", b.rank + 1, b.h);
      out.markers.push_back(Text(stamp, "selected", 300 + b.rank,
                                 b.obstacle.x, b.obstacle.y - b.safe_radius -
                                 0.16, 0.12, buf, Col(0.2, 0.35, 0.95, 1.0)));

      Marker line = Base(stamp, "selected", 600 + b.rank, Marker::LINE_LIST,
                         frame_);
      line.scale.x = 0.01;
      line.color = Col(0.2, 0.35, 0.95, 0.45);
      line.points.push_back(Pt(robot_.x, robot_.y, 0.10));
      line.points.push_back(Pt(b.obstacle.x, b.obstacle.y, 0.10));
      out.markers.push_back(line);
    }
  }

  // The velocity-space cards, in their own frame. Everything here is m/s.
  void DrawVelocityPlane(visualization_msgs::msg::MarkerArray& out,
                         const rclcpp::Time& stamp,
                         const std::vector<dra::BoundaryObstacle>& sel) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = frame_;
    tf.child_frame_id = vel_frame_;
    tf.transform.translation.x = robot_.x;
    tf.transform.translation.y = robot_.y + panel_offset_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_->sendTransform(tf);

    const double span = 2.0 * vel_axis_limit_ * vel_scale_;  // card width [m]
    const double pitch = span * 1.25;
    const int shown = std::min<int>(static_cast<int>(sel.size()),
                                    std::max(0, max_cards_));

    // INDICATIVE, and said so on screen rather than in a doc nobody has open.
    // `boundary_check math` proves this geometry is bit-identical to the
    // frozen filter's ON THE SAME INPUTS; what it cannot fix is that one
    // input is estimated. The seam gets the 1 kHz instantaneous pelvis twist;
    // the overlay must differentiate /odom pose at 100 Hz because no twist is
    // published, and the walking pelvis carries content that sampling cannot
    // represent. Measured over a walking window against the 1 kHz capture:
    // mean |dv| 0.032 m/s (p95 0.055) against a mean body speed of 0.42 m/s,
    // but rare footfall transients reach 20 m/s and the selected SET matches
    // exactly on only ~55 % of ticks, because ordering by closing alignment
    // reorders on tiny velocity differences. See runbook §4.6.
    std::string title = "DPCBF h=0 -- LoS RELATIVE-VELOCITY plane\n"
                        "axes are m/s, NOT metres\n"
                        "INDICATIVE: v_robot is d/dt odom pose\n"
                        "(mean 0.03 m/s vs the 1 kHz seam)";
    if (shown < static_cast<int>(sel.size())) {
      title += "\nshowing " + std::to_string(shown) + " of " +
               std::to_string(sel.size()) + " selected";
    }
    out.markers.push_back(Text(stamp, "vel_title", 0, 0.0, 0.72 * pitch, 0.18,
                               title, Col(0.1, 0.1, 0.1, 1.0), vel_frame_));

    if (sel.empty()) {
      out.markers.push_back(Text(stamp, "vel_title", 1, 0.0, 0.0, 0.20,
                                 "no obstacle inside p_max -- QP unconstrained",
                                 Col(0.45, 0.45, 0.45, 1.0), vel_frame_));
      return;
    }

    for (int k = 0; k < shown; ++k) {
      const auto& b = sel[static_cast<std::size_t>(k)];
      const double cx = (k - 0.5 * (shown - 1)) * pitch;
      DrawVelocityCard(out, stamp, b, k, cx, span);
    }
  }

  void DrawVelocityCard(visualization_msgs::msg::MarkerArray& out,
                        const rclcpp::Time& stamp,
                        const dra::BoundaryObstacle& b, int k, double cx,
                        double span) const {
    const double half = 0.5 * span;
    const auto V = [&](double x, double y) {  // (m/s, m/s) -> card metres
      return Pt(cx + std::clamp(x, -vel_axis_limit_, vel_axis_limit_) *
                         vel_scale_,
                std::clamp(y, -vel_axis_limit_, vel_axis_limit_) * vel_scale_,
                0.0);
    };

    Marker box = Base(stamp, "vel_card", k, Marker::LINE_STRIP, vel_frame_);
    box.scale.x = 0.02;
    box.color = Col(0.35, 0.35, 0.35, 0.9);
    box.points = {Pt(cx - half, -half, 0.0), Pt(cx + half, -half, 0.0),
                  Pt(cx + half, half, 0.0),  Pt(cx - half, half, 0.0),
                  Pt(cx - half, -half, 0.0)};
    out.markers.push_back(box);

    Marker axes = Base(stamp, "vel_axes", k, Marker::LINE_LIST, vel_frame_);
    axes.scale.x = 0.012;
    axes.color = Col(0.55, 0.55, 0.55, 0.9);
    axes.points = {Pt(cx - half, 0.0, 0.0), Pt(cx + half, 0.0, 0.0),
                   Pt(cx, -half, 0.0),      Pt(cx, half, 0.0)};
    out.markers.push_back(axes);

    // h = 0: x~ = vertex_x - curvature * y~^2. Unsafe side is x~ < that.
    Marker curve = Base(stamp, "vel_boundary", k, Marker::LINE_STRIP,
                        vel_frame_);
    curve.scale.x = 0.03;
    curve.color = b.h < 0.0 ? Col(1.0, 0.1, 0.1, 1.0) : Col(0.2, 0.35, 0.95, 1.0);
    Marker fill = Base(stamp, "vel_unsafe", k, Marker::TRIANGLE_LIST,
                       vel_frame_);
    fill.scale.x = fill.scale.y = fill.scale.z = 1.0;
    fill.color = Col(0.85, 0.2, 0.2, 0.16);
    constexpr int kSamples = 64;
    std::vector<geometry_msgs::msg::Point> pts;
    pts.reserve(kSamples + 1);
    for (int i = 0; i <= kSamples; ++i) {
      const double y = -vel_axis_limit_ + 2.0 * vel_axis_limit_ * i / kSamples;
      pts.push_back(V(b.boundary_vertex_x - b.boundary_curvature * y * y, y));
    }
    curve.points = pts;
    out.markers.push_back(curve);
    for (int i = 0; i < kSamples; ++i) {
      const auto& a = pts[static_cast<std::size_t>(i)];
      const auto& c = pts[static_cast<std::size_t>(i) + 1];
      fill.points.push_back(Pt(cx - half, a.y, 0.0));
      fill.points.push_back(a);
      fill.points.push_back(c);
      fill.points.push_back(Pt(cx - half, a.y, 0.0));
      fill.points.push_back(c);
      fill.points.push_back(Pt(cx - half, c.y, 0.0));
    }
    out.markers.push_back(fill);

    Marker dot = Base(stamp, "vel_state", k, Marker::SPHERE, vel_frame_);
    dot.pose.position = V(b.relative_velocity_los[0], b.relative_velocity_los[1]);
    dot.scale.x = dot.scale.y = dot.scale.z = 0.09;
    dot.color = b.h < 0.0 ? Col(1.0, 0.1, 0.1, 1.0) : Col(0.05, 0.55, 0.15, 1.0);
    out.markers.push_back(dot);

    Marker arrow = Base(stamp, "vel_state", 100 + k, Marker::LINE_LIST,
                        vel_frame_);
    arrow.scale.x = 0.02;
    arrow.color = dot.color;
    arrow.points.push_back(Pt(cx, 0.0, 0.0));
    arrow.points.push_back(dot.pose.position);
    out.markers.push_back(arrow);

    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "#%d uid=%d  d=%.2f m  h=%.3f\nvertex=%.2f m/s  "
                  "curv=%.2f s/m",
                  b.rank + 1, b.obstacle.id, b.distance, b.h,
                  b.boundary_vertex_x, b.boundary_curvature);
    out.markers.push_back(Text(stamp, "vel_card", 200 + k, cx, half + 0.16,
                               0.13, buf, Col(0.1, 0.1, 0.1, 1.0), vel_frame_));
    out.markers.push_back(Text(stamp, "vel_axes", 300 + k, cx + half - 0.28,
                               -0.14, 0.11, "x~ [m/s] radial",
                               Col(0.45, 0.45, 0.45, 1.0), vel_frame_));
    out.markers.push_back(Text(stamp, "vel_axes", 400 + k, cx + 0.14,
                               half - 0.14, 0.11, "y~ [m/s]",
                               Col(0.45, 0.45, 0.45, 1.0), vel_frame_));
  }

  void DrawBanner(visualization_msgs::msg::MarkerArray& out,
                  const rclcpp::Time& stamp, bool gt_live,
                  std::size_t n_selected) const {
    // Kept deliberately narrow: TEXT_VIEW_FACING width grows with character
    // count, and under TopDownOrtho a long line sprawls across the whole
    // viewport and stops being readable as a block.
    char head[128];
    std::snprintf(head, sizeof(head), "mode=%s  dpcbf=%s %ss\nsel %zu/%zu  "
                  "p_max=%.1fm",
                  status_mode_.empty() ? "?" : status_mode_.c_str(),
                  status_state_.empty() ? "?" : status_state_.c_str(),
                  status_age_.empty() ? "?" : status_age_.c_str(), n_selected,
                  safe_.size(), params_.detection_radius);
    std::string text = head;
    text += gt_live ? "\nGT: live" : "\nGT UNAVAILABLE - estimated only";
    // Selection depends on closing alignment, hence on the estimated
    // velocity, so the selected-set layer is indicative for the same reason
    // the parabolas are. The workspace layers (gt/tracked/safe/error/eCBF)
    // are NOT -- they are read straight off the topics.
    text += vel_valid_ ? "\nv: d/dt odom - sel/h INDICATIVE"
                       : "\nv: NOT VALID - sel/h meaningless";
    if (Stale()) text += "\n*** " + status_state_ + ": SET RETAINED ***";
    if (error_magnify_ != 1.0) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "\n*** ERROR MAGNIFIED x%.2f ***",
                    error_magnify_);
      text += buf;
    }
    auto col = Col(0.05, 0.05, 0.05, 1.0);
    if (!gt_live) col = Col(0.85, 0.45, 0.0, 1.0);
    if (Stale()) col = Col(0.85, 0.1, 0.1, 1.0);
    out.markers.push_back(Text(stamp, "status", 0, robot_.x,
                               robot_.y - params_.detection_radius - 0.7, 0.16,
                               text, col));
  }

  // ---- gate log ----------------------------------------------------------

  // One JSONL record per published tick, keyed by the SIM-time stamp of the
  // /obstacles_safe frame the recomputation consumed -- not by wall time and
  // not by the node's own tick, so boundary_check can join it against the
  // 1 kHz capture without guessing.
  void LogTick(double t_now, const std::vector<dra::BoundaryObstacle>& sel) {
    if (!log_) return;
    std::fprintf(log_,
                 "{\"t_safe\":%.9f,\"t_odom\":%.9f,\"t_pub\":%.9f,"
                 "\"robot\":[%.17g,%.17g,%.17g,%.17g,%.17g],"
                 "\"n_input\":%zu,\"selected\":[",
                 safe_stamp_, odom_stamp_, t_now, robot_.x, robot_.y,
                 robot_.phi, robot_.sagittal_velocity, robot_.lateral_velocity,
                 safe_.size());
    for (std::size_t i = 0; i < sel.size(); ++i) {
      const auto& b = sel[i];
      std::fprintf(log_,
                   "%s{\"id\":%d,\"rank\":%d,\"d\":%.17g,\"h\":%.17g,"
                   "\"vertex\":%.17g,\"curv\":%.17g,\"ecbf_r\":%.17g}",
                   i ? "," : "", b.obstacle.id, b.rank, b.distance, b.h,
                   b.boundary_vertex_x, b.boundary_curvature, b.ecbf_radius);
    }
    std::fprintf(log_, "]}\n");
    if ((++log_records_ & 0x1fu) == 0u) std::fflush(log_);
  }

  // ---- helpers -----------------------------------------------------------

  bool InScope(const Circle& c) const {
    return std::hypot(c.x - robot_.x, c.y - robot_.y) <=
           params_.detection_radius + 1.5;
  }

  bool Stale() const {
    return status_level_ > diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  Marker Base(const rclcpp::Time& stamp, const std::string& ns, int id,
              std::int32_t type, const std::string& frame) const {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  Marker Text(const rclcpp::Time& stamp, const std::string& ns, int id,
              double x, double y, double h, const std::string& s,
              const std_msgs::msg::ColorRGBA& col,
              const std::string& frame = std::string()) const {
    Marker m = Base(stamp, ns, id, Marker::TEXT_VIEW_FACING,
                    frame.empty() ? frame_ : frame);
    m.pose.position = Pt(x, y, 0.15);
    m.scale.z = h;
    m.color = col;
    m.text = s;
    return m;
  }

  // params
  dra::BoundaryParams params_;
  std::string frame_, vel_frame_, log_path_;
  double rate_hz_ = 10.0, gt_timeout_ = 2.0, nn_gate_ = 0.5;
  double vel_scale_ = 0.35, vel_axis_limit_ = 3.0, panel_offset_ = 4.6;
  double error_magnify_ = 1.0;
  int max_cards_ = 3;
  double vel_tau_ = 0.02;  // pose-difference smoothing constant [s]

  // state
  dpcbf::RobotState robot_;
  std::vector<Circle> gt_, tracked_, safe_;
  double gt_stamp_ = 0.0, safe_stamp_ = 0.0, odom_stamp_ = 0.0;
  double prev_t_ = 0.0, prev_x_ = 0.0, prev_y_ = 0.0;
  double world_vx_ = 0.0, world_vy_ = 0.0;
  bool have_prev_ = false, have_odom_ = false, vel_valid_ = false;
  std::uint8_t status_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string status_state_, status_mode_, status_age_;
  double status_stamp_ = 0.0;
  std::FILE* log_ = nullptr;
  std::uint64_t log_records_ = 0;

  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_gt_,
      sub_tracked_, sub_safe_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      sub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DpcbfOverlay>());
  rclcpp::shutdown();
  return 0;
}
