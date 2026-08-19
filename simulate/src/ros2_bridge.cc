#ifdef UNITREE_MUJOCO_WITH_ROS2

#include "ros2_bridge.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sim_msgs/msg/mj_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <filesystem>

#include "param.h"

namespace {

constexpr double kClockPeriod = 0.004;  // 250 Hz  (§7.1: >=100 Hz)
constexpr double kStatePeriod = 0.010;  // 100 Hz
constexpr double kGtPeriod = 0.020;     //  50 Hz

builtin_interfaces::msg::Time ToRosTime(double t) {
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(t);
  out.nanosec = static_cast<uint32_t>((t - out.sec) * 1e9);
  return out;
}

std::string MirrorXmlPath() {
  const char* env = std::getenv("UNITREE_MUJOCO_MIRROR_XML");
  return env ? env : "/tmp/unitree_mujoco_mirror_model.xml";
}

}  // namespace

void Ros2DumpMirrorModel(mjSpec* spec) {
  namespace fs = std::filesystem;
  const std::string path = MirrorXmlPath();
  char error[512] = "";
  if (mj_saveXML(spec, path.c_str(), error, sizeof(error)) != 0) {
    std::cerr << "[ros2_bridge] mirror model save FAILED: " << error << std::endl;
    return;
  }
  // mj_saveXML keeps compiler asset dirs relative to the original scene
  // location; rewrite them absolute so the mirror loads from anywhere.
  std::ifstream in(path);
  std::string xml((std::istreambuf_iterator<char>(in)),
                  std::istreambuf_iterator<char>());
  in.close();
  const fs::path scene_dir = param::config.robot_scene.parent_path();
  const auto absolutize = [&](const std::string& attr) {
    const std::string key = attr + "=\"";
    const auto pos = xml.find(key);
    if (pos == std::string::npos) return;
    const auto end = xml.find('"', pos + key.size());
    const std::string rel = xml.substr(pos + key.size(), end - pos - key.size());
    const fs::path abs = scene_dir / fs::path(rel);
    xml.replace(pos + key.size(), end - (pos + key.size()), abs.string());
  };
  absolutize("meshdir");
  absolutize("texturedir");
  std::ofstream out(path);
  out << xml;
  std::cout << "[ros2_bridge] mirror model saved: " << path << std::endl;
}

struct SimRos2Bridge::Impl {
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<sim_msgs::msg::MjState>::SharedPtr state_pub;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr gt_pub;
  double last_clock = -1.0;
  double last_state = -1.0;
  double last_gt = -1.0;
  int idle_ticks = 0;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub;
  visualization_msgs::msg::MarkerArray latest_markers;
  std::chrono::steady_clock::time_point marker_received{};
};

SimRos2Bridge::SimRos2Bridge() : impl_(new Impl) {
  // Both DDS stacks must share one CycloneDDS config (R-3/T10): pin the
  // interface from config.yaml for the whole process unless the user already
  // exported CYCLONEDDS_URI, and default the ROS domain to the SDK2 domain.
  if (!std::getenv("CYCLONEDDS_URI")) {
    // MaxAutoParticipantIndex: the whole sim+perception+CLI fleet shares one
    // host/interface; cyclone's default (~10) exhausts fast (T10 follow-on).
    const std::string uri =
        "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"" +
        param::config.interface +
        (param::config.interface == "lo" ? "\" multicast=\"true" : "") +
        "\"/></Interfaces></General><Discovery>"
        "<ParticipantIndex>auto</ParticipantIndex>"
        "<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>"
        "</Discovery></Domain></CycloneDDS>";
    setenv("CYCLONEDDS_URI", uri.c_str(), 0);
  }
  if (!std::getenv("ROS_DOMAIN_ID")) {
    setenv("ROS_DOMAIN_ID", std::to_string(param::config.domain_id).c_str(), 0);
  }
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"use_sim_time", false}});  // we PRODUCE /clock
  impl_->node = rclcpp::Node::make_shared("simulate_ros2_bridge", opts);
  impl_->clock_pub = impl_->node->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
  impl_->state_pub = impl_->node->create_publisher<sim_msgs::msg::MjState>(
      "/sim/mj_state",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  impl_->gt_pub = impl_->node->create_publisher<obstacle_detector::msg::Obstacles>(
      "/sim/gt_obstacles", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  impl_->marker_sub = impl_->node->create_subscription<
      visualization_msgs::msg::MarkerArray>(
      "/navigation/markers", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        impl_->latest_markers = *msg;
        impl_->marker_received = std::chrono::steady_clock::now();
      });
  std::cout << "[ros2_bridge] publishing /clock /sim/mj_state /sim/gt_obstacles"
            << std::endl;
}

SimRos2Bridge::~SimRos2Bridge() {
  impl_.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
}

void SimRos2Bridge::SpinOnce(
    const mjModel* m, const mjData* d,
    const std::function<std::vector<dpcbf::DynamicObstacle>()>& snapshot) {
  if (!m || !d) return;
  rclcpp::spin_some(impl_->node);
  const double t = d->time;

  // /clock — sim-time cadence, plus a keep-alive while paused so late
  // joiners still receive the current sim time.
  if (t - impl_->last_clock >= kClockPeriod || t < impl_->last_clock ||
      ++impl_->idle_ticks >= 100) {
    impl_->idle_ticks = 0;
    rosgraph_msgs::msg::Clock clock;
    clock.clock = ToRosTime(t);
    impl_->clock_pub->publish(clock);
    impl_->last_clock = t;
  }

  // /sim/mj_state
  if (t - impl_->last_state >= kStatePeriod || t < impl_->last_state) {
    sim_msgs::msg::MjState msg;
    msg.sim_time = t;
    msg.qpos.assign(d->qpos, d->qpos + m->nq);
    msg.mocap_pos.assign(d->mocap_pos, d->mocap_pos + 3 * m->nmocap);
    msg.mocap_quat.assign(d->mocap_quat, d->mocap_quat + 4 * m->nmocap);
    impl_->state_pub->publish(msg);
    impl_->last_state = t;
  }

  // /sim/gt_obstacles
  if (t - impl_->last_gt >= kGtPeriod || t < impl_->last_gt) {
    const auto obstacles = snapshot();
    obstacle_detector::msg::Obstacles msg;
    msg.header.stamp = ToRosTime(t);
    msg.header.frame_id = "odom";
    msg.circles.reserve(obstacles.size());
    uint64_t uid = 0;
    for (const auto& o : obstacles) {
      obstacle_detector::msg::CircleObstacle c;
      c.uid = uid++;
      c.center.x = o.position[0];
      c.center.y = o.position[1];
      c.center.z = 0.0;
      c.velocity.x = o.velocity[0];
      c.velocity.y = o.velocity[1];
      c.radius = o.radius;       // ground truth: no margin
      c.true_radius = o.radius;
      msg.circles.push_back(c);
    }
    impl_->gt_pub->publish(msg);
    impl_->last_gt = t;
  }
}

void SimRos2Bridge::UpdateNavigationScene(const mjModel* m,
                                          mjvScene* scene) {
  if (!m || !scene) return;
  scene->ngeom = 0;
  if (impl_->marker_received.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    impl_->marker_received).count() > 0.5) {
    return;
  }
  auto add_connector = [&](int type, double width,
                           const geometry_msgs::msg::Point& a,
                           const geometry_msgs::msg::Point& b,
                           const std_msgs::msg::ColorRGBA& color) {
    if (scene->ngeom >= scene->maxgeom) return;
    mjvGeom* geom = &scene->geoms[scene->ngeom++];
    mjv_initGeom(geom, type, nullptr, nullptr, nullptr,
                 const_cast<float*>(&color.r));
    const mjtNum from[3] = {a.x, a.y, a.z};
    const mjtNum to[3] = {b.x, b.y, b.z};
    mjv_connector(geom, type, width, from, to);
    geom->category = mjCAT_DECOR;
  };
  for (const auto& marker : impl_->latest_markers.markers) {
    if (marker.action == visualization_msgs::msg::Marker::DELETEALL) continue;
    if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
      if (marker.ns == "relative_velocity_vector") continue;
      const bool flat_line = marker.ns == "dpcbf_boundary" ||
                             marker.ns == "goal_outline";
      for (std::size_t i = 1; i < marker.points.size(); ++i) {
        add_connector(flat_line ? mjGEOM_LINE : mjGEOM_CAPSULE,
                      flat_line ? std::max(1.0, marker.scale.x * 100.0)
                                : std::max(0.003, marker.scale.x * 0.5),
                      marker.points[i - 1], marker.points[i], marker.color);
      }
    } else if (marker.type == visualization_msgs::msg::Marker::ARROW &&
               marker.points.size() >= 2) {
      add_connector(mjGEOM_ARROW, std::max(0.005, marker.scale.x * 0.5),
                    marker.points[0], marker.points[1], marker.color);
    } else if ((marker.type == visualization_msgs::msg::Marker::CYLINDER ||
                marker.type == visualization_msgs::msg::Marker::CUBE ||
                marker.type == visualization_msgs::msg::Marker::SPHERE) &&
               scene->ngeom < scene->maxgeom) {
      // ROS Marker scales are full x/y/z extents.  MuJoCo primitive sizes are
      // type-specific: cylinder=(radius, half-height, unused), sphere=(radius,
      // unused, unused), box=(half-x, half-y, half-z).  Treating every marker
      // like a box made a 0.6 m-wide goal disk incorrectly 0.6 m tall.
      mjtNum size[3] = {0.0, 0.0, 0.0};
      if (marker.type == visualization_msgs::msg::Marker::CYLINDER) {
        size[0] = 0.5 * marker.scale.x;
        size[1] = 0.5 * marker.scale.z;
      } else if (marker.type == visualization_msgs::msg::Marker::SPHERE) {
        size[0] = 0.5 * marker.scale.x;
      } else {
        size[0] = 0.5 * marker.scale.x;
        size[1] = 0.5 * marker.scale.y;
        size[2] = 0.5 * marker.scale.z;
      }
      mjtNum pos[3] = {marker.pose.position.x, marker.pose.position.y,
                       marker.pose.position.z};
      mjtNum quat[4] = {marker.pose.orientation.w, marker.pose.orientation.x,
                        marker.pose.orientation.y, marker.pose.orientation.z};
      mjtNum mat[9];
      mju_quat2Mat(mat, quat);
      int geom_type = marker.type == visualization_msgs::msg::Marker::CYLINDER
                          ? mjGEOM_CYLINDER
                          : marker.type == visualization_msgs::msg::Marker::CUBE
                                ? mjGEOM_BOX : mjGEOM_SPHERE;
      mjvGeom* geom = &scene->geoms[scene->ngeom++];
      mjv_initGeom(geom, geom_type, size, pos, mat,
                   const_cast<float*>(&marker.color.r));
      geom->category = mjCAT_DECOR;
    }
  }
}

#endif  // UNITREE_MUJOCO_WITH_ROS2
