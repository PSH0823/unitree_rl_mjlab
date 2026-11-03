// TrackedObjects → MarkerArray relay for the 3D pipeline. Per object: the
// footprint prism as a LINE_LIST wireframe (base + top rings + verticals), a
// velocity ARROW (twist rotated from the object frame into the world frame),
// and the first uuid byte as TEXT. Marker ids derive from the uuid so RViz
// updates tracks in place; lifetime expires markers of dropped tracks.
#include <cmath>
#include <string>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tracked_objects_viz {

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class TrackedObjectsVizNode : public rclcpp::Node {
 public:
  explicit TrackedObjectsVizNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tracked_objects_viz", options) {
    lifetime_ = declare_parameter("marker_lifetime", 0.5);
    min_arrow_speed_ = declare_parameter("min_arrow_speed", 0.05);
    pub_ = create_publisher<MarkerArray>(
        "tracked_objects_markers", rclcpp::QoS(rclcpp::KeepLast(1)));
    sub_ = create_subscription<TrackedObjects>(
        "tracked_objects", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [this](TrackedObjects::ConstSharedPtr msg) { Publish(*msg); });
  }

 private:
  // Stable per-track marker id from the uuid; `kind` separates the three
  // markers of one object.
  static int32_t MarkerId(const TrackedObject& obj, int kind) {
    const auto& u = obj.object_id.uuid;
    const int32_t base = (u[0] | (u[1] << 8) | (u[2] << 16)) & 0x0FFFFFFF;
    return base * 4 + kind;
  }

  void Publish(const TrackedObjects& msg) {
    MarkerArray out;
    const auto life = rclcpp::Duration::from_seconds(lifetime_);
    for (const auto& obj : msg.objects) {
      const auto& pose = obj.kinematics.pose_with_covariance.pose;
      const double half_h = 0.5 * obj.shape.dimensions.z;
      const auto& q = pose.orientation;
      const double yaw =
          std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double c = std::cos(yaw), s = std::sin(yaw);

      Marker prism;
      prism.header = msg.header;
      prism.ns = "prism";
      prism.id = MarkerId(obj, 0);
      prism.type = Marker::LINE_LIST;
      prism.action = Marker::ADD;
      prism.scale.x = 0.02;
      prism.color.g = 1.0F;
      prism.color.b = 0.4F;
      prism.color.a = 0.9F;
      prism.lifetime = life;
      const auto& fp = obj.shape.footprint.points;
      const std::size_t n = fp.size();
      auto world = [&](std::size_t i, double z) {
        geometry_msgs::msg::Point p;
        p.x = pose.position.x + c * fp[i].x - s * fp[i].y;
        p.y = pose.position.y + s * fp[i].x + c * fp[i].y;
        p.z = pose.position.z + z;
        return p;
      };
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        prism.points.push_back(world(i, -half_h));  // base ring
        prism.points.push_back(world(j, -half_h));
        prism.points.push_back(world(i, half_h));   // top ring
        prism.points.push_back(world(j, half_h));
        prism.points.push_back(world(i, -half_h));  // vertical edge
        prism.points.push_back(world(i, half_h));
      }
      out.markers.push_back(prism);

      const auto& tw = obj.kinematics.twist_with_covariance.twist.linear;
      const double speed = std::hypot(tw.x, tw.y);
      if (speed >= min_arrow_speed_) {
        Marker arrow;
        arrow.header = msg.header;
        arrow.ns = "velocity";
        arrow.id = MarkerId(obj, 1);
        arrow.type = Marker::ARROW;
        arrow.action = Marker::ADD;
        arrow.scale.x = 0.03;  // shaft
        arrow.scale.y = 0.08;  // head width
        arrow.scale.z = 0.08;  // head length
        arrow.color.r = 1.0F;
        arrow.color.g = 0.5F;
        arrow.color.a = 0.9F;
        arrow.lifetime = life;
        geometry_msgs::msg::Point from, to;
        from.x = pose.position.x;
        from.y = pose.position.y;
        from.z = pose.position.z + half_h;
        // twist is in the object frame; rotate into the world frame.
        to.x = from.x + c * tw.x - s * tw.y;
        to.y = from.y + s * tw.x + c * tw.y;
        to.z = from.z;
        arrow.points = {from, to};
        out.markers.push_back(arrow);
      }

      Marker text;
      text.header = msg.header;
      text.ns = "id";
      text.id = MarkerId(obj, 2);
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.scale.z = 0.15;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 0.9F;
      text.lifetime = life;
      text.pose.position.x = pose.position.x;
      text.pose.position.y = pose.position.y;
      text.pose.position.z = pose.position.z + half_h + 0.2;
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%02x%02x", obj.object_id.uuid[0],
                    obj.object_id.uuid[1]);
      text.text = buf;
      out.markers.push_back(text);
    }
    pub_->publish(out);
  }

  double lifetime_ = 0.5;
  double min_arrow_speed_ = 0.05;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_;
};

}  // namespace tracked_objects_viz

RCLCPP_COMPONENTS_REGISTER_NODE(tracked_objects_viz::TrackedObjectsVizNode)
