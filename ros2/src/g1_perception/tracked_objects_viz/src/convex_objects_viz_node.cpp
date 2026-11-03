// ConvexObjects → MarkerArray: each polytope as a translucent TRIANGLE_LIST
// plus its edge wireframe, a world-frame velocity arrow, and the short id.
// Coasting hulls (carried through an occlusion) are drawn grey.
#include <cmath>
#include <cstdio>
#include <set>
#include <utility>

#include <perception_3d_msgs/msg/convex_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tracked_objects_viz {

using perception_3d_msgs::msg::ConvexObject;
using perception_3d_msgs::msg::ConvexObjects;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class ConvexObjectsVizNode : public rclcpp::Node {
 public:
  explicit ConvexObjectsVizNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("convex_objects_viz", options) {
    lifetime_ = declare_parameter("marker_lifetime", 0.5);
    min_arrow_speed_ = declare_parameter("min_arrow_speed", 0.05);
    pub_ = create_publisher<MarkerArray>("convex_markers",
                                         rclcpp::QoS(rclcpp::KeepLast(1)));
    sub_ = create_subscription<ConvexObjects>(
        "convex_objects", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [this](ConvexObjects::ConstSharedPtr msg) { Publish(*msg); });
  }

 private:
  static int32_t MarkerId(const ConvexObject& obj, int kind) {
    const auto& u = obj.object_id.uuid;
    const int32_t base = (u[0] | (u[1] << 8) | (u[2] << 16)) & 0x0FFFFFFF;
    return base * 4 + kind;
  }
  // Distinct-ish per-track hue from the uuid.
  static void Color(const ConvexObject& obj, std_msgs::msg::ColorRGBA* c) {
    const double h = (obj.object_id.uuid[3] * 37 % 360) / 60.0;
    const double x = 1.0 - std::abs(std::fmod(h, 2.0) - 1.0);
    double r = 0, g = 0, b = 0;
    if (h < 1) { r = 1; g = x; } else if (h < 2) { r = x; g = 1; }
    else if (h < 3) { g = 1; b = x; } else if (h < 4) { g = x; b = 1; }
    else if (h < 5) { r = x; b = 1; } else { r = 1; b = x; }
    c->r = 0.3F + 0.7F * static_cast<float>(r);
    c->g = 0.3F + 0.7F * static_cast<float>(g);
    c->b = 0.3F + 0.7F * static_cast<float>(b);
  }

  void Publish(const ConvexObjects& msg) {
    MarkerArray out;
    const auto life = rclcpp::Duration::from_seconds(lifetime_);
    for (const auto& obj : msg.objects) {
      std_msgs::msg::ColorRGBA col;
      if (obj.coasting) {
        col.r = col.g = col.b = 0.6F;
      } else {
        Color(obj, &col);
      }

      Marker faces;
      faces.header = msg.header;
      faces.ns = "hull";
      faces.id = MarkerId(obj, 0);
      faces.type = Marker::TRIANGLE_LIST;
      faces.action = Marker::ADD;
      faces.scale.x = faces.scale.y = faces.scale.z = 1.0;
      faces.color = col;
      faces.color.a = 0.35F;
      faces.lifetime = life;
      faces.pose.orientation.w = 1.0;
      std::set<std::pair<uint32_t, uint32_t>> edges;
      const std::size_t nv = obj.vertices.size();
      for (std::size_t k = 0; k + 2 < obj.triangles.size(); k += 3) {
        const uint32_t a = obj.triangles[k], b = obj.triangles[k + 1],
                       c = obj.triangles[k + 2];
        if (a >= nv || b >= nv || c >= nv) continue;
        faces.points.push_back(obj.vertices[a]);
        faces.points.push_back(obj.vertices[b]);
        faces.points.push_back(obj.vertices[c]);
        edges.insert({std::min(a, b), std::max(a, b)});
        edges.insert({std::min(b, c), std::max(b, c)});
        edges.insert({std::min(a, c), std::max(a, c)});
      }
      if (!faces.points.empty()) out.markers.push_back(faces);

      Marker wire;
      wire.header = msg.header;
      wire.ns = "edges";
      wire.id = MarkerId(obj, 1);
      wire.type = Marker::LINE_LIST;
      wire.action = Marker::ADD;
      wire.scale.x = 0.01;
      wire.color = col;
      wire.color.a = 0.9F;
      wire.lifetime = life;
      wire.pose.orientation.w = 1.0;
      for (const auto& e : edges) {
        wire.points.push_back(obj.vertices[e.first]);
        wire.points.push_back(obj.vertices[e.second]);
      }
      if (!wire.points.empty()) out.markers.push_back(wire);

      const double speed = std::hypot(obj.velocity.x, obj.velocity.y);
      if (speed >= min_arrow_speed_) {
        Marker arrow;
        arrow.header = msg.header;
        arrow.ns = "velocity";
        arrow.id = MarkerId(obj, 2);
        arrow.type = Marker::ARROW;
        arrow.action = Marker::ADD;
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.08;
        arrow.scale.z = 0.08;
        arrow.color.r = 1.0F;
        arrow.color.g = 0.5F;
        arrow.color.a = 0.9F;
        arrow.lifetime = life;
        geometry_msgs::msg::Point from = obj.centroid, to = obj.centroid;
        from.z += 0.5 * obj.height;
        to.x += obj.velocity.x;  // world frame already
        to.y += obj.velocity.y;
        to.z = from.z;
        arrow.points = {from, to};
        out.markers.push_back(arrow);
      }

      Marker text;
      text.header = msg.header;
      text.ns = "id";
      text.id = MarkerId(obj, 3);
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.scale.z = 0.15;
      text.color.r = text.color.g = text.color.b = 1.0F;
      text.color.a = 0.9F;
      text.lifetime = life;
      text.pose.position = obj.centroid;
      text.pose.position.z += 0.5 * obj.height + 0.2;
      text.pose.orientation.w = 1.0;
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
  rclcpp::Subscription<ConvexObjects>::SharedPtr sub_;
};

}  // namespace tracked_objects_viz

RCLCPP_COMPONENTS_REGISTER_NODE(tracked_objects_viz::ConvexObjectsVizNode)
