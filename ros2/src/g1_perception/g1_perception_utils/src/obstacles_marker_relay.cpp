// obstacle_detector/Obstacles -> visualization_msgs/MarkerArray for RViz and
// Foxglove (§6.3, §14.4). Circles become cylinders; velocities become arrows;
// track uids become text labels (uid = 0 on sources that don't assign ids).
#include <memory>
#include <string>

#include <obstacle_detector/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObstaclesMarkerRelay : public rclcpp::Node {
 public:
  ObstaclesMarkerRelay() : Node("obstacles_marker_relay") {
    const auto topic = declare_parameter<std::string>("topic", "/tracked_obstacles");
    height_ = declare_parameter<double>("cylinder_height", 1.5);
    color_r_ = declare_parameter<double>("color_r", 0.9);
    color_g_ = declare_parameter<double>("color_g", 0.3);
    color_b_ = declare_parameter<double>("color_b", 0.1);
    alpha_ = declare_parameter<double>("alpha", 0.6);
    show_ids_ = declare_parameter<bool>("show_ids", true);

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        std::string(get_name()) + "/markers", rclcpp::QoS(1));
    sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
        topic, rclcpp::QoS(5),
        [this](obstacle_detector::msg::Obstacles::ConstSharedPtr msg) { Relay(*msg); });
  }

 private:
  void Relay(const obstacle_detector::msg::Obstacles& msg) {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear;
    clear.header = msg.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);

    int id = 0;
    for (const auto& c : msg.circles) {
      visualization_msgs::msg::Marker m;
      m.header = msg.header;
      m.ns = "circles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.center.x;
      m.pose.position.y = c.center.y;
      m.pose.position.z = height_ / 2.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = 2.0 * c.radius;
      m.scale.y = 2.0 * c.radius;
      m.scale.z = height_;
      m.color.r = color_r_;
      m.color.g = color_g_;
      m.color.b = color_b_;
      m.color.a = alpha_;
      out.markers.push_back(m);

      const double speed = std::hypot(c.velocity.x, c.velocity.y);
      if (speed > 1e-3) {
        visualization_msgs::msg::Marker a;
        a.header = msg.header;
        a.ns = "velocities";
        a.id = id++;
        a.type = visualization_msgs::msg::Marker::ARROW;
        a.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point p0, p1;
        p0.x = c.center.x;
        p0.y = c.center.y;
        p0.z = height_;
        p1.x = c.center.x + c.velocity.x;
        p1.y = c.center.y + c.velocity.y;
        p1.z = height_;
        a.points = {p0, p1};
        a.scale.x = 0.04;
        a.scale.y = 0.10;
        a.color.r = 1.0;
        a.color.g = 0.9;
        a.color.b = 0.1;
        a.color.a = 0.9;
        out.markers.push_back(a);
      }

      if (show_ids_) {
        visualization_msgs::msg::Marker t;
        t.header = msg.header;
        t.ns = "ids";
        t.id = id++;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.text = std::to_string(c.uid);
        t.pose.position.x = c.center.x;
        t.pose.position.y = c.center.y;
        t.pose.position.z = height_ + 0.25;
        t.pose.orientation.w = 1.0;
        t.scale.z = 0.25;
        t.color.r = 1.0;
        t.color.g = 1.0;
        t.color.b = 1.0;
        t.color.a = 1.0;
        out.markers.push_back(t);
      }
    }
    pub_->publish(out);
  }

  double height_, color_r_, color_g_, color_b_, alpha_;
  bool show_ids_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclesMarkerRelay>());
  rclcpp::shutdown();
  return 0;
}
