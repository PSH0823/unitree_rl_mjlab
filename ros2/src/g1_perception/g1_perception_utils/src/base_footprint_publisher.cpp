// odom -> base_footprint TF (§8.1/§8.2): base_link projected to the ground
// plane. Published at ~100 Hz from the latest odom->base_link transform;
// deduplicated by stamp so a stalled odometry source does not spam TF.
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "g1_perception_utils/footprint.hpp"

using namespace std::chrono_literals;

class BaseFootprintPublisher : public rclcpp::Node {
 public:
  BaseFootprintPublisher() : Node("base_footprint_publisher") {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_footprint");
    const double rate = declare_parameter<double>("rate", 100.0);

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this] { Tick(); });
  }

 private:
  void Tick() {
    geometry_msgs::msg::TransformStamped in;
    try {
      in = buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return;  // odometry not up yet
    }
    if (in.header.stamp == last_stamp_) return;
    last_stamp_ = in.header.stamp;

    g1_perception_utils::Pose base;
    base.x = in.transform.translation.x;
    base.y = in.transform.translation.y;
    base.z = in.transform.translation.z;
    base.qx = in.transform.rotation.x;
    base.qy = in.transform.rotation.y;
    base.qz = in.transform.rotation.z;
    base.qw = in.transform.rotation.w;
    const auto fp = g1_perception_utils::ProjectToFootprint(base);

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = in.header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = footprint_frame_;
    out.transform.translation.x = fp.x;
    out.transform.translation.y = fp.y;
    out.transform.translation.z = fp.z;
    out.transform.rotation.x = fp.qx;
    out.transform.rotation.y = fp.qy;
    out.transform.rotation.z = fp.qz;
    out.transform.rotation.w = fp.qw;
    broadcaster_->sendTransform(out);
  }

  std::string odom_frame_, base_frame_, footprint_frame_;
  builtin_interfaces::msg::Time last_stamp_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseFootprintPublisher>());
  rclcpp::shutdown();
  return 0;
}
