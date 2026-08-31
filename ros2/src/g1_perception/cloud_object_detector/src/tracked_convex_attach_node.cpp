// Joins autoware TrackedObjects (ids, velocities, 2.5D shapes) with the
// detector's 3D convex hulls (perception_3d_msgs/ConvexObjects) into
// tracked_convex: one convex polytope per track, in the world frame.
//
// Join key: the polygon tracker publishes each track at its LAST MEASUREMENT
// pose verbatim (getTrackedObject → object.pose = last_pose_), and the
// detector stamps the same pose into ConvexObject.centroid — so a track and
// its hull agree to floating-point precision whenever the track was updated
// this frame. A track without a fresh hull (coasting through an occlusion)
// carries its last hull, translated to the predicted pose, flagged
// `coasting`; a track with no hull ever (should not happen) gets a prism
// from its own footprint. No association logic lives here — that is the
// tracker's job.
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <vector>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <perception_3d_msgs/msg/convex_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace cloud_object_detector {

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using perception_3d_msgs::msg::ConvexObject;
using perception_3d_msgs::msg::ConvexObjects;

class TrackedConvexAttachNode : public rclcpp::Node {
 public:
  explicit TrackedConvexAttachNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tracked_convex_attach", options) {
    join_tolerance_ = declare_parameter("join_tolerance", 0.05);  // [m]
    buffer_size_ = static_cast<std::size_t>(declare_parameter("buffer_frames", 10));
    pub_ = create_publisher<ConvexObjects>(
        "tracked_convex", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    det_sub_ = create_subscription<ConvexObjects>(
        "detected_convex", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](ConvexObjects::ConstSharedPtr msg) {
          detections_.push_front(msg);
          while (detections_.size() > buffer_size_) detections_.pop_back();
        });
    trk_sub_ = create_subscription<TrackedObjects>(
        "tracked_objects", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [this](TrackedObjects::ConstSharedPtr msg) { OnTracked(*msg); });
  }

 private:
  struct Cached {
    ConvexObject hull;  // absolute vertices
    geometry_msgs::msg::Point at;  // centroid the vertices belong to
  };
  using Uuid = std::array<uint8_t, 16>;

  static Uuid Key(const TrackedObject& t) {
    Uuid k;
    std::copy(t.object_id.uuid.begin(), t.object_id.uuid.end(), k.begin());
    return k;
  }

  const ConvexObject* FindFresh(const geometry_msgs::msg::Point& p) const {
    const double tol2 = join_tolerance_ * join_tolerance_;
    for (const auto& msg : detections_) {  // newest first
      for (const auto& d : msg->objects) {
        const double dx = d.centroid.x - p.x, dy = d.centroid.y - p.y;
        if (dx * dx + dy * dy <= tol2) return &d;
      }
    }
    return nullptr;
  }

  static void Translate(ConvexObject* obj, const geometry_msgs::msg::Point& from,
                        const geometry_msgs::msg::Point& to) {
    const double dx = to.x - from.x, dy = to.y - from.y, dz = to.z - from.z;
    for (auto& v : obj->vertices) {
      v.x += dx;
      v.y += dy;
      v.z += dz;
    }
    obj->centroid = to;
  }

  static ConvexObject PrismFromTrack(const TrackedObject& t) {
    ConvexObject obj;
    const auto& pose = t.kinematics.pose_with_covariance.pose;
    const auto& fp = t.shape.footprint.points;
    const std::size_t n = fp.size();
    const double h = t.shape.dimensions.z;
    const auto& q = pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (double dz : {-0.5 * h, 0.5 * h}) {
      for (const auto& f : fp) {
        geometry_msgs::msg::Point v;
        v.x = pose.position.x + c * f.x - s * f.y;
        v.y = pose.position.y + s * f.x + c * f.y;
        v.z = pose.position.z + dz;
        obj.vertices.push_back(v);
      }
    }
    auto tri = [&](uint32_t a, uint32_t b, uint32_t cc) {
      obj.triangles.push_back(a);
      obj.triangles.push_back(b);
      obj.triangles.push_back(cc);
    };
    for (std::size_t i = 0; i < n; ++i) {
      const uint32_t a = i, b = (i + 1) % n, cc = n + (i + 1) % n, d = n + i;
      tri(a, b, cc);
      tri(a, cc, d);
    }
    for (std::size_t k = 2; k < n; ++k) {
      tri(0, k, k - 1);
      tri(n, n + k - 1, n + k);
    }
    obj.height = static_cast<float>(h);
    obj.centroid = pose.position;
    return obj;
  }

  void OnTracked(const TrackedObjects& msg) {
    ConvexObjects out;
    out.header = msg.header;
    std::map<Uuid, Cached> next_cache;
    for (const auto& t : msg.objects) {
      const auto& pose = t.kinematics.pose_with_covariance.pose;
      const Uuid key = Key(t);
      ConvexObject obj;
      if (const ConvexObject* fresh = FindFresh(pose.position)) {
        obj = *fresh;
        obj.coasting = false;
        ++joined_;
      } else if (auto it = cache_.find(key); it != cache_.end()) {
        obj = it->second.hull;
        Translate(&obj, it->second.at, pose.position);
        obj.coasting = true;
        ++coasted_;
      } else {
        obj = PrismFromTrack(t);
        obj.coasting = true;
        ++prism_fallbacks_;
      }
      obj.object_id = t.object_id;
      obj.centroid = pose.position;
      // twist is in the object frame; export world-frame velocity.
      const auto& q = pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const auto& tw = t.kinematics.twist_with_covariance.twist.linear;
      obj.velocity.x = std::cos(yaw) * tw.x - std::sin(yaw) * tw.y;
      obj.velocity.y = std::sin(yaw) * tw.x + std::cos(yaw) * tw.y;
      obj.velocity.z = 0.0;
      next_cache[key] = Cached{obj, pose.position};
      out.objects.push_back(std::move(obj));
    }
    cache_.swap(next_cache);  // tracks that vanished drop out of the cache
    pub_->publish(out);
    if (prism_fallbacks_ != last_prism_fallbacks_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "%ld track(s) had no hull ever and got a footprint "
                           "prism (joined %ld, coasted %ld)",
                           prism_fallbacks_, joined_, coasted_);
      last_prism_fallbacks_ = prism_fallbacks_;
    }
  }

  double join_tolerance_ = 0.05;
  std::size_t buffer_size_ = 10;
  std::deque<ConvexObjects::ConstSharedPtr> detections_;
  std::map<Uuid, Cached> cache_;
  long joined_ = 0, coasted_ = 0, prism_fallbacks_ = 0, last_prism_fallbacks_ = 0;
  rclcpp::Publisher<ConvexObjects>::SharedPtr pub_;
  rclcpp::Subscription<ConvexObjects>::SharedPtr det_sub_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr trk_sub_;
};

}  // namespace cloud_object_detector

RCLCPP_COMPONENTS_REGISTER_NODE(cloud_object_detector::TrackedConvexAttachNode)
