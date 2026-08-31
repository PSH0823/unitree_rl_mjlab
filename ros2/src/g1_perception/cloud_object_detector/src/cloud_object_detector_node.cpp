// 3D detection front-end: /points_self_filtered (PointCloud2, sensor frame)
// → autoware_perception_msgs/DetectedObjects (frame = the tracking world
// frame, class UNKNOWN, Shape POLYGON = convex footprint + height). Replaces
// the 2D scan → circle path for the 3D pipeline; the tracker downstream is
// autoware_multi_object_tracker, which owns association and velocity
// estimation — this node is stateless per cloud.
//
// Stages: TF to world frame at the cloud stamp (same policy as the 2D
// extractor: stamped lookup, bounded wait) → range + z-band pass (the z band
// replaces the pointcloud_to_laserscan min/max_height band that was the 2D
// path's only floor rejection) → optional voxel downsample → Euclidean
// clustering → per-cluster convex hull (detection_geometry.h).
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <perception_3d_msgs/msg/convex_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cloud_object_detector/detection_geometry.h"

namespace cloud_object_detector {

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using perception_3d_msgs::msg::ConvexObject;
using perception_3d_msgs::msg::ConvexObjects;

class CloudObjectDetectorNode : public rclcpp::Node {
 public:
  explicit CloudObjectDetectorNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("cloud_object_detector", options),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    tf_timeout_ = declare_parameter("tf_timeout", 0.1);
    max_range_ = declare_parameter("max_range", 5.0);
    // z band in the world frame. The lower edge assumes the world frame's
    // z = 0 sits at the floor (true for the sim GT odom and for DLIO's
    // initial pose; revisit if odom z drift shows up on hardware).
    z_min_ = declare_parameter("z_min", 0.10);
    z_max_ = declare_parameter("z_max", 2.0);
    voxel_leaf_ = declare_parameter("voxel_leaf", 0.05);  // 0 disables
    cluster_tolerance_ = declare_parameter("cluster_tolerance", 0.30);
    min_cluster_size_ = declare_parameter("min_cluster_size", 5);
    max_cluster_size_ = declare_parameter("max_cluster_size", 20000);
    shape_params_.min_height = declare_parameter("min_object_height", 0.05);
    shape_params_.max_hull_vertices = static_cast<std::size_t>(
        declare_parameter("max_hull_vertices", 16));
    // Wall gate (detection_geometry.h IsWallLike): long-and-thin footprints
    // and anything over the absolute extent cap are room structure, not
    // obstacles the tracker should spawn on. 0 disables a rule.
    shape_params_.wall_min_length =
        declare_parameter("wall_min_length", shape_params_.wall_min_length);
    shape_params_.wall_max_thickness = declare_parameter(
        "wall_max_thickness", shape_params_.wall_max_thickness);
    shape_params_.max_object_extent = declare_parameter(
        "max_object_extent", shape_params_.max_object_extent);
    // Depth completion: a front-face-only cluster is extruded away from the
    // sensor until its footprint is at least this thick (see
    // detection_geometry.h). 0 disables.
    shape_params_.min_thickness =
        declare_parameter("min_thickness", shape_params_.min_thickness);
    // Published as the detection's position covariance. This matters: without
    // it the tracker's uncertainty model substitutes the UNKNOWN-class
    // default of (1.0 m)^2, the velocity covariance never converges, and the
    // polygon tracker's suppressUncertainVelocity() gate zeroes every twist
    // below ~1 m/s at publish time (measured 2026-08-26: a 0.5 m/s target
    // tracked with position locked on but published twist exactly 0).
    position_stddev_ = declare_parameter("position_stddev", 0.10);

    // Detections are low-rate control inputs for the tracker, not a raw
    // sensor stream: Reliable depth 5, matching /raw_obstacles (§7.1).
    pub_ = create_publisher<DetectedObjects>(
        "detected_objects", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    // Same stamp, same order, same poses as detected_objects: the 3D
    // convex hull of each padded cluster. autoware's Shape is 2.5D (one
    // footprint for every z); this is the full polytope, whose cross-section
    // follows the object with height. Joined to track ids downstream by
    // TrackedConvexAttachNode.
    convex_pub_ = create_publisher<ConvexObjects>(
        "detected_convex", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          Process(*msg);
        });
    RCLCPP_INFO(get_logger(),
                "cloud_object_detector: world=%s range<=%.1f z=[%.2f, %.2f] "
                "voxel=%.2f tol=%.2f cluster=[%d, %d] min_h=%.2f hull<=%zu",
                world_frame_.c_str(), max_range_, z_min_, z_max_, voxel_leaf_,
                cluster_tolerance_, min_cluster_size_, max_cluster_size_,
                shape_params_.min_height, shape_params_.max_hull_vertices);
  }

 private:
  void Process(const sensor_msgs::msg::PointCloud2& msg) {
    // Stamped lookup, not "latest": the cloud is 100 ms old at 10 Hz and the
    // robot moves — same reasoning as the 2D extractor's stamped TF (P-1).
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
          world_frame_, msg.header.frame_id, msg.header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException& ex) {
      ++tf_misses_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s->%s at cloud stamp unavailable (%ld misses): %s",
                           msg.header.frame_id.c_str(), world_frame_.c_str(),
                           tf_misses_, ex.what());
      return;
    }
    const Eigen::Isometry3d to_world = tf2::transformToEigen(tf);
    // Sensor origin in the world frame — range is measured from the sensor,
    // matching the 2D path's range_max semantics, not from the world origin.
    const Eigen::Vector3d sensor = to_world.translation();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr world(
        new pcl::PointCloud<pcl::PointXYZ>);
    world->reserve(cloud->size());
    const double max_range_sq = max_range_ * max_range_;
    for (const auto& q : cloud->points) {
      if (!pcl::isFinite(q)) continue;
      const Eigen::Vector3d w =
          to_world * Eigen::Vector3d(q.x, q.y, q.z);
      if (w.z() < z_min_ || w.z() > z_max_) continue;
      const double dx = w.x() - sensor.x(), dy = w.y() - sensor.y();
      if (dx * dx + dy * dy > max_range_sq) continue;
      world->emplace_back(static_cast<float>(w.x()), static_cast<float>(w.y()),
                          static_cast<float>(w.z()));
    }

    if (voxel_leaf_ > 0.0 && !world->empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(world);
      const float leaf = static_cast<float>(voxel_leaf_);
      voxel.setLeafSize(leaf, leaf, leaf);
      pcl::PointCloud<pcl::PointXYZ>::Ptr down(
          new pcl::PointCloud<pcl::PointXYZ>);
      voxel.filter(*down);
      world = down;
    }

    DetectedObjects out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = world_frame_;
    ConvexObjects convex;
    convex.header = out.header;

    if (static_cast<int>(world->size()) >= min_cluster_size_) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(world);
      std::vector<pcl::PointIndices> clusters;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(world);
      ec.extract(clusters);

      for (const auto& idx : clusters) {
        std::vector<std::array<double, 3>> pts;
        pts.reserve(idx.indices.size());
        for (const int i : idx.indices) {
          const auto& q = (*world)[static_cast<std::size_t>(i)];
          pts.push_back({q.x, q.y, q.z});
        }
        const ShapeEstimate est =
            EstimateShape(pts, shape_params_, sensor.x(), sensor.y());
        if (!est.valid) {
          if (est.is_wall) {
            ++wall_clusters_;
          } else {
            ++degenerate_clusters_;
          }
          continue;
        }
        out.objects.push_back(ToDetectedObject(est, position_stddev_));
        convex.objects.push_back(ToConvexObject(est));
      }
    }
    ReportStats();
    pub_->publish(out);
    convex_pub_->publish(convex);
  }

  // 3D convex hull of the padded cluster (qhull via pcl::ConvexHull,
  // triangulated facets). If qhull cannot (degenerate, near-planar input),
  // fall back to the footprint prism so the object is never missing here
  // while present in detected_objects.
  ConvexObject ToConvexObject(const ShapeEstimate& est) {
    ConvexObject obj;
    obj.centroid.x = est.cx;  // == the DetectedObject pose: the join key
    obj.centroid.y = est.cy;
    obj.centroid.z = est.cz;
    obj.height = static_cast<float>(est.height);
    obj.coasting = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(est.padded_points.size());
    for (const auto& q : est.padded_points) {
      cloud->emplace_back(static_cast<float>(q[0]), static_cast<float>(q[1]),
                          static_cast<float>(q[2]));
    }
    pcl::PointCloud<pcl::PointXYZ> hull_pts;
    std::vector<pcl::Vertices> polys;
    bool ok = false;
    if (cloud->size() >= 4) {
      try {
        pcl::ConvexHull<pcl::PointXYZ> ch;
        ch.setInputCloud(cloud);
        ch.setDimension(3);
        ch.reconstruct(hull_pts, polys);
        ok = hull_pts.size() >= 4 && !polys.empty();
      } catch (const std::exception&) {
        ok = false;
      }
    }
    if (ok) {
      obj.vertices.reserve(hull_pts.size());
      for (const auto& q : hull_pts) {
        geometry_msgs::msg::Point v;
        v.x = q.x;
        v.y = q.y;
        v.z = q.z;
        obj.vertices.push_back(v);
      }
      for (const auto& poly : polys) {  // fan-triangulate defensively
        const auto& idx = poly.vertices;
        for (std::size_t k = 2; k < idx.size(); ++k) {
          obj.triangles.push_back(idx[0]);
          obj.triangles.push_back(idx[k - 1]);
          obj.triangles.push_back(idx[k]);
        }
      }
      return obj;
    }
    ++hull3d_fallbacks_;
    return PrismFallback(est, obj);
  }

  static ConvexObject PrismFallback(const ShapeEstimate& est,
                                    ConvexObject obj) {
    const std::size_t n = est.footprint.size();
    const double z0 = est.cz - 0.5 * est.height, z1 = est.cz + 0.5 * est.height;
    for (double z : {z0, z1}) {
      for (const auto& f : est.footprint) {
        geometry_msgs::msg::Point v;
        v.x = est.cx + f.x;
        v.y = est.cy + f.y;
        v.z = z;
        obj.vertices.push_back(v);
      }
    }
    auto tri = [&](uint32_t a, uint32_t b, uint32_t c) {
      obj.triangles.push_back(a);
      obj.triangles.push_back(b);
      obj.triangles.push_back(c);
    };
    for (std::size_t i = 0; i < n; ++i) {  // sides
      const uint32_t a = i, b = (i + 1) % n, c = n + (i + 1) % n, d = n + i;
      tri(a, b, c);
      tri(a, c, d);
    }
    for (std::size_t k = 2; k < n; ++k) {  // caps
      tri(0, k, k - 1);
      tri(n, n + k - 1, n + k);
    }
    return obj;
  }

  static DetectedObject ToDetectedObject(const ShapeEstimate& est,
                                         double position_stddev) {
    DetectedObject obj;
    // The tracker's input channel for this node is configured with
    // can_trust_existence_probability=false, so this value is a placeholder
    // it overwrites with the channel default.
    obj.existence_probability = 1.0F;
    ObjectClassification cls;
    cls.label = ObjectClassification::UNKNOWN;
    cls.probability = 1.0F;
    obj.classification.push_back(cls);

    obj.kinematics.pose_with_covariance.pose.position.x = est.cx;
    obj.kinematics.pose_with_covariance.pose.position.y = est.cy;
    obj.kinematics.pose_with_covariance.pose.position.z = est.cz;
    obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
    obj.kinematics.orientation_availability =
        autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
    // 6x6 row-major [x y z roll pitch yaw]: honest cm-scale position noise,
    // huge angular variance (orientation is UNAVAILABLE). See the
    // position_stddev comment in the constructor for why this must be set.
    auto& cov = obj.kinematics.pose_with_covariance.covariance;
    const double var = position_stddev * position_stddev;
    cov[0] = var;    // x-x
    cov[7] = var;    // y-y
    cov[14] = var;   // z-z
    cov[21] = 1e3;   // roll
    cov[28] = 1e3;   // pitch
    cov[35] = 1e3;   // yaw
    obj.kinematics.has_position_covariance = true;
    obj.kinematics.has_twist = false;
    obj.kinematics.has_twist_covariance = false;

    obj.shape.type = Shape::POLYGON;
    obj.shape.dimensions.z = est.height;
    obj.shape.footprint.points.reserve(est.footprint.size());
    for (const auto& v : est.footprint) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(v.x);
      p.y = static_cast<float>(v.y);
      p.z = 0.0F;
      obj.shape.footprint.points.push_back(p);
    }
    return obj;
  }

  // Same observability rule as the rest of the stack: every silent discard
  // path gets a counter and a throttled warning.
  void ReportStats() {
    if (degenerate_clusters_ != last_degenerate_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%ld cluster(s) dropped as degenerate (collinear "
                           "or thinner than min_object_height)",
                           degenerate_clusters_);
      last_degenerate_ = degenerate_clusters_;
    }
    // INFO, not WARN: on any indoor run dropping walls is the gate doing its
    // job continuously, but the count must stay observable — a person
    // brushing a wall can merge into its cluster and vanish here.
    if (hull3d_fallbacks_ != last_hull3d_fallbacks_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "%ld cluster(s) fell back to the footprint prism "
                           "(qhull could not hull the padded points)",
                           hull3d_fallbacks_);
      last_hull3d_fallbacks_ = hull3d_fallbacks_;
    }
    if (wall_clusters_ != last_wall_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "%ld cluster(s) dropped as wall-like (long+thin "
                           "or extent > max_object_extent)",
                           wall_clusters_);
      last_wall_ = wall_clusters_;
    }
  }

  std::string world_frame_;
  double tf_timeout_ = 0.1;
  double max_range_ = 5.0;
  double z_min_ = 0.10;
  double z_max_ = 2.0;
  double voxel_leaf_ = 0.05;
  double cluster_tolerance_ = 0.30;
  int min_cluster_size_ = 5;
  int max_cluster_size_ = 20000;
  double position_stddev_ = 0.10;
  ShapeParams shape_params_;
  long tf_misses_ = 0;
  long degenerate_clusters_ = 0;
  long last_degenerate_ = 0;
  long wall_clusters_ = 0;
  long last_wall_ = 0;
  long hull3d_fallbacks_ = 0;
  long last_hull3d_fallbacks_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  rclcpp::Publisher<ConvexObjects>::SharedPtr convex_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace cloud_object_detector

RCLCPP_COMPONENTS_REGISTER_NODE(cloud_object_detector::CloudObjectDetectorNode)
