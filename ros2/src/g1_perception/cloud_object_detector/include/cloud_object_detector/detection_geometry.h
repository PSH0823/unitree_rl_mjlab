// Pure geometry for the 3D detection front-end: cluster points → convex
// footprint + height, i.e. the autoware_perception_msgs Shape POLYGON
// convention (footprint polygon in the object frame, dimensions.z = height,
// pose at the footprint centroid, pose.z at mid-height). Node-free and
// PCL-free on purpose, mirroring safety_obstacle_filter/gating.h: everything
// subtle enough to get wrong lives here, under gtest, and the node stays a
// thin ROS wrapper.
#ifndef CLOUD_OBJECT_DETECTOR_DETECTION_GEOMETRY_H_
#define CLOUD_OBJECT_DETECTOR_DETECTION_GEOMETRY_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace cloud_object_detector {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct ShapeParams {
  // A cluster thinner than this in z is scan-line debris (a floor ridge the
  // ground band missed, a table edge), not an obstacle the tracker should
  // spawn on.
  double min_height = 0.05;  // [m]
  // Hull vertex cap. The tracker's polygon association/IoU cost scales with
  // vertex count and a Mid-360 cluster hull rarely exceeds ~20 vertices;
  // decimation beyond the cap trades sub-centimetre footprint detail for
  // bounded per-frame cost. 0 disables.
  std::size_t max_hull_vertices = 16;
  // Wall gate. Walls and other room structure read as LONG AND THIN in the
  // footprint: a cluster is dropped when its max extent exceeds
  // wall_min_length while its rotating-calipers width stays under
  // wall_max_thickness — a person (or two abreast) is never both. Anything
  // longer than max_object_extent is dropped regardless of thickness (an
  // L-shaped wall corner hulls into a triangle that is not thin). Zeros
  // disable the respective rule.
  double wall_min_length = 1.0;     // [m]
  double wall_max_thickness = 0.35;  // [m]
  double max_object_extent = 3.0;    // [m]
  // Depth completion. A single-viewpoint lidar sees only the sensor-facing
  // surface, so a solid object clusters into a thin arc and its hull has
  // near-zero thickness. Two stages bring every footprint up to this
  // thickness: first AWAY from the sensor (the occluded volume behind the
  // visible face is where the unseen bulk is), then — if still thin, i.e.
  // the cluster lies along the ray — symmetrically along its own thin axis.
  // 0 disables. Runs after the wall gate (walls must be judged on their
  // true, un-padded thinness).
  double min_thickness = 0.25;  // [m]
};

struct ShapeEstimate {
  bool valid = false;
  bool is_wall = false;  // rejected by the wall gate (valid stays false)
  // CCW convex footprint, relative to (cx, cy); z-free by construction.
  std::vector<Point2> footprint;
  double cx = 0.0;  // centroid of the cluster's xy projection [m, world]
  double cy = 0.0;
  double cz = 0.0;  // mid-height: (z_min + z_max) / 2 [m, world]
  double height = 0.0;  // z_max - z_min [m]
  // The cluster after depth completion (original points plus their
  // extruded copies), world frame — the input for the 3D convex hull, so
  // the polytope and the 2.5D footprint describe the same padded body.
  std::vector<std::array<double, 3>> padded_points;
};

struct HullMetrics {
  double max_extent = 0.0;  // max pairwise vertex distance [m]
  double min_width = 0.0;   // rotating-calipers width: the thinnest slab
                            // between two parallel lines containing the hull
  double min_width_nx = 0.0;  // unit normal of that slab (the thin axis)
  double min_width_ny = 0.0;
};

// O(n^2) over vertices / n edges — fine at <= max_hull_vertices + margin.
inline HullMetrics MeasureHull(const std::vector<Point2>& hull) {
  HullMetrics m;
  const std::size_t n = hull.size();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const double d = std::hypot(hull[j].x - hull[i].x,
                                  hull[j].y - hull[i].y);
      m.max_extent = std::max(m.max_extent, d);
    }
  }
  // For a convex polygon the minimum width is attained with one supporting
  // line flush against an edge, so scanning edges is exact.
  double min_w = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2& a = hull[i];
    const Point2& b = hull[(i + 1) % n];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (len <= 0.0) continue;
    const double nx = -(b.y - a.y) / len, ny = (b.x - a.x) / len;
    double w = 0.0;
    for (const auto& v : hull) {
      w = std::max(w, std::abs((v.x - a.x) * nx + (v.y - a.y) * ny));
    }
    if (w < min_w) {
      min_w = w;
      m.min_width_nx = nx;
      m.min_width_ny = ny;
    }
  }
  if (n >= 2 && min_w != std::numeric_limits<double>::max()) m.min_width = min_w;
  return m;
}

inline bool IsWallLike(const HullMetrics& m, const ShapeParams& p) {
  if (p.max_object_extent > 0.0 && m.max_extent > p.max_object_extent) {
    return true;
  }
  return p.wall_min_length > 0.0 && m.max_extent > p.wall_min_length &&
         m.min_width < p.wall_max_thickness;
}

// Monotone chain. Returns the hull in CCW order without a closing duplicate
// vertex. Degenerate inputs (<3 distinct points, or all collinear) return a
// hull with fewer than 3 vertices — callers must treat that as "no polygon".
inline std::vector<Point2> ConvexHull2D(std::vector<Point2> pts) {
  const auto lt = [](const Point2& a, const Point2& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  };
  const auto cross = [](const Point2& o, const Point2& a, const Point2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  std::sort(pts.begin(), pts.end(), lt);
  pts.erase(std::unique(pts.begin(), pts.end(),
                        [](const Point2& a, const Point2& b) {
                          return a.x == b.x && a.y == b.y;
                        }),
            pts.end());
  const std::size_t n = pts.size();
  if (n < 3) return pts;
  std::vector<Point2> hull(2 * n);
  std::size_t k = 0;
  for (std::size_t i = 0; i < n; ++i) {  // lower hull
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) --k;
    hull[k++] = pts[i];
  }
  for (std::size_t i = n - 1, t = k + 1; i-- > 0;) {  // upper hull
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) --k;
    hull[k++] = pts[i];
  }
  hull.resize(k - 1);  // last point == first point
  return hull;
}

// Uniform decimation to at most max_vertices, keeping hull convexity (a
// subset of a convex polygon's vertices is convex). No-op when already small
// enough or when max_vertices < 3 would make the polygon degenerate.
inline void DecimateHull(std::vector<Point2>* hull, std::size_t max_vertices) {
  if (max_vertices < 3 || hull->size() <= max_vertices) return;
  std::vector<Point2> out;
  out.reserve(max_vertices);
  const double step = static_cast<double>(hull->size()) /
                      static_cast<double>(max_vertices);
  for (std::size_t i = 0; i < max_vertices; ++i) {
    out.push_back((*hull)[static_cast<std::size_t>(i * step)]);
  }
  *hull = std::move(out);
}

// Area centroid of a convex CCW polygon (falls back to the vertex mean for
// a degenerate, near-zero-area polygon).
inline Point2 PolygonCentroid(const std::vector<Point2>& poly) {
  double a2 = 0.0, cx = 0.0, cy = 0.0, mx = 0.0, my = 0.0;
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2& p = poly[i];
    const Point2& q = poly[(i + 1) % n];
    const double cross = p.x * q.y - q.x * p.y;
    a2 += cross;
    cx += (p.x + q.x) * cross;
    cy += (p.y + q.y) * cross;
    mx += p.x;
    my += p.y;
  }
  if (std::abs(a2) < 1e-12) {
    return {mx / static_cast<double>(n), my / static_cast<double>(n)};
  }
  return {cx / (3.0 * a2), cy / (3.0 * a2)};
}

// Extrude the hull along `dir` (unit vector, sensor->object) by `depth`:
// union of the hull and its translated copy, re-hulled. The result contains
// the original front face and the assumed occluded volume behind it.
inline std::vector<Point2> ExtrudeHull(const std::vector<Point2>& hull,
                                       double dir_x, double dir_y,
                                       double depth) {
  std::vector<Point2> ext;
  ext.reserve(2 * hull.size());
  for (const auto& v : hull) {
    ext.push_back(v);
    ext.push_back({v.x + dir_x * depth, v.y + dir_y * depth});
  }
  return ConvexHull2D(std::move(ext));
}

// points: cluster in the world (tracking) frame, [x, y, z] each.
// sensor_x/sensor_y: lidar origin in the same frame, used for the
// depth-completion extrusion; pass NaN (or use the 2-arg overload) to skip it.
inline ShapeEstimate EstimateShape(
    const std::vector<std::array<double, 3>>& points, const ShapeParams& p,
    double sensor_x, double sensor_y) {
  ShapeEstimate est;
  if (points.size() < 3) return est;

  double sx = 0.0, sy = 0.0;
  double z_min = points.front()[2], z_max = points.front()[2];
  std::vector<Point2> xy;
  xy.reserve(points.size());
  for (const auto& q : points) {
    sx += q[0];
    sy += q[1];
    z_min = std::min(z_min, q[2]);
    z_max = std::max(z_max, q[2]);
    xy.push_back({q[0], q[1]});
  }
  if (z_max - z_min < p.min_height) return est;

  auto hull = ConvexHull2D(std::move(xy));
  if (hull.size() < 3) return est;  // collinear cluster: no footprint area
  if (IsWallLike(MeasureHull(hull), p)) {
    est.is_wall = true;
    return est;
  }

  // Depth completion (see min_thickness), on the POINT SET so the 2.5D
  // footprint and the 3D hull come from the same padded body. Only after
  // the wall gate, so a wall face is judged thin and dropped, not padded.
  // Stage 1 — along the viewing ray: the occluded volume behind the
  //   visible face. Correct for a face seen head-on.
  // Stage 2 — along the hull's own thin axis, symmetric: a cluster lying
  //   ALONG the ray (an object edge, a wall fragment seen end-on) gains
  //   nothing from stage 1 and would stay a line. Nothing is known about
  //   which side its bulk is on, so it is padded both ways.
  est.padded_points = points;
  auto extrude_points = [&](double ux, double uy, double d, bool both) {
    const std::size_t n0 = est.padded_points.size();
    est.padded_points.reserve(n0 * (both ? 3 : 2));
    for (std::size_t i = 0; i < n0; ++i) {
      // By value: push_back may reallocate and a reference into the vector
      // would dangle (it did — copies came out with z = 0).
      const std::array<double, 3> q = est.padded_points[i];
      est.padded_points.push_back({q[0] + ux * d, q[1] + uy * d, q[2]});
      if (both) est.padded_points.push_back({q[0] - ux * d, q[1] - uy * d, q[2]});
    }
  };
  if (p.min_thickness > 0.0) {
    HullMetrics m = MeasureHull(hull);
    if (m.min_width < p.min_thickness && std::isfinite(sensor_x) &&
        std::isfinite(sensor_y)) {
      const double mean_x = sx / static_cast<double>(points.size());
      const double mean_y = sy / static_cast<double>(points.size());
      const double dx = mean_x - sensor_x, dy = mean_y - sensor_y;
      const double norm = std::hypot(dx, dy);
      if (norm > 1e-6) {
        const double d = p.min_thickness - m.min_width;
        extrude_points(dx / norm, dy / norm, d, false);
        hull = ExtrudeHull(hull, dx / norm, dy / norm, d);
        m = MeasureHull(hull);
      }
    }
    if (m.min_width < p.min_thickness) {
      const double d = 0.5 * (p.min_thickness - m.min_width);
      extrude_points(m.min_width_nx, m.min_width_ny, d, true);
      std::vector<Point2> both;
      both.reserve(3 * hull.size());
      for (const auto& v : hull) {
        both.push_back(v);
        both.push_back({v.x + m.min_width_nx * d, v.y + m.min_width_ny * d});
        both.push_back({v.x - m.min_width_nx * d, v.y - m.min_width_ny * d});
      }
      hull = ConvexHull2D(std::move(both));
    }
  }
  DecimateHull(&hull, p.max_hull_vertices);

  // Pose at the footprint's AREA centroid, not the point centroid: raw
  // points sit on the visible surface, so their mean is biased toward the
  // sensor by roughly half the object depth. After extrusion the polygon
  // centroid approximates the occupancy center.
  const Point2 c = PolygonCentroid(hull);
  est.cx = c.x;
  est.cy = c.y;
  est.cz = 0.5 * (z_min + z_max);
  est.height = z_max - z_min;
  est.footprint.reserve(hull.size());
  for (const auto& v : hull) {
    est.footprint.push_back({v.x - est.cx, v.y - est.cy});
  }
  est.valid = true;
  return est;
}

inline ShapeEstimate EstimateShape(
    const std::vector<std::array<double, 3>>& points, const ShapeParams& p) {
  return EstimateShape(points, p, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
}

}  // namespace cloud_object_detector

#endif  // CLOUD_OBJECT_DETECTOR_DETECTION_GEOMETRY_H_
