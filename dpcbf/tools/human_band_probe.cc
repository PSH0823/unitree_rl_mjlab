// human_band_probe — what a `pointcloud_to_laserscan` height band actually
// recovers from a standing human.
//
// Motivation: DPCBF models obstacles as vertical cylinders. A cylinder gives the
// same circle at every height and every aspect angle; a person does not. This
// tool compiles the REAL obstacle field from dpcbf_config.yaml (so the body plan
// it measures is exactly the one the simulator spawns), isolates one human, and
// for each candidate [min_height, max_height] band emulates the two stages that
// decide what DPCBF is handed:
//
//   1. projection  — `pointcloud_to_laserscan` keeps the MINIMUM range per
//      azimuth bin over the band, so the 2D scan is the band's union silhouette.
//   2. extraction  — `obstacle_extractor` with `circles_from_visibles` fits a
//      line to the visible arc and takes the circumcircle of the equilateral
//      triangle on that chord: r_fit = chord / sqrt(3), centre pulled r_fit/2
//      toward the sensor (utilities/circle.h). It also splits a segment wherever
//      consecutive returns are more than `max_group_distance` apart — which is
//      how one person becomes two obstacles.
//
// The Mid360's own vertical FOV is applied first, because a band edge the sensor
// cannot reach at that range is inert. The sensor is mounted upside down
// (scene_g1.xml, roll = pi), so its native -7.21..+52.16 deg becomes +7.21 deg
// up to -52.16 deg down: reachable heights at horizontal distance d are
// [z_sensor - d*tan(52.16), z_sensor + d*tan(7.21)].
//
// Usage:
//   human_band_probe <scene.xml> <dpcbf_config.yaml> [--range 2.0] [--sensor-z 1.265]
//                    [--band lo:hi]...   (repeatable; a default set is used if omitted)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "dpcbf/dynamic_obstacles.h"

namespace {

// Mid360 elevation limits after the roll = pi mount (degrees, world frame).
constexpr double kElevUpDeg = 7.21;
constexpr double kElevDownDeg = 52.16;
// obstacle_detector.yaml: a wider gap than this starts a new segment.
constexpr double kMaxGroupDistance = 0.10;
// obstacle_detector.yaml: added to every published circle radius.
constexpr double kRadiusEnlargement = 0.17;

struct Band { double lo, hi; };

struct Fit {
  int returns = 0;
  int groups = 0;
  int occluded = 0;   // azimuth bins where something stood IN FRONT of the body
  double chord = 0.0;
  double r_fit = 0.0;
  double centre_err = 0.0;
  double max_gap = 0.0;
};

Fit Probe(const mjModel* m, mjData* d, int body_id, double range, double truth_radius,
          double band_lo, double band_hi) {
  constexpr int kAzimuthSamples = 1200;
  constexpr int kHeightSamples = 48;
  const double sx = -range, sy = 0.0;
  // Anything nearer than the body's own near face is genuinely in the way; a hit
  // beyond it is just background the azimuth fan overshot into.
  const double foreground = range - truth_radius;

  std::vector<double> hx, hy;
  Fit f;
  const double half_fan = std::atan2(0.8, range);
  for (int i = 0; i < kAzimuthSamples; ++i) {
    const double a = -half_fan + 2.0 * half_fan * i / (kAzimuthSamples - 1.0);
    double best = std::numeric_limits<double>::infinity();
    double bx = 0.0, by = 0.0;
    bool blocked = false;
    for (int k = 0; k < kHeightSamples; ++k) {
      const double z = band_lo + (band_hi - band_lo) * k / (kHeightSamples - 1.0);
      mjtNum pnt[3] = {sx, sy, z};
      mjtNum vec[3] = {std::cos(a), std::sin(a), 0.0};
      int geom_id = -1;
      const mjtNum dist = mj_ray(m, d, pnt, vec, nullptr, 1, -1, &geom_id);
      if (dist < 0 || geom_id < 0) continue;
      if (m->geom_bodyid[geom_id] != body_id) {
        if (dist < foreground) blocked = true;
        continue;
      }
      if (dist < best) {
        best = dist;
        bx = sx + dist * vec[0];
        by = sy + dist * vec[1];
      }
    }
    if (std::isfinite(best)) {
      hx.push_back(bx);
      hy.push_back(by);
    } else if (blocked) {
      ++f.occluded;
    }
  }

  f.returns = static_cast<int>(hx.size());
  if (f.returns < 2) return f;

  f.groups = 1;
  for (std::size_t i = 1; i < hx.size(); ++i) {
    const double gap = std::hypot(hx[i] - hx[i - 1], hy[i] - hy[i - 1]);
    f.max_gap = std::max(f.max_gap, gap);
    if (gap > kMaxGroupDistance) ++f.groups;
  }

  f.chord = std::hypot(hx.back() - hx.front(), hy.back() - hy.front());
  f.r_fit = f.chord / std::sqrt(3.0);
  const double mx = 0.5 * (hx.front() + hx.back());
  const double my = 0.5 * (hy.front() + hy.back());
  const double norm = std::hypot(mx - sx, my - sy);
  const double cx = mx + 0.5 * f.r_fit * (mx - sx) / norm;
  const double cy = my + 0.5 * f.r_fit * (my - sy) / norm;
  f.centre_err = std::hypot(cx, cy);   // the human sits at the origin
  return f;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s <scene.xml> <dpcbf_config.yaml> [--range M] "
                 "[--sensor-z M] [--band lo:hi]...\n",
                 argv[0]);
    return 2;
  }
  const char* scene_path = argv[1];
  const char* config_path = argv[2];
  double range = 2.0;
  double sensor_z = 1.265;   // pelvis 0.793 + torso 0.044 + mid360 0.428
  std::vector<Band> bands;
  for (int i = 3; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--range") && i + 1 < argc) {
      range = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--sensor-z") && i + 1 < argc) {
      sensor_z = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--band") && i + 1 < argc) {
      const std::string spec = argv[++i];
      const std::size_t colon = spec.find(':');
      if (colon == std::string::npos) {
        std::fprintf(stderr, "--band expects lo:hi, got '%s'\n", spec.c_str());
        return 2;
      }
      bands.push_back({std::atof(spec.substr(0, colon).c_str()),
                       std::atof(spec.substr(colon + 1).c_str())});
    } else {
      std::fprintf(stderr, "unknown argument '%s'\n", argv[i]);
      return 2;
    }
  }
  if (bands.empty()) {
    bands = {{0.15, 1.60},   // shipped
             {0.20, 0.70},   // legs only
             {0.75, 0.95},   // hips + hanging arms
             {1.00, 1.40},   // torso
             {1.42, 1.55},   // shoulders / neck
             {1.55, 1.75}};  // head only
  }

  dpcbf::DynamicObstacleManager manager;
  manager.LoadConfig(config_path);
  if (manager.shape() != dpcbf::ObstacleShape::kHuman) {
    std::fprintf(stderr,
                 "warning: dynamic_obstacles.shape is not 'human'; this probe is "
                 "only meaningful for the human body plan\n");
  }

  char load_error[1024] = "";
  mjSpec* spec = mj_parseXML(scene_path, nullptr, load_error, sizeof(load_error));
  if (!spec) {
    std::fprintf(stderr, "parse failed: %s\n", load_error);
    return 1;
  }
  manager.AddToSpec(spec);
  mjModel* model = mj_compile(spec, nullptr);
  if (!model) {
    std::fprintf(stderr, "compile failed: %s\n", mjs_getError(spec));
    mj_deleteSpec(spec);
    return 1;
  }
  mjData* data = mj_makeData(model);
  manager.BindModel(model, data);

  const std::vector<dpcbf::DynamicObstacle> obstacles = manager.Snapshot();
  if (obstacles.empty()) {
    std::fprintf(stderr, "no obstacles configured\n");
    return 1;
  }

  // Isolate obstacle 0 at the arena centre and park every other one far below
  // the floor, so the only thing the rays can hit is the body under test (the
  // static arena walls stay put; occluded azimuths are counted, not hidden).
  const int body_id = mj_name2id(model, mjOBJ_BODY, "dpcbf_obstacle_0");
  for (const dpcbf::DynamicObstacle& obstacle : obstacles) {
    if (obstacle.mocap_id < 0) continue;
    mjtNum* pos = data->mocap_pos + 3 * obstacle.mocap_id;
    pos[0] = 0.0;
    pos[1] = 0.0;
    pos[2] = (obstacle.mocap_id == model->body_mocapid[body_id]) ? 0.0 : -100.0;
  }

  const double height = obstacles[0].height;
  const double truth = obstacles[0].radius;
  const double z_top = sensor_z + range * std::tan(kElevUpDeg * mjPI / 180.0);
  const double z_bot = std::max(0.0, sensor_z - range * std::tan(kElevDownDeg * mjPI / 180.0));

  std::printf("standing human: H = %.3f m, DPCBF ground-truth radius = %.3f m\n", height, truth);
  std::printf("sensor: z = %.3f m, horizontal range = %.2f m -> reachable band [%.2f, %.2f] m\n",
              sensor_z, range, z_bot, z_top);
  std::printf("published radius = r_fit + radius_enlargement (%.2f m); a segment splits at a "
              "%.2f m gap\n\n", kRadiusEnlargement, kMaxGroupDistance);

  std::printf("  requested band   effective band   yaw   returns  chord   r_fit  published  "
              "|c_err|  max_gap  groups\n");
  const double yaws[] = {0.0, 30.0, 60.0, 90.0};
  const int mocap = model->body_mocapid[body_id];
  for (const Band& band : bands) {
    const double lo = std::max(band.lo, z_bot);
    const double hi = std::min(band.hi, z_top);
    if (hi <= lo) {
      std::printf("  %.2f - %.2f      OUT OF SENSOR FOV at this range\n", band.lo, band.hi);
      continue;
    }
    for (double yaw_deg : yaws) {
      const double yaw = yaw_deg * mjPI / 180.0;
      data->mocap_quat[4 * mocap + 0] = std::cos(0.5 * yaw);
      data->mocap_quat[4 * mocap + 1] = 0.0;
      data->mocap_quat[4 * mocap + 2] = 0.0;
      data->mocap_quat[4 * mocap + 3] = std::sin(0.5 * yaw);
      mj_forward(model, data);

      const Fit f = Probe(model, data, body_id, range, truth, lo, hi);
      std::printf("  %.2f - %.2f      %.2f - %.2f     %3.0f   %6d  %5.3f  %5.3f    %5.3f    "
                  "%5.3f    %5.3f  %5d%s\n",
                  band.lo, band.hi, lo, hi, yaw_deg, f.returns, f.chord, f.r_fit,
                  f.r_fit + kRadiusEnlargement, f.centre_err, f.max_gap, f.groups,
                  f.occluded > 0 ? "  (occluded bins)" : "");
    }
    std::printf("\n");
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  return 0;
}
