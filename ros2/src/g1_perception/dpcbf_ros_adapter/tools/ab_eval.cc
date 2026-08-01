// Offline paired-filter A/B evaluation (§17.3, Phase-4 §0 fallback): two
// independent DpcbfSafetyFilter instances (each owns its OSQP state — safe
// per §2.3-4) consume the SAME robot state and scripted command profile at
// 1 kHz; the oracle arm eats exact ground-truth obstacles, the estimated arm
// eats the recorded /obstacles_safe stream through the adapter's
// Materialize (extrapolation + §10.3 staleness ladder + call-site command
// scaling) — i.e. exactly what the live kEstimated seam would feed.
//
// Inputs (binary streams share the FilterIoObstacle record layout):
//   --oracle    per-tick records {double t; u32 n; n×FilterIoObstacle}
//   --estimated frames {double stamp; u32 n; n×FilterIoObstacle}
//   --config    dpcbf_config.yaml
//   --profile   text lines "t lx ly rx" (piecewise-constant axes)
//   --timestep  Initialize() dt (default 0.001)
//   --latency   estimated frame delivery latency in s (default 0.002 —
//               Phase-3 measured cloud→tracked p95 1.7 ms)
//   --robot     "x y phi" static pose (default 0 0 0)
//   --out       CSV path
//
// Usage: ab_eval --oracle o.bin --estimated e.bin --config c.yaml \
//                --profile p.txt --out out.csv

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/dpcbf_seam.h"
#include "dpcbf_ros_adapter/filter_io_log.h"
#include "dpcbf_ros_adapter/obstacle_buffer.h"

namespace dra = dpcbf_ros_adapter;

namespace {

struct ObstacleRecord {
  double key = 0.0;  // t (oracle) or stamp (estimated)
  std::vector<dpcbf::ObstacleState> obstacles;
};

std::vector<ObstacleRecord> ReadStream(const std::string& path) {
  std::vector<ObstacleRecord> out;
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", path.c_str());
    std::exit(2);
  }
  for (;;) {
    double key;
    std::uint32_t n;
    if (std::fread(&key, sizeof(key), 1, f) != 1) break;
    if (std::fread(&n, sizeof(n), 1, f) != 1) break;
    ObstacleRecord rec;
    rec.key = key;
    rec.obstacles.reserve(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      dra::FilterIoObstacle o{};
      if (std::fread(&o, sizeof(o), 1, f) != 1) {
        std::fprintf(stderr, "truncated stream %s\n", path.c_str());
        std::exit(2);
      }
      rec.obstacles.push_back(
          {o.x, o.y, o.radius, o.velocity_x, o.velocity_y, o.id});
    }
    out.push_back(std::move(rec));
  }
  std::fclose(f);
  return out;
}

struct AxesProfile {
  struct Point {
    double t;
    float lx, ly, rx;
  };
  std::vector<Point> points;

  static AxesProfile Load(const std::string& path) {
    AxesProfile p;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      Point pt{};
      if (ss >> pt.t >> pt.lx >> pt.ly >> pt.rx) p.points.push_back(pt);
    }
    return p;
  }
  // Piecewise-constant hold of the last breakpoint at or before t.
  Point Sample(double t) const {
    Point cur{0.0, 0.f, 0.f, 0.f};
    for (const auto& pt : points) {
      if (pt.t > t) break;
      cur = pt;
    }
    return cur;
  }
};

const char* Arg(int argc, char** argv, const char* name,
                const char* fallback = nullptr) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
  }
  return fallback;
}

}  // namespace

int main(int argc, char** argv) {
  const char* oracle_path = Arg(argc, argv, "--oracle");
  const char* est_path = Arg(argc, argv, "--estimated");
  const char* config_path = Arg(argc, argv, "--config");
  const char* profile_path = Arg(argc, argv, "--profile");
  const char* out_path = Arg(argc, argv, "--out");
  if (!oracle_path || !est_path || !config_path || !profile_path ||
      !out_path) {
    std::fprintf(stderr,
                 "usage: ab_eval --oracle O --estimated E --config C "
                 "--profile P --out CSV [--timestep DT] [--latency L] "
                 "[--robot 'x y phi']\n");
    return 2;
  }
  const double timestep = std::atof(Arg(argc, argv, "--timestep", "0.001"));
  const double latency = std::atof(Arg(argc, argv, "--latency", "0.002"));
  dpcbf::RobotState robot;
  {
    std::istringstream ss(Arg(argc, argv, "--robot", "0 0 0"));
    ss >> robot.x >> robot.y >> robot.phi;
  }

  const auto oracle = ReadStream(oracle_path);
  const auto estimated = ReadStream(est_path);
  const auto profile = AxesProfile::Load(profile_path);
  if (oracle.empty()) {
    std::fprintf(stderr, "oracle stream is empty\n");
    return 2;
  }

  dpcbf::DpcbfSafetyFilter filter_oracle, filter_estimated;
  filter_oracle.LoadConfig(config_path);
  filter_estimated.LoadConfig(config_path);
  filter_oracle.Initialize(timestep);
  filter_estimated.Initialize(timestep);
  const auto limits = dra::SeamLimits::FromFilter(filter_oracle);
  const dra::StalenessPolicy policy;  // Appendix-A defaults

  std::FILE* out = std::fopen(out_path, "w");
  if (!out) {
    std::fprintf(stderr, "cannot open %s\n", out_path);
    return 2;
  }
  std::fprintf(out,
               "t,des_sag,des_lat,des_yaw,o_sag,o_lat,o_yaw,e_sag,e_lat,"
               "e_yaw,e_age,e_scale,e_state,o_active,e_active,o_solved,"
               "e_solved,o_nobs,e_nobs\n");

  std::size_t est_idx = 0;
  dra::ObstacleFrame est_frame;  // stamp<0 until the first delivered frame
  dra::MaterializedSnapshot est_snap;
  for (const auto& tick : oracle) {
    const double t = tick.key;
    // Deliver estimated frames whose stamp+latency has passed.
    while (est_idx < estimated.size() &&
           estimated[est_idx].key + latency <= t) {
      const auto& fr = estimated[est_idx];
      est_frame.stamp = fr.key;
      est_frame.count = std::min(fr.obstacles.size(), dra::kMaxObstacles);
      for (std::size_t i = 0; i < est_frame.count; ++i) {
        est_frame.obstacles[i] = fr.obstacles[i];
      }
      ++est_idx;
    }
    const auto axes = profile.Sample(t);
    const auto desired = dra::AxesToDesired(axes.lx, axes.ly, axes.rx, limits);

    const auto o_res = filter_oracle.Filter(robot, desired, tick.obstacles);

    dra::Materialize(est_frame, t, policy, &est_snap);
    const auto e_desired = dra::ScaleDesired(desired, est_snap.command_scale);
    const auto e_res =
        filter_estimated.Filter(robot, e_desired, est_snap.obstacles);

    std::fprintf(out,
                 "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,"
                 "%.4f,%u,%d,%d,%d,%d,%zu,%zu\n",
                 t, desired.sagittal, desired.lateral, desired.yaw_rate,
                 o_res.command.sagittal, o_res.command.lateral,
                 o_res.command.yaw_rate, e_res.command.sagittal,
                 e_res.command.lateral, e_res.command.yaw_rate,
                 std::isinf(est_snap.age_s) ? -1.0 : est_snap.age_s,
                 est_snap.command_scale,
                 static_cast<unsigned>(est_snap.state),
                 o_res.active_constraints, e_res.active_constraints,
                 o_res.solved ? 1 : 0, e_res.solved ? 1 : 0,
                 tick.obstacles.size(), est_snap.obstacles.size());
  }
  std::fclose(out);
  std::printf("ab_eval: %zu ticks, %zu estimated frames -> %s\n",
              oracle.size(), estimated.size(), out_path);
  return 0;
}
