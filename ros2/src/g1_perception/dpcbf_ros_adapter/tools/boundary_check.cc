// Validation gate for the DPCBF overlay's recomputed constraint geometry.
//
// A recomputed boundary that is subtly wrong is the worst outcome, because it
// will be believed. So the overlay's shared math (dpcbf_boundary.h) is never
// trusted on its own: it is proven against the FROZEN library, on the inputs
// a real run actually produced.
//
// Two modes, deliberately separated -- they measure different things and
// blurring them into one tolerance would hide both:
//
//   math <capture.bin> <config.yaml>            [the CTest]
//     Replays every Filter() record through dpcbf::DpcbfSafetyFilter (the
//     oracle, which already publishes its own selected_obstacles with
//     boundary_vertex_x / boundary_curvature) AND through SelectAndEvaluate()
//     on the identical inputs. Asserts the selected obstacle SET, its ORDER,
//     and the constraint PARAMETERS agree. Since both see bit-identical
//     inputs, any disagreement is a bug in the recomputation, not sampling --
//     so the tolerance is round-off, not a tuned number. Fails loudly.
//
//   join <capture.bin> <overlay.jsonl> <config.yaml>   [a measurement]
//     Joins a LIVE overlay log against the same capture and prices what the
//     overlay's estimated inputs cost. The overlay sees /obstacles_safe and
//     /odom at 10 Hz and must differentiate /odom's pose for the body
//     velocity (the bridge publishes no twist), while the seam has exact
//     state at 1 kHz. This is estimation error, a property of the run, not a
//     defect -- it is reported, never asserted.
//
// Join rule (mode `join`): NEAREST-PRECEDING capture tick. For an overlay
// record stamped t_safe, take the last capture record with t <= t_safe. No
// interpolation: the capture's obstacle set is piecewise-constant between
// perception frames, so interpolating it would invent states the filter never
// saw. Records with no preceding tick, or a gap > join_max_gap, are dropped
// and counted.
//
// Built from simulate/CMakeLists.txt (it needs the dpcbf static libs); the
// source lives in the adapter package to respect the outside-ros2/ file
// budget -- the same arrangement as t1_replay.cc and ab_eval.cc.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/dpcbf_boundary.h"
#include "dpcbf_ros_adapter/dpcbf_boundary_config.h"
#include "dpcbf_ros_adapter/filter_io_log.h"

namespace dra = dpcbf_ros_adapter;

namespace {

// Round-off budget, not a tuned threshold. SelectAndEvaluate and
// EvaluateConstraint perform the same operations in the same order on the
// same doubles, so exact equality is the expected outcome; this leaves room
// only for the compiler reassociating within a single expression. If a run
// needs this loosened to pass, the recomputation is wrong -- fix it, do not
// widen the tolerance until a wrong answer fits.
constexpr double kAbsTol = 1.0e-12;
constexpr double kRelTol = 1.0e-12;

bool Close(double a, double b) {
  if (a == b) return true;
  const double diff = std::abs(a - b);
  return diff <= kAbsTol ||
         diff <= kRelTol * std::max(std::abs(a), std::abs(b));
}

struct Stat {
  double max = 0.0;
  double sum = 0.0;
  std::uint64_t n = 0;
  void Add(double v) {
    max = std::max(max, v);
    sum += v;
    ++n;
  }
  double mean() const { return n ? sum / static_cast<double>(n) : 0.0; }
};

struct CaptureTick {
  double t = 0.0;
  dpcbf::RobotState robot;
  std::vector<dpcbf::ObstacleState> obstacles;
};

bool ReadHeader(std::FILE* f, dra::FilterIoHeader* h) {
  return std::fread(h, sizeof(*h), 1, f) == 1 &&
         std::memcmp(h->magic, dra::kFilterIoMagic, 8) == 0;
}

// Reads one record; returns false at EOF or on a truncated tail (captures end
// via SIGINT, so a cut final record is expected and not an error).
bool ReadRecord(std::FILE* f, CaptureTick* tick) {
  dra::FilterIoPrefix prefix{};
  if (std::fread(&prefix, sizeof(prefix), 1, f) != 1) return false;
  std::vector<dra::FilterIoObstacle> recorded(prefix.n_obstacles);
  if (prefix.n_obstacles &&
      std::fread(recorded.data(), sizeof(dra::FilterIoObstacle),
                 recorded.size(), f) != recorded.size()) {
    return false;
  }
  dra::FilterIoSuffix suffix{};
  if (std::fread(&suffix, sizeof(suffix), 1, f) != 1) return false;

  tick->t = prefix.t;
  tick->robot = {prefix.robot[0], prefix.robot[1], prefix.robot[2],
                 prefix.robot[3], prefix.robot[4]};
  tick->obstacles.clear();
  tick->obstacles.reserve(recorded.size());
  for (const auto& r : recorded) {
    tick->obstacles.push_back(
        {r.x, r.y, r.radius, r.velocity_x, r.velocity_y, r.id});
  }
  return true;
}

int RunMath(const char* capture_path, const char* config_path,
            std::uint64_t stride) {
  std::FILE* f = std::fopen(capture_path, "rb");
  if (!f) {
    // 77 = CTest SKIP_RETURN_CODE, matching t1_oracle_equivalence: the
    // capture is a generated fixture, absent on a bare clone.
    std::fprintf(stderr, "BOUNDARY SKIP: no capture at %s\n", capture_path);
    return 77;
  }
  dra::FilterIoHeader header{};
  if (!ReadHeader(f, &header)) {
    std::fprintf(stderr, "bad capture header\n");
    std::fclose(f);
    return 2;
  }

  dpcbf::DpcbfSafetyFilter filter;
  filter.LoadConfig(config_path);
  filter.Initialize(header.timestep);
  const dra::BoundaryParams params = dra::LoadBoundaryParams(config_path);

  CaptureTick tick;
  std::uint64_t records = 0, checked = 0, selected_total = 0;
  Stat d_vertex, d_curv, d_h, d_ecbf;
  const dpcbf::VelocityCommand zero_command;

  while (ReadRecord(f, &tick)) {
    const std::uint64_t index = records++;
    if (stride > 1 && (index % stride) != 0) continue;

    // Oracle: the frozen filter's own selection + boundary coefficients.
    const auto truth = filter.Filter(tick.robot, zero_command, tick.obstacles);
    // Under test: the overlay's shared recomputation, same inputs.
    const auto got = dra::SelectAndEvaluate(params, tick.robot, tick.obstacles);

    auto fail = [&](const std::string& what) {
      std::fprintf(stderr,
                   "BOUNDARY FAIL at record %" PRIu64 " (t=%.6f): %s\n",
                   index, tick.t, what.c_str());
      std::fclose(f);
      return 1;
    };

    if (got.size() != truth.selected_obstacles.size()) {
      return fail("selected count " + std::to_string(got.size()) + " vs " +
                  std::to_string(truth.selected_obstacles.size()));
    }
    for (std::size_t i = 0; i < got.size(); ++i) {
      const auto& g = got[i];
      const auto& w = truth.selected_obstacles[i];
      if (g.obstacle.id != w.obstacle.id) {
        return fail("selected[" + std::to_string(i) + "] id " +
                    std::to_string(g.obstacle.id) + " vs " +
                    std::to_string(w.obstacle.id));
      }
      if (!Close(g.boundary_vertex_x, w.boundary_vertex_x)) {
        return fail("selected[" + std::to_string(i) + "] vertex_x " +
                    std::to_string(g.boundary_vertex_x) + " vs " +
                    std::to_string(w.boundary_vertex_x));
      }
      if (!Close(g.boundary_curvature, w.boundary_curvature)) {
        return fail("selected[" + std::to_string(i) + "] curvature " +
                    std::to_string(g.boundary_curvature) + " vs " +
                    std::to_string(w.boundary_curvature));
      }
      if (!Close(g.h, w.constraint.h)) {
        return fail("selected[" + std::to_string(i) + "] h " +
                    std::to_string(g.h) + " vs " +
                    std::to_string(w.constraint.h));
      }
      if (!Close(g.distance, w.distance)) {
        return fail("selected[" + std::to_string(i) + "] distance");
      }
      if (!Close(g.relative_velocity_los[0], w.relative_velocity_los[0]) ||
          !Close(g.relative_velocity_los[1], w.relative_velocity_los[1])) {
        return fail("selected[" + std::to_string(i) + "] LoS relative velocity");
      }
      d_vertex.Add(std::abs(g.boundary_vertex_x - w.boundary_vertex_x));
      d_curv.Add(std::abs(g.boundary_curvature - w.boundary_curvature));
      d_h.Add(std::abs(g.h - w.constraint.h));

      // The eCBF family the shipped config also runs: its zero set is world
      // geometry, so verify it against the frozen evaluator too.
      const auto ecbf = filter.EvaluateEcbfConstraint(tick.robot, w.obstacle);
      if (!Close(g.ecbf_h, ecbf.distance_barrier)) {
        return fail("selected[" + std::to_string(i) + "] eCBF barrier " +
                    std::to_string(g.ecbf_h) + " vs " +
                    std::to_string(ecbf.distance_barrier));
      }
      d_ecbf.Add(std::abs(g.ecbf_h - ecbf.distance_barrier));
      ++selected_total;
    }
    ++checked;
  }
  std::fclose(f);

  if (checked == 0) {
    std::fprintf(stderr, "BOUNDARY FAIL: capture holds zero usable records\n");
    return 1;
  }
  std::printf(
      "BOUNDARY PASS: %" PRIu64 " ticks checked (of %" PRIu64
      " records, stride %" PRIu64 "), %" PRIu64 " selected-obstacle rows\n",
      checked, records, stride, selected_total);
  std::printf("  selected set + order: exact on every tick\n");
  std::printf("  max |d vertex_x|  = %.3g m/s   (tol %.0e)\n", d_vertex.max,
              kAbsTol);
  std::printf("  max |d curvature| = %.3g s/m   (tol %.0e)\n", d_curv.max,
              kAbsTol);
  std::printf("  max |d h|         = %.3g       (tol %.0e)\n", d_h.max,
              kAbsTol);
  std::printf("  max |d eCBF h|    = %.3g m^2   (tol %.0e)\n", d_ecbf.max,
              kAbsTol);
  return 0;
}

// --- join mode ------------------------------------------------------------

struct OverlayRecord {
  double t_safe = 0.0;
  double robot[5] = {0, 0, 0, 0, 0};
  std::vector<int> ids;
  std::vector<double> vertex, curv;
};

bool ParseOverlayLine(const char* line, OverlayRecord* rec) {
  // Deliberately a minimal scan rather than a JSON dependency: the writer is
  // this repo's own overlay node and the schema is fixed.
  const char* p = std::strstr(line, "\"t_safe\":");
  if (!p || std::sscanf(p + 9, "%lf", &rec->t_safe) != 1) return false;
  p = std::strstr(line, "\"robot\":[");
  if (!p || std::sscanf(p + 9, "%lf,%lf,%lf,%lf,%lf", &rec->robot[0],
                        &rec->robot[1], &rec->robot[2], &rec->robot[3],
                        &rec->robot[4]) != 5) {
    return false;
  }
  rec->ids.clear();
  rec->vertex.clear();
  rec->curv.clear();
  for (p = std::strstr(line, "\"id\":"); p; p = std::strstr(p + 1, "\"id\":")) {
    int id = 0;
    double vertex = 0.0, curvature = 0.0;
    if (std::sscanf(p + 5, "%d", &id) != 1) continue;
    const char* v = std::strstr(p, "\"vertex\":");
    const char* c = std::strstr(p, "\"curv\":");
    if (!v || !c || std::sscanf(v + 9, "%lf", &vertex) != 1 ||
        std::sscanf(c + 7, "%lf", &curvature) != 1) {
      continue;
    }
    rec->ids.push_back(id);
    rec->vertex.push_back(vertex);
    rec->curv.push_back(curvature);
  }
  return true;
}

int RunJoin(const char* capture_path, const char* overlay_path,
            const char* config_path, double join_max_gap) {
  std::FILE* cf = std::fopen(capture_path, "rb");
  std::FILE* of = std::fopen(overlay_path, "r");
  if (!cf || !of) {
    std::fprintf(stderr, "join: cannot open %s or %s\n", capture_path,
                 overlay_path);
    if (cf) std::fclose(cf);
    if (of) std::fclose(of);
    return 2;
  }
  dra::FilterIoHeader header{};
  if (!ReadHeader(cf, &header)) {
    std::fprintf(stderr, "bad capture header\n");
    std::fclose(cf);
    std::fclose(of);
    return 2;
  }
  const dra::BoundaryParams params = dra::LoadBoundaryParams(config_path);

  std::vector<OverlayRecord> overlay;
  char line[1 << 16];
  while (std::fgets(line, sizeof(line), of)) {
    OverlayRecord rec;
    if (ParseOverlayLine(line, &rec)) overlay.push_back(std::move(rec));
  }
  std::fclose(of);
  if (overlay.empty()) {
    std::fprintf(stderr, "join: overlay log holds no parsable records\n");
    std::fclose(cf);
    return 2;
  }
  std::sort(overlay.begin(), overlay.end(),
            [](const OverlayRecord& a, const OverlayRecord& b) {
              return a.t_safe < b.t_safe;
            });

  // Single forward pass: hold the last capture tick with t <= overlay t_safe.
  CaptureTick tick, held;
  bool have_held = false;
  std::size_t next = 0;
  std::uint64_t joined = 0, dropped_gap = 0, dropped_nopre = 0;
  std::uint64_t set_equal = 0;
  Stat d_speed, d_vertex, d_curv, jaccard_miss;

  while (next < overlay.size() && ReadRecord(cf, &tick)) {
    while (next < overlay.size() && overlay[next].t_safe < tick.t) {
      const OverlayRecord& rec = overlay[next++];
      if (!have_held) {
        ++dropped_nopre;
        continue;
      }
      if (rec.t_safe - held.t > join_max_gap) {
        ++dropped_gap;
        continue;
      }
      const auto truth = dra::SelectAndEvaluate(params, held.robot,
                                                held.obstacles);
      ++joined;
      d_speed.Add(std::hypot(rec.robot[3] - held.robot.sagittal_velocity,
                             rec.robot[4] - held.robot.lateral_velocity));

      std::vector<int> want;
      want.reserve(truth.size());
      for (const auto& b : truth) want.push_back(b.obstacle.id);
      std::vector<int> got = rec.ids;
      if (want == got) ++set_equal;
      std::vector<int> ws = want, gs = got;
      std::sort(ws.begin(), ws.end());
      std::sort(gs.begin(), gs.end());
      std::vector<int> common;
      std::set_intersection(ws.begin(), ws.end(), gs.begin(), gs.end(),
                            std::back_inserter(common));
      const std::size_t uni = ws.size() + gs.size() - common.size();
      jaccard_miss.Add(uni ? 1.0 - static_cast<double>(common.size()) /
                                       static_cast<double>(uni)
                           : 0.0);
      for (std::size_t i = 0; i < got.size(); ++i) {
        for (const auto& b : truth) {
          if (b.obstacle.id != got[i]) continue;
          d_vertex.Add(std::abs(rec.vertex[i] - b.boundary_vertex_x));
          d_curv.Add(std::abs(rec.curv[i] - b.boundary_curvature));
          break;
        }
      }
    }
    held = tick;
    have_held = true;
  }
  std::fclose(cf);

  std::printf("BOUNDARY JOIN (measurement, not a gate)\n");
  std::printf("  join rule: nearest-preceding capture tick, max gap %.3f s\n",
              join_max_gap);
  std::printf("  overlay records %zu, joined %" PRIu64
              ", dropped %" PRIu64 " (gap) + %" PRIu64 " (no preceding tick)\n",
              overlay.size(), joined, dropped_gap, dropped_nopre);
  if (joined == 0) {
    std::printf("  nothing joined -- overlay and capture do not overlap\n");
    return 0;
  }
  std::printf("  selected set identical on %" PRIu64 "/%" PRIu64
              " ticks (%.1f%%)\n",
              set_equal, joined,
              100.0 * static_cast<double>(set_equal) /
                  static_cast<double>(joined));
  std::printf("  set disagreement (Jaccard): mean %.4f  max %.4f\n",
              jaccard_miss.mean(), jaccard_miss.max);
  std::printf("  |d body speed| (differentiated vs exact): mean %.4f  "
              "max %.4f m/s\n",
              d_speed.mean(), d_speed.max);
  std::printf("  |d vertex_x|: mean %.4g  max %.4g m/s\n", d_vertex.mean(),
              d_vertex.max);
  std::printf("  |d curvature|: mean %.4g  max %.4g s/m\n", d_curv.mean(),
              d_curv.max);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "math") == 0 && argc >= 4) {
    const std::uint64_t stride =
        argc >= 5 ? std::strtoull(argv[4], nullptr, 10) : 1;
    return RunMath(argv[2], argv[3], stride ? stride : 1);
  }
  if (argc >= 2 && std::strcmp(argv[1], "join") == 0 && argc >= 5) {
    const double gap = argc >= 6 ? std::strtod(argv[5], nullptr) : 0.15;
    return RunJoin(argv[2], argv[3], argv[4], gap);
  }
  std::fprintf(stderr,
               "usage: %s math <capture.bin> <config.yaml> [stride]\n"
               "       %s join <capture.bin> <overlay.jsonl> <config.yaml> "
               "[max_gap_s]\n",
               argv[0], argv[0]);
  return 2;
}
