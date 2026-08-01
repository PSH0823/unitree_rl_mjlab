// Gate T1 (oracle equivalence, §16.2): replays a baseline Filter-I/O capture
// through the REFACTORED seam — ObstacleSource in kOracle mode + the shared
// dpcbf_seam.h mapping + a fresh DpcbfSafetyFilter — and demands byte
// identity on every Filter() input and output.
//
// The capture comes from the pre-refactor binary (scratch-instrumented
// main.cc at the pre-seam-refactor revision) driven by the scripted command
// profile on the seeded obstacle field. Nondeterministic wall-clock pacing
// makes two LIVE runs incomparable; replaying the recorded input sequence
// through the refactored acquisition path is exactly H-10's technique.
//
// Usage: t1_replay <capture.bin> <dpcbf_config.yaml>
// Exit 0 = byte-identical; nonzero prints the first mismatching record.
//
// Built from simulate/CMakeLists.txt (needs the dpcbf static libs); source
// lives in the adapter package to respect the Phase-4 outside-ros2/ budget.

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf_ros_adapter/dpcbf_seam.h"
#include "dpcbf_ros_adapter/filter_io_log.h"
#include "dpcbf_ros_adapter/obstacle_source.h"

namespace dra = dpcbf_ros_adapter;

namespace {

bool BitEq(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}
bool BitEq(float a, float b) {
  return std::memcmp(&a, &b, sizeof(float)) == 0;
}

struct Mismatch {
  std::string what;
};

bool CompareObstacles(const std::vector<dpcbf::ObstacleState>& got,
                      const std::vector<dra::FilterIoObstacle>& want,
                      std::string* what) {
  if (got.size() != want.size()) {
    *what = "obstacle count " + std::to_string(got.size()) + " vs " +
            std::to_string(want.size());
    return false;
  }
  for (std::size_t i = 0; i < got.size(); ++i) {
    const auto& g = got[i];
    const auto& w = want[i];
    if (!BitEq(g.x, w.x) || !BitEq(g.y, w.y) || !BitEq(g.radius, w.radius) ||
        !BitEq(g.velocity_x, w.velocity_x) ||
        !BitEq(g.velocity_y, w.velocity_y) || g.id != w.id) {
      *what = "obstacle[" + std::to_string(i) + "] differs";
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <capture.bin> <dpcbf_config.yaml>\n",
                 argv[0]);
    return 2;
  }
  std::FILE* f = std::fopen(argv[1], "rb");
  if (!f) {
    // 77 = CTest SKIP_RETURN_CODE: the baseline capture is a generated
    // fixture (gitignored, md5 in §21) — skip cleanly on bare machines.
    std::fprintf(stderr, "T1 SKIP: no capture at %s\n", argv[1]);
    return 77;
  }
  dra::FilterIoHeader header{};
  if (std::fread(&header, sizeof(header), 1, f) != 1 ||
      std::memcmp(header.magic, dra::kFilterIoMagic, 8) != 0) {
    std::fprintf(stderr, "bad capture header\n");
    return 2;
  }

  dpcbf::DpcbfSafetyFilter filter;
  filter.LoadConfig(argv[2]);
  filter.Initialize(header.timestep);
  const auto limits = dra::SeamLimits::FromFilter(filter);

  // The refactored acquisition path under test: kOracle pass-through, fully
  // offline (no ROS graph in the loop — exactly the §10.4 kOracle contract).
  std::vector<dpcbf::ObstacleState> current;
  dra::ObstacleSource::Config cfg;
  cfg.mode = dra::ObstacleSource::Mode::kOracle;
  cfg.enable_ros = false;
  cfg.oracle = [&current] { return current; };
  dra::ObstacleSource source(cfg);

  std::uint64_t records = 0;
  std::vector<dra::FilterIoObstacle> recorded;
  for (;;) {
    dra::FilterIoPrefix prefix{};
    const std::size_t got = std::fread(&prefix, sizeof(prefix), 1, f);
    if (got != 1) break;  // EOF
    recorded.resize(prefix.n_obstacles);
    if (prefix.n_obstacles &&
        std::fread(recorded.data(), sizeof(dra::FilterIoObstacle),
                   recorded.size(), f) != recorded.size()) {
      // SIGINT-terminated captures may cut mid-record; the file honestly
      // ends here (count + md5 recorded alongside the fixture).
      std::fprintf(stderr, "note: capture tail truncated at record %" PRIu64
                   " — comparing the complete records only\n", records);
      break;
    }
    dra::FilterIoSuffix suffix{};
    if (std::fread(&suffix, sizeof(suffix), 1, f) != 1) {
      std::fprintf(stderr, "note: capture tail truncated at record %" PRIu64
                   " — comparing the complete records only\n", records);
      break;
    }

    current.clear();
    current.reserve(recorded.size());
    for (const auto& r : recorded) {
      current.push_back(
          {r.x, r.y, r.radius, r.velocity_x, r.velocity_y, r.id});
    }

    auto fail = [&](const std::string& what) {
      std::fprintf(stderr,
                   "T1 FAIL at record %" PRIu64 " (t=%.6f): %s\n",
                   records, prefix.t, what.c_str());
      return 1;
    };

    // 1. Acquisition: the kOracle path must be a bit-exact pass-through.
    auto snap = source.GetObstacles(prefix.t);
    std::string what;
    if (!CompareObstacles(snap.obstacles, recorded, &what)) return fail(what);
    if (!snap.fresh || snap.command_scale != 1.0 || snap.age_s != 0.0) {
      return fail("oracle snapshot not fresh/scale-1/age-0");
    }

    // 2. Command mapping: identical desired command from the same axes.
    const auto desired =
        dra::AxesToDesired(prefix.lx, prefix.ly, prefix.rx, limits);
    if (!BitEq(desired.sagittal, prefix.desired[0]) ||
        !BitEq(desired.lateral, prefix.desired[1]) ||
        !BitEq(desired.yaw_rate, prefix.desired[2])) {
      return fail("desired command differs");
    }
    const auto scaled = dra::ScaleDesired(desired, snap.command_scale);

    // 3. Filter itself (byte-identical given the identical input sequence —
    //    also proves dpcbf determinism end to end).
    const dpcbf::RobotState robot{prefix.robot[0], prefix.robot[1],
                                  prefix.robot[2], prefix.robot[3],
                                  prefix.robot[4]};
    const auto result = filter.Filter(robot, scaled, snap.obstacles);
    if (!BitEq(result.command.sagittal, suffix.out_command[0]) ||
        !BitEq(result.command.lateral, suffix.out_command[1]) ||
        !BitEq(result.command.yaw_rate, suffix.out_command[2])) {
      return fail("output command differs");
    }
    for (int i = 0; i < 3; ++i) {
      if (!BitEq(result.acceleration[i], suffix.acceleration[i])) {
        return fail("acceleration differs");
      }
    }
    if (result.active_constraints != suffix.active_constraints ||
        result.active_dpcbf_constraints != suffix.active_dpcbf_constraints ||
        result.active_ecbf_constraints != suffix.active_ecbf_constraints ||
        static_cast<std::uint8_t>(result.solved) != suffix.solved) {
      return fail("constraint/solved bookkeeping differs");
    }

    // 4. Axes returned to the joystick.
    const auto axes = dra::CommandToAxes(result.command, limits);
    for (int i = 0; i < 3; ++i) {
      if (!BitEq(axes[i], suffix.out_axes[i])) {
        return fail("output axes differ");
      }
    }
    ++records;
  }
  std::fclose(f);
  if (records == 0) {
    std::fprintf(stderr, "T1 FAIL: capture holds zero records\n");
    return 1;
  }
  std::printf("T1 PASS: %" PRIu64
              " Filter() calls byte-identical (timestep %.6g)\n",
              records, header.timestep);
  return 0;
}
