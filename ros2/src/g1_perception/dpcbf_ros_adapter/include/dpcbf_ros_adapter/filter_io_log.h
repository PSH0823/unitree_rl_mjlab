#pragma once
// Binary capture format for every Filter() call at the 1 kHz seam — the H-10
// technique behind gate T1 (byte-identical oracle equivalence) and the
// measurement instrument for T6 (staleness timeline at query resolution).
//
// The format is shared by three producers/consumers that must agree byte for
// byte: the pre-refactor baseline capture (scratch-patched main.cc), the
// refactored simulate seam (env UNITREE_DPCBF_FILTER_LOG=<path>), and the
// t1_replay / analysis tools. Packed little-endian structs, no padding.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"

namespace dpcbf_ros_adapter {

#pragma pack(push, 1)
struct FilterIoHeader {
  char magic[8];        // "T1CAP01\0"
  double timestep;      // m->opt.timestep passed to Initialize()
  std::uint32_t mode;   // 0 oracle / 1 shadow / 2 estimated / 99 baseline
  std::uint32_t reserved;
};

struct FilterIoObstacle {
  double x, y, radius, velocity_x, velocity_y;
  std::int32_t id;
};

struct FilterIoPrefix {
  double t;              // d->time at the query (sim time)
  float lx, ly, rx;      // input axes as delivered to the seam
  double robot[5];       // x, y, phi, sagittal_velocity, lateral_velocity
  double desired[3];     // AxisToCommand result BEFORE staleness scaling
  double command_scale;  // §10.3 scale applied (1.0 at baseline/oracle)
  double age_s;          // adapter age (0.0 at baseline/oracle)
  std::uint32_t staleness_state;  // StalenessState (0 at baseline/oracle)
  std::uint32_t n_obstacles;      // FilterIoObstacle records that follow
};

struct FilterIoSuffix {
  double out_command[3];  // filtered.command sagittal/lateral/yaw_rate
  double acceleration[3];
  std::int32_t active_constraints;
  std::int32_t active_dpcbf_constraints;
  std::int32_t active_ecbf_constraints;
  std::uint8_t solved;
  float out_axes[3];      // CommandToAxis result returned to the joystick
};
#pragma pack(pop)

static_assert(sizeof(FilterIoHeader) == 24, "packed layout");
static_assert(sizeof(FilterIoObstacle) == 44, "packed layout");
static_assert(sizeof(FilterIoPrefix) == 108, "packed layout");
static_assert(sizeof(FilterIoSuffix) == 73, "packed layout");

constexpr char kFilterIoMagic[8] = {'T', '1', 'C', 'A', 'P', '0', '1', '\0'};

class FilterIoWriter {
 public:
  // Opens `path` and writes the header. A default-constructed writer is
  // inert (enabled() false) so the seam can call Append unconditionally.
  FilterIoWriter() = default;
  FilterIoWriter(const std::string& path, double timestep,
                 std::uint32_t mode) {
    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) return;
    // Big stdio buffer: at 1 kHz with ~90 obstacles a record is ~4 KB; keep
    // fwrite off the syscall path as much as possible on the bridge thread.
    std::setvbuf(file_, nullptr, _IOFBF, 8u << 20);
    FilterIoHeader h{};
    std::memcpy(h.magic, kFilterIoMagic, sizeof(h.magic));
    h.timestep = timestep;
    h.mode = mode;
    std::fwrite(&h, sizeof(h), 1, file_);
  }
  ~FilterIoWriter() { Close(); }
  FilterIoWriter(const FilterIoWriter&) = delete;
  FilterIoWriter& operator=(const FilterIoWriter&) = delete;

  bool enabled() const { return file_ != nullptr; }

  void Append(const FilterIoPrefix& prefix,
              const std::vector<dpcbf::ObstacleState>& obstacles,
              const FilterIoSuffix& suffix) {
    if (!file_) return;
    FilterIoPrefix p = prefix;
    p.n_obstacles = static_cast<std::uint32_t>(obstacles.size());
    std::fwrite(&p, sizeof(p), 1, file_);
    for (const auto& o : obstacles) {
      const FilterIoObstacle rec{o.x, o.y, o.radius, o.velocity_x,
                                 o.velocity_y, o.id};
      std::fwrite(&rec, sizeof(rec), 1, file_);
    }
    std::fwrite(&suffix, sizeof(suffix), 1, file_);
    // Capture runs end via SIGINT (no clean shutdown path in simulate);
    // bound the unflushed tail to <=256 records. Not used in benchmark runs.
    if ((++appends_ & 0xffu) == 0u) std::fflush(file_);
  }

  void Close() {
    if (file_) {
      std::fclose(file_);
      file_ = nullptr;
    }
  }

 private:
  std::FILE* file_ = nullptr;
  std::uint64_t appends_ = 0;
};

}  // namespace dpcbf_ros_adapter
