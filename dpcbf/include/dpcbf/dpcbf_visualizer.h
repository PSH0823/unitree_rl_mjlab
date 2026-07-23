#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"

namespace dpcbf {

class DpcbfVisualizer {
 public:
  DpcbfVisualizer();
  ~DpcbfVisualizer();
  DpcbfVisualizer(DpcbfVisualizer&&) noexcept;
  DpcbfVisualizer& operator=(DpcbfVisualizer&&) noexcept;
  DpcbfVisualizer(const DpcbfVisualizer&) = delete;
  DpcbfVisualizer& operator=(const DpcbfVisualizer&) = delete;

  void LoadConfig(const std::filesystem::path& path);
  void Start();
  void Stop();
  void Update(const RobotState& robot,
              const std::vector<ObstacleState>& all_obstacles,
              const SafetyFilterResult& filter_result);
  bool enabled() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dpcbf
