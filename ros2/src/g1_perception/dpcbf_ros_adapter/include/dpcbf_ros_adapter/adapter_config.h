#pragma once
// Loads ObstacleSource's tunables (topic + §10.3 staleness ladder) from
// config/dpcbf_ros_adapter.yaml — the file the ros2 tree has always recorded
// as the Appendix-A values.
//
// Until now that yaml was documentation only: main.cc filled
// ObstacleSource::Config with just {mode, oracle}, so the ladder came from
// the compiled-in defaults in obstacle_buffer.h, which merely happened to
// equal the yaml. This loader makes the yaml the control surface so the two
// cannot drift apart silently.
//
// Conventions mirror dpcbf_boundary_config.h: header-only, consumers pull
// yaml-cpp themselves (the adapter library gains no dependency), and every
// key is REQUIRED — a file the loader would half-read must not silently
// yield plausible-looking defaults.

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "dpcbf_ros_adapter/obstacle_source.h"
#include "dpcbf_ros_adapter/viz_publisher.h"

namespace dpcbf_ros_adapter {

// Fills `config->topic` and `config->staleness` from `path`. Mode, oracle
// and the diagnostics wiring stay the caller's business. Throws
// std::runtime_error on a missing file, missing key or out-of-range value.
inline void LoadAdapterConfig(const std::filesystem::path& path,
                              ObstacleSource::Config* config) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node node = root["dpcbf_ros_adapter"];
  if (!node) {
    throw std::runtime_error("missing 'dpcbf_ros_adapter' in " +
                             path.string());
  }
  auto required = [&path](const YAML::Node& parent, const char* key) {
    if (!parent[key]) {
      throw std::runtime_error(std::string("missing '") + key + "' in " +
                               path.string());
    }
    return parent[key];
  };

  const std::string topic = required(node, "topic").as<std::string>();
  StalenessPolicy staleness;
  staleness.max_age_s = required(node, "max_age").as<double>();
  staleness.fade_out_s = required(node, "fade_out").as<double>();
  staleness.hold_after_stale_s =
      required(node, "hold_after_stale").as<double>();

  // fade_out = 0 is legal (Classify/CommandScale hit the >= bound before the
  // division), so only genuinely nonsensical values are rejected here.
  if (topic.empty() || topic.front() != '/' || staleness.max_age_s <= 0.0 ||
      staleness.fade_out_s < 0.0 || staleness.hold_after_stale_s < 0.0) {
    throw std::runtime_error("invalid adapter parameter range in " +
                             path.string());
  }
  config->topic = topic;
  config->staleness = staleness;
}

// The plot bridge's control surface (same file, own section). Returns the
// section's `enabled` flag; `config` is filled either way. Same discipline
// as above: the section and every key in it are REQUIRED, so a yaml that
// predates the plot bridge fails loudly instead of silently running without
// visualization the operator thinks is on.
inline bool LoadVizBridgeConfig(const std::filesystem::path& path,
                                DpcbfVizPublisher::Config* config) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node node = root["plot_bridge"];
  if (!node) {
    throw std::runtime_error("missing 'plot_bridge' in " + path.string());
  }
  auto required = [&path](const YAML::Node& parent, const char* key) {
    if (!parent[key]) {
      throw std::runtime_error(std::string("missing '") + key + "' in " +
                               path.string());
    }
    return parent[key];
  };
  const bool enabled = required(node, "enabled").as<bool>();
  const std::string topic = required(node, "topic").as<std::string>();
  const double rate_hz = required(node, "rate_hz").as<double>();
  const std::string frame_id = required(node, "frame_id").as<std::string>();
  // 20–50 Hz is the design band; 1–100 the sanity range. Rates beyond the
  // control rate are meaningless (each tick publishes at most once).
  if (topic.empty() || topic.front() != '/' || rate_hz < 1.0 ||
      rate_hz > 100.0 || frame_id.empty()) {
    throw std::runtime_error("invalid plot_bridge parameter range in " +
                             path.string());
  }
  config->topic = topic;
  config->rate_hz = rate_hz;
  config->frame_id = frame_id;
  return enabled;
}

}  // namespace dpcbf_ros_adapter
