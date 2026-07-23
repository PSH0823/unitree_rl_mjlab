#include "dpcbf/dpcbf_visualizer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace dpcbf {
namespace {

const std::array<cv::Scalar, 10> kActiveColors = {
    cv::Scalar(70, 130, 230), cv::Scalar(60, 180, 75),
    cv::Scalar(210, 120, 60), cv::Scalar(180, 80, 190),
    cv::Scalar(60, 190, 210), cv::Scalar(200, 150, 70),
    cv::Scalar(100, 100, 230), cv::Scalar(120, 200, 120),
    cv::Scalar(220, 120, 160), cv::Scalar(90, 170, 220)};

cv::Point WorldToPixel(double x, double y, const cv::Rect& area,
                       double min_x, double max_x,
                       double min_y, double max_y) {
  const double scale = std::min(area.width / (max_x - min_x),
                                area.height / (max_y - min_y));
  const double used_width = (max_x - min_x) * scale;
  const double used_height = (max_y - min_y) * scale;
  const double left = area.x + 0.5 * (area.width - used_width);
  const double bottom = area.y + 0.5 * (area.height + used_height);
  return {static_cast<int>(std::lround(left + (x - min_x) * scale)),
          static_cast<int>(std::lround(bottom - (y - min_y) * scale))};
}

double WorldScale(const cv::Rect& area, double min_x, double max_x,
                  double min_y, double max_y) {
  return std::min(area.width / (max_x - min_x),
                  area.height / (max_y - min_y));
}

void DrawDashedLine(cv::Mat& image, cv::Point start, cv::Point end,
                    const cv::Scalar& color, int thickness = 1) {
  const double length = cv::norm(end - start);
  if (length < 1.0) return;
  constexpr double dash = 7.0;
  constexpr double gap = 5.0;
  for (double offset = 0.0; offset < length; offset += dash + gap) {
    const double t0 = offset / length;
    const double t1 = std::min(offset + dash, length) / length;
    const cv::Point p0 = start + (end - start) * t0;
    const cv::Point p1 = start + (end - start) * t1;
    cv::line(image, p0, p1, color, thickness, cv::LINE_AA);
  }
}

}  // namespace

struct DpcbfVisualizer::Impl {
  bool enabled = true;
  int width = 1500;
  int height = 900;
  double refresh_hz = 30.0;
  double velocity_axis_limit = 3.0;
  double world_margin = 1.0;
  double velocity_arrow_seconds = 1.0;
  double world_parabola_lateral_limit = 1.0;
  double world_parabola_backward_limit = 1.5;
  std::string window_name = "DPCBF Top-Down and Velocity Boundaries";
  std::array<double, 2> arena_size{10.0, 10.0};
  std::array<double, 2> arena_center{0.0, 0.0};
  double robot_radius = 0.25;
  double detection_radius = 5.0;

  std::mutex mutex;
  RobotState robot;
  std::vector<ObstacleState> all_obstacles;
  SafetyFilterResult filter_result;
  bool has_data = false;
  std::atomic<bool> running{false};
  std::thread render_thread;

  void RenderLoop() {
    try {
      cv::namedWindow(window_name, cv::WINDOW_NORMAL);
      cv::resizeWindow(window_name, width, height);
      const auto period = std::chrono::duration<double>(1.0 / refresh_hz);
      while (running.load()) {
        const auto start = std::chrono::steady_clock::now();
        RobotState robot_copy;
        std::vector<ObstacleState> obstacles_copy;
        SafetyFilterResult result_copy;
        bool ready = false;
        {
          const std::lock_guard<std::mutex> lock(mutex);
          robot_copy = robot;
          obstacles_copy = all_obstacles;
          result_copy = filter_result;
          ready = has_data;
        }
        cv::Mat frame(height, width, CV_8UC3, cv::Scalar(246, 246, 246));
        if (ready) {
          DrawFrame(frame, robot_copy, obstacles_copy, result_copy);
        } else {
          cv::putText(frame, "Waiting for MuJoCo state...", {40, 70},
                      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(60, 60, 60), 2,
                      cv::LINE_AA);
        }
        cv::imshow(window_name, frame);
        cv::waitKey(1);
        if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) < 1.0) {
          running.store(false);
          break;
        }
        std::this_thread::sleep_until(start + period);
      }
      cv::destroyWindow(window_name);
    } catch (const cv::Exception& error) {
      std::cerr << "DPCBF visualization window stopped: " << error.what() << '\n';
      running.store(false);
    }
  }

  void DrawFrame(cv::Mat& frame, const RobotState& robot_state,
                 const std::vector<ObstacleState>& obstacles,
                 const SafetyFilterResult& result) const {
    const int world_width = static_cast<int>(width * 0.53);
    const cv::Rect world_area(25, 60, world_width - 50, height - 90);
    cv::putText(frame, "World top-down", {25, 35}, cv::FONT_HERSHEY_SIMPLEX,
                0.8, cv::Scalar(45, 45, 45), 2, cv::LINE_AA);

    const double arena_min_x = arena_center[0] - 0.5 * arena_size[0];
    const double arena_max_x = arena_center[0] + 0.5 * arena_size[0];
    const double arena_min_y = arena_center[1] - 0.5 * arena_size[1];
    const double arena_max_y = arena_center[1] + 0.5 * arena_size[1];
    const double min_x = std::min(arena_min_x, robot_state.x - detection_radius) - world_margin;
    const double max_x = std::max(arena_max_x, robot_state.x + detection_radius) + world_margin;
    const double min_y = std::min(arena_min_y, robot_state.y - detection_radius) - world_margin;
    const double max_y = std::max(arena_max_y, robot_state.y + detection_radius) + world_margin;
    const double scale = WorldScale(world_area, min_x, max_x, min_y, max_y);

    const cv::Point arena_top_left = WorldToPixel(arena_min_x, arena_max_y,
                                                   world_area, min_x, max_x, min_y, max_y);
    const cv::Point arena_bottom_right = WorldToPixel(arena_max_x, arena_min_y,
                                                       world_area, min_x, max_x, min_y, max_y);
    cv::rectangle(frame, arena_top_left, arena_bottom_right,
                  cv::Scalar(165, 145, 105), 2, cv::LINE_AA);

    const cv::Point robot_pixel = WorldToPixel(robot_state.x, robot_state.y,
                                                world_area, min_x, max_x, min_y, max_y);
    const int detection_pixels = std::max(1, static_cast<int>(detection_radius * scale));
    for (int angle = 0; angle < 360; angle += 16) {
      cv::ellipse(frame, robot_pixel, {detection_pixels, detection_pixels}, 0,
                  angle, angle + 9, cv::Scalar(175, 175, 175), 1, cv::LINE_AA);
    }

    const auto selected_color = [&result](int id, std::size_t* selected_index) {
      for (std::size_t i = 0; i < result.selected_obstacles.size(); ++i) {
        if (result.selected_obstacles[i].obstacle.id == id) {
          if (selected_index) *selected_index = i;
          return true;
        }
      }
      return false;
    };

    for (const auto& obstacle : obstacles) {
      const cv::Point center = WorldToPixel(obstacle.x, obstacle.y, world_area,
                                             min_x, max_x, min_y, max_y);
      const int radius = std::max(3, static_cast<int>(obstacle.radius * scale));
      std::size_t active_index = 0;
      const bool active = selected_color(obstacle.id, &active_index);
      cv::circle(frame, center, radius, active ? cv::Scalar(125, 75, 35)
                                               : cv::Scalar(175, 125, 85),
                 cv::FILLED, cv::LINE_AA);
      cv::circle(frame, center, radius,
                 active ? kActiveColors[active_index % kActiveColors.size()]
                        : cv::Scalar(120, 90, 65),
                 active ? 3 : 1, cv::LINE_AA);
    }

    for (std::size_t i = 0; i < result.selected_obstacles.size(); ++i) {
      const auto& visual = result.selected_obstacles[i];
      const cv::Scalar color = kActiveColors[i % kActiveColors.size()];
      const cv::Point obstacle_pixel = WorldToPixel(
          visual.obstacle.x, visual.obstacle.y, world_area,
          min_x, max_x, min_y, max_y);
      DrawDashedLine(frame, robot_pixel, obstacle_pixel, color, 1);

      const double los_angle = std::atan2(visual.obstacle.y - robot_state.y,
                                          visual.obstacle.x - robot_state.x);
      const double cosine = std::cos(los_angle);
      const double sine = std::sin(los_angle);
      std::vector<cv::Point> world_parabola;
      constexpr int parabola_samples = 100;
      world_parabola.reserve(parabola_samples + 1);
      for (int sample = 0; sample <= parabola_samples; ++sample) {
        const double local_y =
            -world_parabola_lateral_limit +
            2.0 * world_parabola_lateral_limit * sample / parabola_samples;
        const double local_x =
            visual.boundary_vertex_x -
            visual.boundary_curvature * local_y * local_y;
        if (local_x < -world_parabola_backward_limit) {
          continue;
        }
        const double world_x =
            robot_state.x +
            velocity_arrow_seconds * (cosine * local_x - sine * local_y);
        const double world_y =
            robot_state.y +
            velocity_arrow_seconds * (sine * local_x + cosine * local_y);
        world_parabola.push_back(WorldToPixel(
            world_x, world_y, world_area, min_x, max_x, min_y, max_y));
      }
      if (world_parabola.size() >= 2) {
        cv::polylines(frame, world_parabola, false, color, 2, cv::LINE_AA);
      }

      const double arrow_world_x =
          robot_state.x +
          visual.relative_velocity_world[0] * velocity_arrow_seconds;
      const double arrow_world_y =
          robot_state.y +
          visual.relative_velocity_world[1] * velocity_arrow_seconds;
      const cv::Point arrow_end = WorldToPixel(
          arrow_world_x, arrow_world_y, world_area,
          min_x, max_x, min_y, max_y);
      cv::arrowedLine(frame, robot_pixel, arrow_end, color, 2,
                      cv::LINE_AA, 0, 0.22);
      cv::putText(frame, "#" + std::to_string(i + 1),
                  obstacle_pixel + cv::Point(7, -7), cv::FONT_HERSHEY_SIMPLEX,
                  0.5, color, 2, cv::LINE_AA);
    }

    const int robot_pixels = std::max(5, static_cast<int>(robot_radius * scale));
    cv::circle(frame, robot_pixel, robot_pixels, cv::Scalar(215, 215, 215),
               cv::FILLED, cv::LINE_AA);
    cv::circle(frame, robot_pixel, robot_pixels, cv::Scalar(75, 75, 75), 2,
               cv::LINE_AA);
    const cv::Point heading_end(
        robot_pixel.x + static_cast<int>(1.7 * robot_pixels * std::cos(robot_state.phi)),
        robot_pixel.y - static_cast<int>(1.7 * robot_pixels * std::sin(robot_state.phi)));
    cv::arrowedLine(frame, robot_pixel, heading_end, cv::Scalar(60, 60, 60), 2,
                    cv::LINE_AA, 0, 0.25);
    cv::putText(frame, "p_max=" + std::to_string(detection_radius).substr(0, 3) + "m",
                {world_area.x + 10, world_area.y + 25}, cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(95, 95, 95), 1, cv::LINE_AA);

    DrawVelocityPanels(frame, result, world_width);
  }

  void DrawVelocityPanels(cv::Mat& frame, const SafetyFilterResult& result,
                          int start_x) const {
    const int panel_x = start_x + 10;
    const int panel_width = width - panel_x - 20;
    cv::putText(frame, "DPCBF h=0 in each obstacle LoS velocity frame",
                {panel_x, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.72,
                cv::Scalar(45, 45, 45), 2, cv::LINE_AA);
    if (result.selected_obstacles.empty()) {
      cv::putText(frame, "No obstacle selected inside p_max", {panel_x + 20, 90},
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(100, 100, 100), 1,
                  cv::LINE_AA);
      return;
    }

    const int count = static_cast<int>(result.selected_obstacles.size());
    const int columns = count <= 3 ? 1 : 2;
    const int rows = (count + columns - 1) / columns;
    const int gap = 10;
    const int card_width = (panel_width - gap * (columns - 1)) / columns;
    const int card_height = (height - 75 - gap * (rows - 1)) / rows;
    for (int index = 0; index < count; ++index) {
      const int column = index % columns;
      const int row = index / columns;
      const cv::Rect card(panel_x + column * (card_width + gap),
                          55 + row * (card_height + gap),
                          card_width, card_height);
      DrawVelocityCard(frame, card, result.selected_obstacles[index], index);
    }
  }

  void DrawVelocityCard(cv::Mat& frame, const cv::Rect& card,
                        const DpcbfVisualizationObstacle& visual,
                        int index) const {
    const cv::Scalar color = kActiveColors[index % kActiveColors.size()];
    cv::rectangle(frame, card, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    const cv::Rect plot(card.x + 40, card.y + 28,
                        std::max(20, card.width - 52),
                        std::max(20, card.height - 52));
    const auto velocity_to_pixel = [&](double x, double y) {
      const double normalized_x = (x + velocity_axis_limit) / (2.0 * velocity_axis_limit);
      const double normalized_y = (y + velocity_axis_limit) / (2.0 * velocity_axis_limit);
      return cv::Point(plot.x + static_cast<int>(normalized_x * plot.width),
                       plot.y + plot.height - static_cast<int>(normalized_y * plot.height));
    };
    const cv::Point origin = velocity_to_pixel(0.0, 0.0);
    cv::line(frame, {plot.x, origin.y}, {plot.x + plot.width, origin.y},
             cv::Scalar(185, 185, 185), 1, cv::LINE_AA);
    cv::line(frame, {origin.x, plot.y}, {origin.x, plot.y + plot.height},
             cv::Scalar(185, 185, 185), 1, cv::LINE_AA);

    std::vector<cv::Point> unsafe_polygon;
    unsafe_polygon.push_back(velocity_to_pixel(-velocity_axis_limit,
                                                -velocity_axis_limit));
    std::vector<cv::Point> curve;
    constexpr int samples = 100;
    for (int sample = 0; sample <= samples; ++sample) {
      const double y = -velocity_axis_limit +
                       2.0 * velocity_axis_limit * sample / samples;
      const double boundary_x = visual.boundary_vertex_x -
                                visual.boundary_curvature * y * y;
      curve.push_back(velocity_to_pixel(
          std::clamp(boundary_x, -velocity_axis_limit, velocity_axis_limit), y));
    }
    unsafe_polygon.insert(unsafe_polygon.end(), curve.begin(), curve.end());
    unsafe_polygon.push_back(velocity_to_pixel(-velocity_axis_limit,
                                                velocity_axis_limit));
    cv::Mat overlay = frame.clone();
    cv::fillConvexPoly(overlay, unsafe_polygon, cv::Scalar(225, 225, 238),
                       cv::LINE_AA);
    cv::addWeighted(overlay, 0.38, frame, 0.62, 0.0, frame);
    for (std::size_t i = 1; i < curve.size(); ++i) {
      cv::line(frame, curve[i - 1], curve[i], color, 2, cv::LINE_AA);
    }

    const cv::Point relative = velocity_to_pixel(visual.relative_velocity_los[0],
                                                  visual.relative_velocity_los[1]);
    cv::arrowedLine(frame, origin, relative, color, 2, cv::LINE_AA, 0, 0.18);
    cv::circle(frame, relative, 4, color, cv::FILLED, cv::LINE_AA);
    const std::string title = "#" + std::to_string(index + 1) +
        " id=" + std::to_string(visual.obstacle.id) +
        "  d=" + cv::format("%.2f", visual.distance) +
        "m  h=" + cv::format("%.3f", visual.constraint.h);
    cv::putText(frame, title, {card.x + 8, card.y + 19},
                cv::FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv::LINE_AA);
    cv::putText(frame, "X (radial)", {plot.x + plot.width - 72, origin.y - 4},
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(100, 100, 100), 1,
                cv::LINE_AA);
    cv::putText(frame, "Y", {origin.x + 4, plot.y + 12},
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(100, 100, 100), 1,
                cv::LINE_AA);
  }
};

DpcbfVisualizer::DpcbfVisualizer() : impl_(std::make_unique<Impl>()) {}
DpcbfVisualizer::~DpcbfVisualizer() { Stop(); }
DpcbfVisualizer::DpcbfVisualizer(DpcbfVisualizer&&) noexcept = default;
DpcbfVisualizer& DpcbfVisualizer::operator=(DpcbfVisualizer&&) noexcept = default;

void DpcbfVisualizer::LoadConfig(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node cfg = root["visualization"];
  const YAML::Node arena = root["dynamic_obstacles"]["arena"];
  const YAML::Node robot = root["robot"];
  if (!cfg || !arena || !robot) {
    throw std::runtime_error("missing visualization, arena, or robot configuration");
  }
  impl_->enabled = cfg["enabled"].as<bool>(true);
  impl_->width = cfg["window_width"].as<int>(1500);
  impl_->height = cfg["window_height"].as<int>(900);
  impl_->refresh_hz = cfg["refresh_hz"].as<double>(30.0);
  impl_->velocity_axis_limit = cfg["velocity_axis_limit"].as<double>(3.0);
  impl_->world_margin = cfg["world_margin"].as<double>(1.0);
  impl_->velocity_arrow_seconds = cfg["velocity_arrow_seconds"].as<double>(1.0);
  impl_->world_parabola_lateral_limit =
      cfg["world_parabola_lateral_limit"].as<double>(1.0);
  impl_->world_parabola_backward_limit =
      cfg["world_parabola_backward_limit"].as<double>(1.5);
  impl_->window_name = cfg["window_name"].as<std::string>(impl_->window_name);
  impl_->arena_size = {arena["size"][0].as<double>(), arena["size"][1].as<double>()};
  impl_->arena_center = {arena["center"][0].as<double>(), arena["center"][1].as<double>()};
  impl_->robot_radius = robot["r_rob"].as<double>();
  impl_->detection_radius = robot["p_max"].as<double>();
  if (impl_->width < 800 || impl_->height < 500 || impl_->refresh_hz <= 0.0 ||
      impl_->velocity_axis_limit <= 0.0 || impl_->world_margin < 0.0 ||
      impl_->velocity_arrow_seconds <= 0.0 ||
      impl_->world_parabola_lateral_limit <= 0.0 ||
      impl_->world_parabola_backward_limit <= 0.0) {
    throw std::runtime_error("invalid visualization configuration");
  }
}

void DpcbfVisualizer::Start() {
  if (!impl_->enabled || impl_->running.exchange(true)) return;
  impl_->render_thread = std::thread([this] { impl_->RenderLoop(); });
}

void DpcbfVisualizer::Stop() {
  impl_->running.store(false);
  if (impl_->render_thread.joinable()) impl_->render_thread.join();
}

void DpcbfVisualizer::Update(const RobotState& robot,
                             const std::vector<ObstacleState>& all_obstacles,
                             const SafetyFilterResult& filter_result) {
  if (!impl_->enabled) return;
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->robot = robot;
  impl_->all_obstacles = all_obstacles;
  impl_->filter_result = filter_result;
  impl_->has_data = true;
}

bool DpcbfVisualizer::enabled() const { return impl_->enabled; }

}  // namespace dpcbf
