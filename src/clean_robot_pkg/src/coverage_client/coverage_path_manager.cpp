#include "clean_robot_pkg/coverage_path_manager.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace clean_robot_pkg
{

CoveragePathManager::CoveragePathManager(const rclcpp::Logger & logger)
: logger_(logger), current_lane_idx_(0), lane_threshold_rad_(M_PI / 4.0)
{}

void CoveragePathManager::set_path(const nav_msgs::msg::Path & path)
{
  raw_path_ = path;
  lanes_ = split_into_lanes(path);
  current_lane_idx_ = 0;
}

const CoveragePathManager::Lane * CoveragePathManager::get_current_lane() const
{
  if (current_lane_idx_ < lanes_.size()) {
    return &lanes_[current_lane_idx_];
  }
  return nullptr;
}

void CoveragePathManager::increment_lane()
{
  if (current_lane_idx_ < lanes_.size()) {
    current_lane_idx_++;
  }
}

std::pair<int, double> CoveragePathManager::find_nearest_waypoint_in_lane(size_t lane_idx, double rx, double ry) const
{
  if (lane_idx >= lanes_.size()) {
    return {-1, -1.0};
  }

  const auto & lane = lanes_[lane_idx];
  int best_idx = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < lane.size(); ++i) {
    double dx = lane[i].pose.position.x - rx;
    double dy = lane[i].pose.position.y - ry;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = static_cast<int>(i);
    }
  }
  return {best_idx, min_dist};
}

std::pair<int, int> CoveragePathManager::find_nearest_waypoint_global(double rx, double ry) const
{
  int best_lane = -1;
  int best_wp = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < lanes_.size(); ++i) {
    auto [wp_idx, dist] = find_nearest_waypoint_in_lane(i, rx, ry);
    if (wp_idx != -1 && dist < min_dist) {
      min_dist = dist;
      best_lane = static_cast<int>(i);
      best_wp = wp_idx;
    }
  }

  return {best_lane, best_wp};
}

nav_msgs::msg::Path CoveragePathManager::get_path_from_waypoint(size_t lane_idx, size_t start_wp_idx) const
{
  nav_msgs::msg::Path new_path;
  if (lane_idx >= lanes_.size()) return new_path;

  new_path.header = raw_path_.header;
  for (size_t i = start_wp_idx; i < lanes_[lane_idx].size(); ++i) {
    new_path.poses.push_back(lanes_[lane_idx][i]);
  }
  for (size_t i = lane_idx + 1; i < lanes_.size(); ++i) {
    for (const auto & pose : lanes_[i]) {
      new_path.poses.push_back(pose);
    }
  }
  return new_path;
}

std::pair<int, int> CoveragePathManager::search_next_clear_waypoint(
  double rx, double ry, 
  std::function<bool(double, double)> check_obstacle_func)
{
  if (current_lane_idx_ >= lanes_.size()) return {-1, -1};

  auto [near_idx, dist] = find_nearest_waypoint_in_lane(current_lane_idx_, rx, ry);
  if (near_idx == -1) return {-1, -1};

  const auto & current_lane = lanes_[current_lane_idx_];
  int found_wp_idx = -1;
  for (size_t i = static_cast<size_t>(near_idx); i < current_lane.size(); ++i) {
    if (!check_obstacle_func(current_lane[i].pose.position.x, current_lane[i].pose.position.y)) {
      found_wp_idx = static_cast<int>(i);
      break;
    }
  }

  if (found_wp_idx != -1) {
    double d = std::hypot(current_lane[found_wp_idx].pose.position.x - rx,
                         current_lane[found_wp_idx].pose.position.y - ry);
    if (d < 2.0) return {static_cast<int>(current_lane_idx_), found_wp_idx};
  }

  if (current_lane_idx_ + 1 < lanes_.size()) {
    current_lane_idx_++;
    auto [next_idx, next_dist] = find_nearest_waypoint_in_lane(current_lane_idx_, rx, ry);
    return {static_cast<int>(current_lane_idx_), next_idx};
  }

  return {-1, -1};
}

std::vector<CoveragePathManager::Lane> CoveragePathManager::split_into_lanes(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) return {};
  std::vector<Lane> lanes;
  Lane current_lane;
  if (path.poses.size() < 2) {
    current_lane.push_back(path.poses[0]);
    lanes.push_back(current_lane);
    return lanes;
  }

  double prev_heading = 0.0;
  bool has_prev_heading = false;
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
    double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 0.05) {
      current_lane.push_back(path.poses[i]);
      continue;
    }
    double heading = std::atan2(dy, dx);
    if (has_prev_heading) {
      double diff = std::remainder(heading - prev_heading, 2.0 * M_PI);
      if (std::abs(diff) > lane_threshold_rad_) {
        if (!current_lane.empty()) lanes.push_back(current_lane);
        current_lane.clear();
      }
    }
    current_lane.push_back(path.poses[i]);
    prev_heading = heading;
    has_prev_heading = true;
  }
  current_lane.push_back(path.poses.back());
  lanes.push_back(current_lane);

  std::vector<Lane> filtered;
  for (auto & l : lanes) { if (l.size() > 2) filtered.push_back(l); }
  return filtered;
}

} // namespace clean_robot_pkg
