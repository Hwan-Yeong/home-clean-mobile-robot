#ifndef CLEAN_ROBOT_PKG__COVERAGE_PATH_MANAGER_HPP_
#define CLEAN_ROBOT_PKG__COVERAGE_PATH_MANAGER_HPP_

#include <vector>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace clean_robot_pkg
{

class CoveragePathManager
{
public:
  using Lane = std::vector<geometry_msgs::msg::PoseStamped>;
  enum class LaneStatus { PENDING, IN_PROGRESS, CLEANED, BLOCKED };

  explicit CoveragePathManager(const rclcpp::Logger & logger);

  void set_path(const nav_msgs::msg::Path & path);
  
  const Lane * get_current_lane() const;
  void mark_lane_status(size_t index, LaneStatus status);
  void update_progress(double rx, double ry, double threshold = 0.3);

  size_t get_current_lane_idx() const { return current_lane_idx_; }
  void set_current_lane_idx(size_t idx) { current_lane_idx_ = idx; }
  size_t get_num_lanes() const { return lanes_.size(); }
  size_t get_cleaned_lanes_count() const;

  std::pair<int, double> find_nearest_waypoint_in_lane(size_t lane_idx, double rx, double ry) const;
  std::pair<int, int> find_nearest_waypoint_global(double rx, double ry) const;
  
  // Intelligent path selection
  std::pair<int, int> select_next_best_lane(double rx, double ry);
  
  nav_msgs::msg::Path get_path_from_waypoint(size_t lane_idx, size_t start_wp_idx) const;
  
  std::pair<int, int> search_next_clear_waypoint(
    double rx, double ry, 
    std::function<bool(double, double)> check_obstacle_func);

private:
  std::vector<Lane> split_into_lanes(const nav_msgs::msg::Path & path);

  rclcpp::Logger logger_;
  nav_msgs::msg::Path raw_path_;
  std::vector<Lane> lanes_;
  std::vector<LaneStatus> lane_statuses_;
  size_t current_lane_idx_;
  double lane_threshold_rad_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__COVERAGE_PATH_MANAGER_HPP_
