#ifndef CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_
#define CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "clean_robot_pkg/robot_facade.hpp"
#include "clean_robot_pkg/strategies/i_cleaning_strategy.hpp"
#include "clean_robot_pkg/strategies/zigzag_strategy.hpp"

namespace clean_robot_pkg
{

class CoverageClientNode : public rclcpp::Node
{
public:
  explicit CoverageClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void clean_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  
  void monitor_timer_callback();
  void logging_timer_callback();
  void perform_path_transition();
  void start_mission();
  void auto_localize();

  std::shared_ptr<RobotFacade> robot_;
  std::shared_ptr<ICleaningStrategy> strategy_;
  
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clean_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr logging_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_polygon_;
  bool cleaning_in_progress_;
  bool transitioning_;
  bool obstacle_detected_;
  bool is_rotating_;
  bool localized_;
  int localization_retries_;
  rclcpp::Time last_transition_time_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_
