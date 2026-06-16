#ifndef CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_
#define CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_

#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "clean_robot_pkg/coverage_path_manager.hpp"

namespace clean_robot_pkg
{

class CoverageClientNode : public rclcpp::Node
{
public:
  using ComputeCoveragePath = opennav_coverage_msgs::action::ComputeCoveragePath;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleCompute = rclcpp_action::ClientGoalHandle<ComputeCoveragePath>;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit CoverageClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void clean_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  
  void obstacle_monitor_timer_callback();
  void perform_path_transition();
  void send_updated_path(nav_msgs::msg::Path path);
  
  void start_coverage(const geometry_msgs::msg::PolygonStamped & polygon_msg);
  void coverage_result_callback(const GoalHandleCompute::WrappedResult & result);
  
  void follow_coverage_path(const nav_msgs::msg::Path & path);
  void follow_path_result_callback(const GoalHandleFollow::WrappedResult & result);

  std::optional<std::pair<double, double>> get_robot_pose_in_map();
  void publish_path_markers(const nav_msgs::msg::Path & path);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clean_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  rclcpp_action::Client<ComputeCoveragePath>::SharedPtr coverage_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<CoveragePathManager> path_manager_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_polygon_;
  bool cleaning_in_progress_;
  bool ready_logged_;
  bool transitioning_;
  bool obstacle_detected_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__COVERAGE_CLIENT_NODE_HPP_
