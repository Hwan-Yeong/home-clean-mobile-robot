#ifndef CLEAN_ROBOT_PKG__ROBOT_FACADE_HPP_
#define CLEAN_ROBOT_PKG__ROBOT_FACADE_HPP_

#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace clean_robot_pkg
{

class RobotFacade
{
public:
  using ComputeCoveragePath = opennav_coverage_msgs::action::ComputeCoveragePath;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit RobotFacade(rclcpp::Node* node);

  std::optional<std::pair<double, double>> get_pose();
  
  void request_coverage_path(
    const geometry_msgs::msg::PolygonStamped & polygon,
    std::function<void(const nav_msgs::msg::Path &)> callback);

  void follow_path(
    const nav_msgs::msg::Path & path,
    std::function<void(bool)> result_callback,
    std::function<void(float)> feedback_callback);

  void navigate_to_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    std::function<void(bool)> result_callback);

  void turn_to_heading(double target_yaw, std::function<void(bool)> callback);
  
  void set_initial_pose(double x, double y, double yaw);
  bool is_localized();

  void cancel_all_navigation();
  void publish_markers(const nav_msgs::msg::Path & path);

private:
  rclcpp::Node* node_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  
  rclcpp_action::Client<ComputeCoveragePath>::SharedPtr coverage_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  
  rclcpp::TimerBase::SharedPtr rotation_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__ROBOT_FACADE_HPP_
