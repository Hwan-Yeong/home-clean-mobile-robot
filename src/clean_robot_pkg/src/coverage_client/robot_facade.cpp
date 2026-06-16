#include "clean_robot_pkg/robot_facade.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace clean_robot_pkg
{

RobotFacade::RobotFacade(rclcpp::Node* node)
: node_(node)
{
  action_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  coverage_client_ = rclcpp_action::create_client<ComputeCoveragePath>(
    node_, "/compute_coverage_path", action_cb_group_);
  
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    node_, "/follow_path", action_cb_group_);

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/coverage_path_markers", 10);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::optional<std::pair<double, double>> RobotFacade::get_pose()
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
    return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
  } catch (...) {
    return std::nullopt;
  }
}

void RobotFacade::request_coverage_path(
  const geometry_msgs::msg::PolygonStamped & polygon,
  std::function<void(const nav_msgs::msg::Path &)> callback)
{
  if (!coverage_client_->wait_for_action_server(std::chrono::seconds(10))) return;

  auto goal = ComputeCoveragePath::Goal();
  goal.generate_path = true;
  goal.frame_id = polygon.header.frame_id;
  opennav_coverage_msgs::msg::Coordinates boundary;
  for (const auto & pt : polygon.polygon.points) {
    opennav_coverage_msgs::msg::Coordinate c; c.axis1 = pt.x; c.axis2 = pt.y;
    boundary.coordinates.push_back(c);
  }
  goal.polygons.push_back(boundary);

  auto options = rclcpp_action::Client<ComputeCoveragePath>::SendGoalOptions();
  options.result_callback = [callback](const rclcpp_action::ClientGoalHandle<ComputeCoveragePath>::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      callback(result.result->nav_path);
    }
  };
  coverage_client_->async_send_goal(goal, options);
}

void RobotFacade::follow_path(
  const nav_msgs::msg::Path & path,
  std::function<void(bool)> result_callback,
  std::function<void(float)> feedback_callback)
{
  if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(10))) return;

  auto goal = FollowPath::Goal();
  goal.path = path;

  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  
  options.result_callback = [result_callback](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult & result) {
    result_callback(result.code == rclcpp_action::ResultCode::SUCCEEDED);
  };

  options.feedback_callback = [feedback_callback](
    rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback) {
    feedback_callback(feedback->distance_to_goal);
  };

  follow_path_client_->async_send_goal(goal, options);
}

void RobotFacade::turn_to_heading(double target_yaw, std::function<void(bool)> callback)
{
  if (rotation_timer_) rotation_timer_->cancel();

  auto start_time = node_->now();
  rotation_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this, target_yaw, callback, start_time]() {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      double current_yaw = tf2::getYaw(tf.transform.rotation);
      double diff = std::remainder(target_yaw - current_yaw, 2.0 * M_PI);

      if (std::abs(diff) < 0.1 || (node_->now() - start_time).seconds() > 5.0) {
        geometry_msgs::msg::Twist stop;
        vel_pub_->publish(stop);
        rotation_timer_->cancel();
        callback(true);
        return;
      }

      geometry_msgs::msg::Twist twist;
      twist.angular.z = (diff > 0) ? 0.6 : -0.6;
      vel_pub_->publish(twist);
    } catch (...) {
      // Wait for next tick
    }
  });
}

void RobotFacade::cancel_all_navigation()
{
  follow_path_client_->async_cancel_all_goals();
}

void RobotFacade::publish_markers(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) return;
  auto m = [&](int id, const geometry_msgs::msg::Pose & p, float r, float g, float b) {
    visualization_msgs::msg::Marker mk; mk.header = path.header; mk.id = id;
    mk.type = mk.ARROW; mk.pose = p; mk.scale.x = 0.5; mk.scale.y = 0.1; mk.scale.z = 0.1;
    mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = 1.0; return mk;
  };
  marker_pub_->publish(m(0, path.poses.front().pose, 0.0, 1.0, 0.0));
  marker_pub_->publish(m(1, path.poses.back().pose, 1.0, 0.0, 0.0));
}

} // namespace clean_robot_pkg
