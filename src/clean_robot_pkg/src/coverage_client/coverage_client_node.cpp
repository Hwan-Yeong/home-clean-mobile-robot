#include "clean_robot_pkg/coverage_client_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace clean_robot_pkg
{

CoverageClientNode::CoverageClientNode(const rclcpp::NodeOptions & options)
: Node("coverage_client_node", options), 
  cleaning_in_progress_(false), 
  ready_logged_(false),
  transitioning_(false),
  obstacle_detected_(false)
{
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/cleaning_zone", 10,
    std::bind(&CoverageClientNode::polygon_callback, this, std::placeholders::_1));

  clean_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/clean_start", 10,
    std::bind(&CoverageClientNode::clean_start_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CoverageClientNode::scan_callback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/coverage_path_markers", 10);

  coverage_client_ = rclcpp_action::create_client<ComputeCoveragePath>(
    this, "/compute_coverage_path", callback_group_);
  
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    this, "/follow_path", callback_group_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  path_manager_ = std::make_unique<CoveragePathManager>(this->get_logger());

  monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CoverageClientNode::obstacle_monitor_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "CoverageClientNode initialized.");
}

void CoverageClientNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int angle_range = 15;
  bool obstacle = false;
  for (int i = -angle_range; i <= angle_range; ++i) {
    size_t idx = (i + msg->ranges.size()) % msg->ranges.size();
    if (msg->ranges[idx] > msg->range_min && msg->ranges[idx] < msg->range_max) {
      if (msg->ranges[idx] < 0.6) { obstacle = true; break; }
    }
  }
  obstacle_detected_ = obstacle;
}

void CoverageClientNode::polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (msg->polygon.points.size() < 3) return;
  latest_polygon_ = msg;
  if (!ready_logged_) {
    ready_logged_ = true;
    RCLCPP_INFO(this->get_logger(), "Cleaning zone ready.");
  }
}

void CoverageClientNode::clean_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !cleaning_in_progress_ && latest_polygon_) {
    start_coverage(*latest_polygon_);
  }
}

void CoverageClientNode::obstacle_monitor_timer_callback()
{
  if (cleaning_in_progress_ && !transitioning_ && obstacle_detected_) {
    perform_path_transition();
  }
}

void CoverageClientNode::perform_path_transition()
{
  transitioning_ = true;
  follow_path_client_->async_cancel_all_goals();
  auto pose = get_robot_pose_in_map();
  if (!pose) { transitioning_ = false; return; }

  auto check_blocked = [&](double x, double y) {
    return std::hypot(x - pose->first, y - pose->second) < 0.5;
  };

  auto [lane_idx, wp_idx] = path_manager_->search_next_clear_waypoint(pose->first, pose->second, check_blocked);
  if (lane_idx != -1) {
    send_updated_path(path_manager_->get_path_from_waypoint(lane_idx, wp_idx));
  } else {
    cleaning_in_progress_ = false;
  }
  transitioning_ = false;
}

void CoverageClientNode::send_updated_path(nav_msgs::msg::Path path)
{
  auto now = this->now();
  path.header.stamp = now;
  for (auto & p : path.poses) {
    p.header.stamp = now;
    p.header.frame_id = path.header.frame_id;
    if (p.pose.orientation.w == 0.0) p.pose.orientation.w = 1.0;
  }
  follow_coverage_path(path);
  publish_path_markers(path);
}

void CoverageClientNode::start_coverage(const geometry_msgs::msg::PolygonStamped & polygon_msg)
{
  if (!coverage_client_->wait_for_action_server(std::chrono::seconds(5))) return;
  auto goal = ComputeCoveragePath::Goal();
  goal.generate_path = true;
  goal.frame_id = polygon_msg.header.frame_id;
  opennav_coverage_msgs::msg::Coordinates boundary;
  for (const auto & pt : polygon_msg.polygon.points) {
    opennav_coverage_msgs::msg::Coordinate c; c.axis1 = pt.x; c.axis2 = pt.y;
    boundary.coordinates.push_back(c);
  }
  goal.polygons.push_back(boundary);
  cleaning_in_progress_ = true;
  auto options = rclcpp_action::Client<ComputeCoveragePath>::SendGoalOptions();
  options.result_callback = std::bind(&CoverageClientNode::coverage_result_callback, this, std::placeholders::_1);
  coverage_client_->async_send_goal(goal, options);
}

void CoverageClientNode::coverage_result_callback(const GoalHandleCompute::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) { cleaning_in_progress_ = false; return; }
  path_manager_->set_path(result.result->nav_path);
  auto pose = get_robot_pose_in_map();
  if (pose) {
    auto [lane_idx, wp_idx] = path_manager_->find_nearest_waypoint_global(pose->first, pose->second);
    if (lane_idx != -1) {
      auto trimmed = path_manager_->get_path_from_waypoint(lane_idx, wp_idx);
      send_updated_path(trimmed);
    } else {
      send_updated_path(result.result->nav_path);
    }
  } else {
    send_updated_path(result.result->nav_path);
  }
}

void CoverageClientNode::follow_coverage_path(const nav_msgs::msg::Path & path)
{
  if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) return;
  auto goal = FollowPath::Goal(); goal.path = path;
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.result_callback = std::bind(&CoverageClientNode::follow_path_result_callback, this, std::placeholders::_1);
  follow_path_client_->async_send_goal(goal, options);
}

void CoverageClientNode::follow_path_result_callback(const GoalHandleFollow::WrappedResult & result)
{
  cleaning_in_progress_ = false;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) RCLCPP_INFO(this->get_logger(), "Done!");
}

std::optional<std::pair<double, double>> CoverageClientNode::get_robot_pose_in_map()
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
    return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
  } catch (...) { return std::nullopt; }
}

void CoverageClientNode::publish_path_markers(const nav_msgs::msg::Path & path)
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
