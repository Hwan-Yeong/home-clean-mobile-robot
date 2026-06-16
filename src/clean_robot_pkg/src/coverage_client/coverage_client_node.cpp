#include "clean_robot_pkg/coverage_client_node.hpp"
#include <tf2/utils.h>

namespace clean_robot_pkg
{

CoverageClientNode::CoverageClientNode(const rclcpp::NodeOptions & options)
: Node("coverage_client_node", options),
  cleaning_in_progress_(false),
  transitioning_(false),
  obstacle_detected_(false),
  is_rotating_(false),
  last_transition_time_(this->now())
{
  // Initialize Facade and Strategy
  robot_ = std::make_shared<RobotFacade>(this);
  strategy_ = std::make_shared<ZigZagStrategy>(this->get_logger());

  polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/cleaning_zone", 10,
    std::bind(&CoverageClientNode::polygon_callback, this, std::placeholders::_1));

  clean_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/clean_start", 10,
    std::bind(&CoverageClientNode::clean_start_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&CoverageClientNode::scan_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CoverageClientNode::monitor_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Coverage Orchestrator initialized (Strategy: %s)", strategy_->get_name().c_str());
}

void CoverageClientNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Simple front obstacle detection
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
  strategy_->set_area(*msg);
}

void CoverageClientNode::clean_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !cleaning_in_progress_ && latest_polygon_) {
    start_mission();
  }
}

void CoverageClientNode::monitor_timer_callback()
{
  if (!cleaning_in_progress_ || transitioning_ || is_rotating_) return;

  // 1. Cooldown logic: Block transition for 5 seconds after the last one
  if ((this->now() - last_transition_time_).seconds() < 5.0) return;

  // 2. Obstacle detection
  if (obstacle_detected_) {
    perform_path_transition();
  }
}

void CoverageClientNode::start_mission()
{
  RCLCPP_INFO(this->get_logger(), "Starting mission...");
  cleaning_in_progress_ = true;
  last_transition_time_ = this->now();
  
  robot_->request_coverage_path(*latest_polygon_, [this](const nav_msgs::msg::Path & full_path) {
    // Special case for ZigZagStrategy: it needs the full path from server
    if (auto zigzag = std::dynamic_pointer_cast<ZigZagStrategy>(strategy_)) {
      zigzag->set_full_path(full_path);
    }
    
    auto pose = robot_->get_pose();
    if (pose) {
      auto segment = strategy_->select_next_path(pose->first, pose->second);
      if (!segment.poses.empty()) {
        robot_->follow_path(segment, 
          [this](bool success) { cleaning_in_progress_ = !success; },
          [this](float /*dist*/) { 
             auto p = robot_->get_pose();
             if (p) strategy_->update_progress(p->first, p->second);
          }
        );
        robot_->publish_markers(segment);
      }
    }
  });
}

void CoverageClientNode::perform_path_transition()
{
  transitioning_ = true;
  RCLCPP_INFO(this->get_logger(), "Obstacle! Searching for next available path...");
  
  robot_->cancel_all_navigation();
  auto pose = robot_->get_pose();
  if (pose) {
    auto segment = strategy_->select_next_path(pose->first, pose->second);
    if (!segment.poses.empty()) {
      last_transition_time_ = this->now();
      
      // Calculate target yaw from segment orientation
      double target_yaw = tf2::getYaw(segment.poses[0].pose.orientation);
      
      RCLCPP_INFO(this->get_logger(), "Path found. Rotating to heading %.2f first.", target_yaw);
      is_rotating_ = true;
      
      robot_->turn_to_heading(target_yaw, [this, segment](bool /*success*/) {
        RCLCPP_INFO(this->get_logger(), "Rotation complete. Starting path following.");
        is_rotating_ = false;
        
        robot_->follow_path(segment, 
          [this](bool success) { cleaning_in_progress_ = !success; },
          [this](float /*dist*/) {
             auto p = robot_->get_pose();
             if (p) strategy_->update_progress(p->first, p->second);
          }
        );
        robot_->publish_markers(segment);
      });
    }
  }
  transitioning_ = false;
}

} // namespace clean_robot_pkg
