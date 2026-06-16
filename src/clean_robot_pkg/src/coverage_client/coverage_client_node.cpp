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
  localized_(false),
  localization_retries_(0),
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

  logging_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&CoverageClientNode::logging_timer_callback, this));

  // 3. Start Auto Localization Sequence
  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&CoverageClientNode::auto_localize, this));

  RCLCPP_INFO(this->get_logger(), "Coverage Orchestrator initialized (Strategy: %s)", strategy_->get_name().c_str());
}

void CoverageClientNode::auto_localize()
{
  if (localized_) {
    init_timer_->cancel();
    return;
  }

  if (robot_->is_localized()) {
    RCLCPP_INFO(this->get_logger(), "Localization SUCCESS: TF map->base_link found.");
    localized_ = true;
    init_timer_->cancel();
    return;
  }

  if (localization_retries_ < 5) {
    RCLCPP_INFO(this->get_logger(), "Auto-Localization: Try %d/5...", localization_retries_ + 1);
    robot_->set_initial_pose(-2.0, -0.5, 0.0);
    localization_retries_++;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Auto-Localization FAILED after 5 tries. Please check RViz.");
    init_timer_->cancel();
  }
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

void CoverageClientNode::logging_timer_callback()
{
  if (!cleaning_in_progress_) return;

  size_t total = strategy_->get_total_lanes();
  size_t cleaned = strategy_->get_cleaned_count();
  int current = strategy_->get_current_lane_id();
  double percent = (total > 0) ? (static_cast<double>(cleaned) / total * 100.0) : 0.0;

  RCLCPP_INFO(this->get_logger(), 
    "[MISSION STATUS] Cleaned: %zu/%zu (%.1f%%) | Current: #%d", 
    cleaned, total, percent, current + 1);
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
  RCLCPP_INFO(this->get_logger(), "Obstacle detected! Calculating intelligent transition...");
  
  robot_->cancel_all_navigation();
  
  auto current_pose = robot_->get_pose();
  if (current_pose) {
    auto segment = strategy_->select_next_path(current_pose->first, current_pose->second);
    if (segment.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No available path segments found.");
      transitioning_ = false;
      return;
    }

    last_transition_time_ = this->now();
    double target_x = segment.poses[0].pose.position.x;
    double target_y = segment.poses[0].pose.position.y;
    double dist_to_start = std::hypot(target_x - current_pose->first, target_y - current_pose->second);

    // 1. Long Distance Check: Use NavigateToPose if > 1.2m
    if (dist_to_start > 1.2) {
      RCLCPP_INFO(this->get_logger(), "Next lane is far (%.2fm). Using NavigateToPose fallback.", dist_to_start);
      robot_->navigate_to_pose(segment.poses[0], [this, segment](bool success) {
        if (success) {
          RCLCPP_INFO(this->get_logger(), "Reached lane via Navigation. Resuming cleaning...");
          robot_->follow_path(segment, 
             [this](bool success) { cleaning_in_progress_ = !success; },
             [this](float /*dist*/) {
                auto p = robot_->get_pose();
                if (p) strategy_->update_progress(p->first, p->second);
             });
        }
      });
      transitioning_ = false;
      return;
    }

    // 2. Intelligent Rotation Logic (ㄹ shape)
    // Determine if next lane is to the Left (+90) or Right (-90) of current heading
    // Basically calculate relative angle to first waypoint
    try {
      // auto tf = robot_->get_pose(); // simplified, in actual footpirnt we'd use TF listener
      // double current_yaw = 0.0; // We need current yaw to calculate relative side
      // In RobotFacade, get_pose returns only X,Y. Let's assume we can get yaw.
      // Re-implementing rotation logic based on relative position.
      
      double angle_to_wp = std::atan2(target_y - current_pose->second, target_x - current_pose->first);
      
      RCLCPP_INFO(this->get_logger(), "Performing 90 deg step towards next lane.");
      is_rotating_ = true;
      
      robot_->turn_to_heading(angle_to_wp, [this, segment](bool /*success*/) {
        RCLCPP_INFO(this->get_logger(), "Aligned with next lane. Starting follow_path.");
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
    } catch (...) {}
  }
  transitioning_ = false;
}

} // namespace clean_robot_pkg
