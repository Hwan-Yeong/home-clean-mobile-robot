#include "clean_robot_pkg/map_to_polygon_node.hpp"

#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point32.hpp"

namespace clean_robot_pkg
{

MapToPolygonNode::MapToPolygonNode(const rclcpp::NodeOptions & options)
: Node("map_to_polygon_node", options)
{
  // Parameters
  this->declare_parameter("free_threshold", 50);
  this->declare_parameter("contour_simplify_epsilon", 0.02);
  this->declare_parameter("min_area_ratio", 0.05);
  this->declare_parameter("publish_rate", 1.0);

  free_threshold_ = this->get_parameter("free_threshold").as_int();
  simplify_epsilon_ = this->get_parameter("contour_simplify_epsilon").as_double();
  min_area_ratio_ = this->get_parameter("min_area_ratio").as_double();

  // QoS for map topic
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .transient_local()
    .reliable();

  // Subscriber
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    map_qos,
    std::bind(&MapToPolygonNode::map_callback, this, std::placeholders::_1));

  // Publisher
  polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/cleaning_zone", 10);

  // Timer for periodic publication
  double publish_rate = this->get_parameter("publish_rate").as_double();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
    std::bind(&MapToPolygonNode::publish_polygon, this));

  RCLCPP_INFO(this->get_logger(), "MapToPolygonNode initialized.");
}

void MapToPolygonNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  int width = msg->info.width;
  int height = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  cv::Mat free_mask = cv::Mat::zeros(height, width, CV_8UC1);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int8_t val = msg->data[y * width + x];
      if (val >= 0 && val <= free_threshold_) {
        free_mask.at<uint8_t>(y, x) = 255;
      }
    }
  }

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(free_mask, free_mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(free_mask, free_mask, cv::MORPH_OPEN, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(free_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    RCLCPP_WARN(this->get_logger(), "No free-space contours found.");
    return;
  }

  auto largest_it = std::max_element(contours.begin(), contours.end(),
    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
      return cv::contourArea(a) < cv::contourArea(b);
    });

  double total_area = static_cast<double>(width) * height;
  double contour_area = cv::contourArea(*largest_it);

  if (contour_area / total_area < min_area_ratio_) {
    return;
  }

  std::vector<cv::Point> simplified;
  double perimeter = cv::arcLength(*largest_it, true);
  cv::approxPolyDP(*largest_it, simplified, simplify_epsilon_ * perimeter, true);

  auto polygon_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>();
  polygon_msg->header.frame_id = "map";
  polygon_msg->header.stamp = this->now();

  for (const auto& pt : simplified) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(origin_x + pt.x * resolution);
    p.y = static_cast<float>(origin_y + pt.y * resolution);
    p.z = 0.0f;
    polygon_msg->polygon.points.push_back(p);
  }

  if (!polygon_msg->polygon.points.empty()) {
    const auto& first = polygon_msg->polygon.points.front();
    const auto& last = polygon_msg->polygon.points.back();
    if (std::abs(first.x - last.x) > 1e-4 || std::abs(first.y - last.y) > 1e-4) {
      polygon_msg->polygon.points.push_back(first);
    }
  }

  latest_polygon_ = polygon_msg;
  RCLCPP_DEBUG(this->get_logger(), "Polygon updated.");
}

void MapToPolygonNode::publish_polygon()
{
  if (latest_polygon_) {
    latest_polygon_->header.stamp = this->now();
    polygon_pub_->publish(*latest_polygon_);
  }
}

} // namespace clean_robot_pkg
