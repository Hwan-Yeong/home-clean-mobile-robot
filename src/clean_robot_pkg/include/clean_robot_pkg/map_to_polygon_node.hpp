#ifndef CLEAN_ROBOT_PKG__MAP_TO_POLYGON_NODE_HPP_
#define CLEAN_ROBOT_PKG__MAP_TO_POLYGON_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace clean_robot_pkg
{

class MapToPolygonNode : public rclcpp::Node
{
public:
  explicit MapToPolygonNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish_polygon();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PolygonStamped::SharedPtr latest_polygon_;

  int free_threshold_;
  double simplify_epsilon_;
  double min_area_ratio_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__MAP_TO_POLYGON_NODE_HPP_
