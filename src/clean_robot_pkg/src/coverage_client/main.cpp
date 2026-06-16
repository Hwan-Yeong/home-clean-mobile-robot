#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "clean_robot_pkg/coverage_client_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<clean_robot_pkg::CoverageClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
