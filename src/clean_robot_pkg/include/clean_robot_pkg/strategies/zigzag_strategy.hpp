#ifndef CLEAN_ROBOT_PKG__STRATEGIES__ZIGZAG_STRATEGY_HPP_
#define CLEAN_ROBOT_PKG__STRATEGIES__ZIGZAG_STRATEGY_HPP_

#include "clean_robot_pkg/strategies/i_cleaning_strategy.hpp"
#include "clean_robot_pkg/coverage_path_manager.hpp"

namespace clean_robot_pkg
{

class ZigZagStrategy : public ICleaningStrategy
{
public:
  explicit ZigZagStrategy(const rclcpp::Logger & logger);

  void set_area(const geometry_msgs::msg::PolygonStamped & polygon) override;
  void set_full_path(const nav_msgs::msg::Path & path);
  
  nav_msgs::msg::Path select_next_path(double rx, double ry) override;
  void update_progress(double rx, double ry) override;
  bool is_finished() const override;
  std::string get_name() const override { return "ZigZag"; }

private:
  std::unique_ptr<CoveragePathManager> path_manager_;
  bool area_initialized_;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__STRATEGIES__ZIGZAG_STRATEGY_HPP_
