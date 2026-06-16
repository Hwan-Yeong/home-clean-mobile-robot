#ifndef CLEAN_ROBOT_PKG__STRATEGIES__I_CLEANING_STRATEGY_HPP_
#define CLEAN_ROBOT_PKG__STRATEGIES__I_CLEANING_STRATEGY_HPP_

#include <memory>
#include <vector>
#include <string>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace clean_robot_pkg
{

/**
 * @brief Interface for cleaning strategies (Strategy Pattern).
 */
class ICleaningStrategy
{
public:
  virtual ~ICleaningStrategy() = default;

  /**
   * @brief Initialize strategies with a new area.
   */
  virtual void set_area(const geometry_msgs::msg::PolygonStamped & polygon) = 0;

  /**
   * @brief Select the next best path based on current robot location.
   * @return The next path segment to follow.
   */
  virtual nav_msgs::msg::Path select_next_path(double rx, double ry) = 0;

  /**
   * @brief Update cleaning progress based on current robot pose.
   */
  virtual void update_progress(double rx, double ry) = 0;

  /**
   * @brief Check if all lanes/areas are cleaned.
   */
  virtual bool is_finished() const = 0;

  virtual std::string get_name() const = 0;
};

} // namespace clean_robot_pkg

#endif  // CLEAN_ROBOT_PKG__STRATEGIES__I_CLEANING_STRATEGY_HPP_
