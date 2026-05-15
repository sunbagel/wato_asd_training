#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cmath>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot {

class CostmapCore {
public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
  explicit CostmapCore(const rclcpp::Logger& logger);

  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  nav_msgs::msg::OccupancyGrid
  getCostmapMessage(const std_msgs::msg::Header &header);

private:
  rclcpp::Logger logger_;

  std::vector<int8_t> costmap_data_;
  std::vector<std::pair<int, int>> obstacle_cells_;
  int width_;
  int height_;
  double resolution_;
  double inflation_radius_;
};

} // namespace robot

#endif