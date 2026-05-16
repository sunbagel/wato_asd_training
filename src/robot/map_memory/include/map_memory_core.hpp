#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot {

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger &logger, int width, int height,
                         double resolution, double origin_x, double origin_y);

  // Fuse a local costmap into the global map
  // robot_x, robot_y, robot_yaw are the robot's pose in the global frame.
  void integrateCostmap(const nav_msgs::msg::OccupancyGrid &local,
                        double robot_x, double robot_y, double robot_yaw);

  nav_msgs::msg::OccupancyGrid getMapMessage(const rclcpp::Time &stamp,
                                             const std::string &frame_id) const;

private:
  rclcpp::Logger logger_;

  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;

  std::vector<int8_t> global_data_;
};

}  // namespace robot

#endif
