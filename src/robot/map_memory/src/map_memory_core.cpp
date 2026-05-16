#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger, int width,
                             int height, double resolution, double origin_x,
                             double origin_y)
    : logger_(logger), width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y),
      global_data_(width * height, -1) {}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &local,
                                     double robot_x, double robot_y,
                                     double robot_yaw) {
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  const int local_w = static_cast<int>(local.info.width);
  const int local_h = static_cast<int>(local.info.height);
  const double local_res = local.info.resolution;
  const double local_ox = local.info.origin.position.x;
  const double local_oy = local.info.origin.position.y;

  for (int j = 0; j < local_h; ++j) {
    for (int i = 0; i < local_w; ++i) {
      int8_t cell = local.data[j * local_w + i];

      // -1 means unknown
      if (cell < 0) {
        continue;
      }

      // Center of this local cell in the lidar frame
      double lx = local_ox + (i + 0.5) * local_res;
      double ly = local_oy + (j + 0.5) * local_res;

      // Rotate by robot yaw, then translate to world frame
      double wx = robot_x + lx * cos_yaw - ly * sin_yaw;
      double wy = robot_y + lx * sin_yaw + ly * cos_yaw;

      // Convert world coords to global grid indices
      int gx = static_cast<int>((wx - origin_x_) / resolution_);
      int gy = static_cast<int>((wy - origin_y_) / resolution_);

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
        continue;
      }

      // new data overrides old
      global_data_[gy * width_ + gx] = cell;
    }
  }
}

nav_msgs::msg::OccupancyGrid
MapMemoryCore::getMapMessage(const rclcpp::Time &stamp,
                             const std::string &frame_id) const {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;

  msg.data.assign(global_data_.begin(), global_data_.end());
  return msg;
}

}  // namespace robot
