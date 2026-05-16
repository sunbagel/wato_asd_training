#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger &logger)
    : logger_(logger), width_(400), height_(400), resolution_(0.1),
      inflation_radius_(1.2) {}

void CostmapCore::initializeCostmap() {
  costmap_data_.assign(width_ * height_, 0);
  obstacle_cells_.clear();
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid,
                                int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>((x / resolution_) + (width_ / 2));
  y_grid = static_cast<int>((y / resolution_) + (height_ / 2));
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    costmap_data_[y_grid * width_ + x_grid] = 100;
    obstacle_cells_.emplace_back(x_grid, y_grid); // mark obstacles
  }
}

void CostmapCore::inflateObstacles() {

  int radius_cells = inflation_radius_ / resolution_;

  for (auto &[ox, oy] : obstacle_cells_) {

    // For each obstacle, inflate cost of cells within the radius
    int y_min = std::max(0, oy - radius_cells);
    int y_max = std::min(height_ - 1, oy + radius_cells);
    int x_min = std::max(0, ox - radius_cells);
    int x_max = std::min(width_ - 1, ox + radius_cells);

    for (int y = y_min; y <= y_max; ++y) {
      for (int x = x_min; x <= x_max; ++x) {
        double dist =
            std::sqrt((x - ox) * (x - ox) + (y - oy) * (y - oy)) * resolution_;
        if (dist > inflation_radius_)
          continue;

        // Inflated cost
        int cost = static_cast<int>(100.0 * (1.0 - dist / inflation_radius_));
        int idx = y * width_ + x;
        if (cost > costmap_data_[idx]) {
          costmap_data_[idx] = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getCostmapMessage(const std_msgs::msg::Header &header) {
  
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.origin.position.x = -(width_ * resolution_) / 2.0;
  grid.info.origin.position.y = -(height_ * resolution_) / 2.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.assign(costmap_data_.begin(), costmap_data_.end());
  return grid;
}

} // namespace robot