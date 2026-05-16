#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger &logger, int obstacle_threshold,
                         double inflation_cost_weight, double unknown_cell_cost)
    : logger_(logger), obstacle_threshold_(obstacle_threshold),
      inflation_cost_weight_(inflation_cost_weight), unknown_cell_cost_(unknown_cell_cost) {}

// helpers
bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                              double wx, double wy, int &cx, int &cy) const {
  const double res = map.info.resolution;
  cx = static_cast<int>((wx - map.info.origin.position.x) / res);
  cy = static_cast<int>((wy - map.info.origin.position.y) / res);
  return cx >= 0 && cx < static_cast<int>(map.info.width) &&
         cy >= 0 && cy < static_cast<int>(map.info.height);
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                              int cx, int cy, double &wx, double &wy) const {
  const double res = map.info.resolution;
  wx = map.info.origin.position.x + (cx + 0.5) * res;
  wy = map.info.origin.position.y + (cy + 0.5) * res;
}

bool PlannerCore::isBlocked(const nav_msgs::msg::OccupancyGrid &map,
                            int cx, int cy) const {
  // treat unknown cells (-1) as traversable, so that we can update costmap as we explore
  int8_t val = map.data[cy * static_cast<int>(map.info.width) + cx];
  return val >= obstacle_threshold_;
}

double PlannerCore::euclideanDistance(const CellIndex &a, const CellIndex &b) {
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid &map,
    double sx, double sy,
    double gx, double gy) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  int start_cx, start_cy, goal_cx, goal_cy;
  if (!worldToGrid(map, sx, sy, start_cx, start_cy)) {
    RCLCPP_WARN(logger_, "Start position is outside the map.");
    return {};
  }
  if (!worldToGrid(map, gx, gy, goal_cx, goal_cy)) {
    RCLCPP_WARN(logger_, "Goal position is outside the map.");
    return {};
  }
  if (isBlocked(map, goal_cx, goal_cy)) {
    RCLCPP_WARN(logger_, "Goal cell is blocked.");
    return {};
  }

  // If robot stuck near wall, snap the start to the nearest free cell
  if (isBlocked(map, start_cx, start_cy)) {
    bool found = false;
    for (int radius = 1; radius <= 10 && !found; ++radius) {
      for (int dy = -radius; dy <= radius && !found; ++dy) {
        for (int dx = -radius; dx <= radius && !found; ++dx) {
          if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
          int nx = start_cx + dx, ny = start_cy + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          if (!isBlocked(map, nx, ny)) {
            start_cx = nx;
            start_cy = ny;
            found = true;
          }
        }
      }
    }
    if (!found) {
      RCLCPP_WARN(logger_, "Start cell is blocked and no free cell found nearby.");
      return {};
    }
  }

  CellIndex start(start_cx, start_cy);
  CellIndex goal(goal_cx, goal_cy);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open_set.emplace(start, euclideanDistance(start, goal));

  // 8-connected neighbors: (dx, dy, step_cost)
  const int dx[8] = {1, -1, 0,  0,  1,  1, -1, -1};
  const int dy[8] = {0,  0, 1, -1,  1, -1,  1, -1};
  const double step_cost[8] = {1.0, 1.0, 1.0, 1.0,
                                M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      // Reconstruct path
      std::vector<geometry_msgs::msg::PoseStamped> path;
      CellIndex idx = goal;
      
      while (idx != start) {
        geometry_msgs::msg::PoseStamped pose;
        gridToWorld(map, idx.x, idx.y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;

        path.push_back(pose);
        idx = came_from.at(idx);
      }

      geometry_msgs::msg::PoseStamped start_pose;
      gridToWorld(map, start.x, start.y, start_pose.pose.position.x, start_pose.pose.position.y);
      start_pose.pose.orientation.w = 1.0;
      path.push_back(start_pose);

      std::reverse(path.begin(), path.end());
      return path;
    }

    double g_curr = g_score.count(current.index) ? g_score[current.index]
                                                  : std::numeric_limits<double>::infinity();

    // for each neighbour                                               
    for (int i = 0; i < 8; ++i) {
      int nx = current.index.x + dx[i];
      int ny = current.index.y + dy[i];

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (isBlocked(map, nx, ny)) {
        continue;
      }

      CellIndex neighbor(nx, ny);
      // Add penalty for inflated/unknown cells so we don't ride too close to walls
      int8_t cell_val = map.data[ny * width + nx];
      double cell_cost = 0.0;
      if (cell_val < 0) {
        cell_cost = unknown_cell_cost_;
      } else if (cell_val > 0) {
        cell_cost = (static_cast<double>(cell_val) / 100.0) * inflation_cost_weight_;
      }
      double tentative_g = g_curr + step_cost[i] + cell_cost;

      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        // insert into open set
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current.index;
        open_set.emplace(neighbor, tentative_g + euclideanDistance(neighbor, goal));
      }
    }
  }

  RCLCPP_WARN(logger_, "A* failed to find a path to goal.");
  return {};
}

}  // namespace robot
