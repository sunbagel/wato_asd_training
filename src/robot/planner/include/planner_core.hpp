#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

// --------------------------------------------------------------

namespace robot
{

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger &logger, int obstacle_threshold,
                       double inflation_cost_weight, double unknown_cell_cost);

  // Plan a path from world (sx, sy) to world (gx, gy) using A* on map.
  // Returns an ordered list of PoseStamped waypoints (in the map frame),
  // or an empty vector if no path is found.
  std::vector<geometry_msgs::msg::PoseStamped> planPath(
      const nav_msgs::msg::OccupancyGrid &map,
      double sx, double sy,
      double gx, double gy) const;

private:
  bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                   double wx, double wy, int &cx, int &cy) const;
  void gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                   int cx, int cy, double &wx, double &wy) const;
  bool isBlocked(const nav_msgs::msg::OccupancyGrid &map, int cx, int cy) const;
  static double euclideanDistance(const CellIndex &a, const CellIndex &b);

  rclcpp::Logger logger_;
  int obstacle_threshold_;
  double inflation_cost_weight_;
  double unknown_cell_cost_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
