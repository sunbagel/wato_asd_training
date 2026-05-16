#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
public:
  explicit ControlCore(const rclcpp::Logger & logger);

  // Returns the first point on path within lookahead distance
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & robot_pos,
    double lookahead_distance) const;

  // Pure-pursuit velocity
  geometry_msgs::msg::Twist computeVelocity(
    const geometry_msgs::msg::PoseStamped & target,
    const geometry_msgs::msg::Point & robot_pos,
    double robot_yaw,
    double linear_speed,
    double lookahead_distance,
    double max_angular_speed) const;

  // 2-D Euclidean distance 
  double distance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) const;

  // Extracts yaw from a quaternion
  double extractYaw(const geometry_msgs::msg::Quaternion & q) const;

private:
  rclcpp::Logger logger_;
};

}  // namespace robot

#endif
