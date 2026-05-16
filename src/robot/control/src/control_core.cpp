#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger & logger)
: logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & robot_pos,
  double lookahead_distance) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  for (const auto & pose : path.poses) {
    // get first pose in lookahead distance
    if (distance(pose.pose.position, robot_pos) >= lookahead_distance) {
      return pose;
    }
  }

  // head for the last pose
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::Point & robot_pos,
  double robot_yaw,
  double linear_speed,
  double lookahead_distance,
  double max_angular_speed) const
{
  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;

  // Angle from world x-axis to the lookahead point, then expressed relative to
  // the robot's heading.
  double alpha = std::atan2(dy, dx) - robot_yaw;

  // Normalize to [-pi, pi]
  while (alpha > M_PI)  alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed;
  cmd.angular.z = std::clamp(
    2.0 * linear_speed * std::sin(alpha) / lookahead_distance,
    -max_angular_speed, max_angular_speed);

  return cmd;
}

double ControlCore::distance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace robot
