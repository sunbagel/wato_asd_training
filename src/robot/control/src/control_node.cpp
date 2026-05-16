#include <chrono>
#include <cmath>

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control_node"),
  control_(robot::ControlCore(this->get_logger()))
{
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
  goal_tolerance_     = this->declare_parameter<double>("goal_tolerance", 0.3);
  linear_speed_       = this->declare_parameter<double>("linear_speed", 0.5);
  max_angular_speed_  = this->declare_parameter<double>("max_angular_speed", 2.5);
  in_place_turn_threshold_ = this->declare_parameter<double>("in_place_turn_threshold", M_PI / 2.0);
  int control_period_ms = this->declare_parameter<int>("control_period_ms", 100);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms),
    std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    publishZeroTwist();
    return;
  }

  const auto & robot_pos = robot_odom_->pose.pose.position;
  const auto & goal_pos  = current_path_->poses.back().pose.position;

  if (control_.distance(robot_pos, goal_pos) < goal_tolerance_) {
    publishZeroTwist();
    return;
  }

  auto lookahead = control_.findLookaheadPoint(
    *current_path_, robot_pos, lookahead_distance_);
  if (!lookahead) {
    publishZeroTwist();
    return;
  }

  double yaw = control_.extractYaw(robot_odom_->pose.pose.orientation);
  auto cmd = control_.computeVelocity(
    *lookahead, robot_pos, yaw,
    linear_speed_, lookahead_distance_, max_angular_speed_,
    in_place_turn_threshold_);

  cmd_vel_pub_->publish(cmd);
}

void ControlNode::publishZeroTwist() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
