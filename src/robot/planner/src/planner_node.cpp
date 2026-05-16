#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner_node"),
      planner_(robot::PlannerCore(
          this->get_logger(),
          this->declare_parameter<int>("obstacle_threshold", 50))),
      state_(State::WAITING_FOR_GOAL)
{
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  global_frame_ = this->declare_parameter<std::string>("global_frame", "sim_world");
  int plan_period_ms = this->declare_parameter<int>("plan_period_ms", 500);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(plan_period_ms),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;

      // Publish an empty path to signal control to stop
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = global_frame_;
      path_pub_->publish(empty_path);
    } else {
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  auto waypoints = planner_.planPath(
      current_map_,
      robot_pose_.position.x, robot_pose_.position.y,
      goal_.point.x, goal_.point.y);

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  for (auto &pose : waypoints) {
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = global_frame_;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
