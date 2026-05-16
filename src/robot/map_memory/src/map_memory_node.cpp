#include <chrono>
#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"),
      map_memory_(robot::MapMemoryCore(
          this->get_logger(),
          this->declare_parameter<int>("width", 400),
          this->declare_parameter<int>("height", 400),
          this->declare_parameter<double>("resolution", 0.1),
          this->declare_parameter<double>("origin_x", -20.0),
          this->declare_parameter<double>("origin_y", -20.0))) {
  global_frame_ = this->declare_parameter<std::string>("global_frame", "sim_world");
  update_distance_threshold_ =
      this->declare_parameter<double>("update_distance_threshold", 1.5);
  int publish_period_ms = this->declare_parameter<int>("publish_period_ms", 1000);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
}

void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw_);

  has_pose_ = true;
}

void MapMemoryNode::timerCallback() {
  if (!has_pose_ || !latest_costmap_) {
    return;
  }

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  bool moved_enough = std::sqrt(dx * dx + dy * dy) >= update_distance_threshold_;

  if (!has_fused_once_ || moved_enough) {
    map_memory_.integrateCostmap(*latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    has_fused_once_ = true;
  }

  map_pub_->publish(map_memory_.getMapMessage(this->now(), global_frame_));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
