#include "control_node.hpp"
#include <chrono>

using std::placeholders::_1;

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, _1));

  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));

  path_received_ = false;
  odom_received_ = false;

  control_.initControlParams(1.0, 0.4, 2);
}

void ControlNode::pathCallback(nav_msgs::msg::Path::SharedPtr path) {
  control_.current_path_ = *path;
  path_received_ = true;
}

void ControlNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
  control_.robot_odom_ = *odom;
  odom_received_ = true;
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!path_received_ || !odom_received_) {
    return;
  }

  // Find the lookahead point
  auto lookahead_point = control_.findLookaheadPoint();
  if (!lookahead_point) {
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = control_.computeVelocity(*lookahead_point);

  // Publish the velocity command
  vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
