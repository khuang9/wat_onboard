#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    void controlLoop();

    void pathCallback(nav_msgs::msg::Path::SharedPtr path);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool path_received_, odom_received_;
};

#endif
