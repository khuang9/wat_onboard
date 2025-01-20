#include "planner_node.hpp"
#include <chrono>

using std::placeholders::_1;

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, _1));
  gp_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::gpCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, _1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  planner_.global_map_ = *map;
  // RCLCPP_INFO(this->get_logger(), "Map received");
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    publishPath();
  }
}

void PlannerNode::gpCallback(geometry_msgs::msg::PointStamped::SharedPtr gp) {
  planner_.goal_ = *gp;
  // RCLCPP_INFO(this->get_logger(), "GOal received");
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  publishPath();
}

void PlannerNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
  //RCLCPP_INFO(this->get_logger(), "Pose received");
  planner_.pose_ = odom->pose.pose;
}

bool PlannerNode::goalReached() {
    double dx = planner_.goal_.point.x - planner_.pose_.position.x;
    double dy = planner_.goal_.point.y - planner_.pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      publishPath();
    }
  }
}
// todo: if time, add h-cost to consideration of next position in path
// // todo: if time, try initialize first map to publish regardless of distance, ie no need to see init position to (-100, -100)
// // todo: if time, try getting map res to 0.2
void PlannerNode::publishPath() {
  if (!goal_received_ || planner_.global_map_.data.empty()) {
    // RCLCPP_INFO(this->get_logger(), "Got: %d", planner_.global_map_.data[30]);
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  // A* Implementation (pseudo-code)
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";
  // RCLCPP_INFO(this->get_logger(), "Header set");
  // Compute path using A* on global_map_
  std::vector<robot::CellIndex> a_path = planner_.aStar();
  // for (int i = -15; i < 15; i++) {
  //   geometry_msgs::msg::PoseStamped p;
  //   p.pose.position.x = i;//ci.x * planner_.global_map_.info.resolution + planner_.global_map_.info.origin.position.x;
  //   p.pose.position.y = 0;//ci.y * planner_.global_map_.info.resolution + planner_.global_map_.info.origin.position.y;
  //   p.header.frame_id = "sim_world";
  //   path.poses.push_back(p);
  // }
  // RCLCPP_INFO(this->get_logger(), "PATH got");
  // Fill path.poses with the resulting waypoints.
  for (robot::CellIndex &ci : a_path) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = planner_.to_coord(ci.x, 0, planner_.global_map_);//ci.x * planner_.global_map_.info.resolution + planner_.global_map_.info.origin.position.x;
    p.pose.position.y = planner_.to_coord(ci.y, 1, planner_.global_map_);//ci.y * planner_.global_map_.info.resolution + planner_.global_map_.info.origin.position.y;
    p.header.frame_id = "sim_world";
    path.poses.push_back(p);
  }
  // RCLCPP_INFO(this->get_logger(), "Path set");
  path_pub_->publish(path);
  // RCLCPP_INFO(this->get_logger(), "Published path");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
