#include <chrono>
#include <memory>

#include "costmap_node.hpp"

using std::placeholders::_1;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, _1));
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

// Laser callback function
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  // (resolution, rows, columns, max_cost, origin_x_proportion, origin_y_proportion)
  costmap_.initializeCostmap(0.1, 150, 150, 100, 1, -0.5, -0.5);

  // Step 2: Convert LaserScan to grid and mark + inflate obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double range = scan->ranges[i];
      if (range < scan->range_max && range > scan->range_min) {
          // Calculate grid coordinates
          std::vector<int> grid_coords = costmap_.convertToGrid(range, angle);
       
          int x_grid = grid_coords[0];
          int y_grid = grid_coords[1];

          // Mark and inflate obstacles
          costmap_.markAndInflateObstacle(x_grid, y_grid);
      }
  }

  // Step 3: Publish costmap
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid occ_grid = costmap_.populateOccGrid();
  occ_grid.header.stamp = this->get_clock()->now();
  occ_grid.header.frame_id = "sim_world";
  occ_grid_pub_->publish(occ_grid);
  
  // RCLCPP_INFO(this->get_logger(), "Costmap published: %d s, %d nanos", occ_grid.header.stamp.sec, occ_grid.header.stamp.nanosec);

  // RCLCPP_INFO(this->get_logger(), "Publishing new costmap");
  // nav_msgs::msg::OccupancyGrid occ_grid;//.populateOccGrid();
  // RCLCPP_INFO(this->get_logger(), "Initialized");
  // occ_grid.header.stamp = this->get_clock()->now();
  // RCLCPP_INFO(this->get_logger(), "Stamp set");
  // occ_grid.header.frame_id = "map";
  // RCLCPP_INFO(this->get_logger(), "Frame id set");
  // occ_grid.info.resolution = 0.1;
  // RCLCPP_INFO(this->get_logger(), "Resolution set");
  // occ_grid.info.origin = geometry_msgs::msg::Pose();
  // RCLCPP_INFO(this->get_logger(), "Origin set");
  // occ_grid.info.width = 100;
  // RCLCPP_INFO(this->get_logger(), "Width set");
  // occ_grid.info.height = 100;
  // RCLCPP_INFO(this->get_logger(), "Height set");
  
  // occ_grid.data.assign(100 * 100, -1);
  // occ_grid.data[1050] = 100;
  // RCLCPP_INFO(this->get_logger(), "Data set");
  
  // occ_grid_pub_->publish(occ_grid);
  // RCLCPP_INFO(this->get_logger(), "Published successfully");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}