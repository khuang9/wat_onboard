#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector> // may switch
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Place callback function here
    void publishMessage();

    // Laser callback function
    void laserCallback(sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Publishing costmap
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;

    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
};

#endif 