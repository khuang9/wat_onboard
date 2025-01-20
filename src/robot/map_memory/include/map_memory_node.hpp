#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void initializeMap(double res, double w, double h, double ox, double oy);

    void costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom);
    void velCallback(geometry_msgs::msg::Twist::SharedPtr vel);

    void publishMap();
    void integrateCostmap();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    std::vector<int> times_seen_;
    const double distance_threshold = 2;
    const double z_expect = 0.9; // z of robot on ground
    const double max_z_var = 0.1; // + or - 0.1 m from 0.9 m
    const double max_roll_var = M_PI/72; // + or - 2.5 degrees
    const double max_pitch_var = M_PI/72; // + or - 2.5 degrees
    const double recency_bias = 0; // How much new costmap data valued compared to old data in the case a cell's cost is less than previously calculated
    const int times_seen_effectiveness = 5;
    const double max_ang_spd = M_PI;
    double prev_x, prev_y, prev_yaw;
    double curr_x, curr_y, curr_yaw;
    double prev_roll, prev_pitch, prev_z;
    bool new_costmap_available_;
    double ang_vel;
    double lin_vel;

    double latest_costmap_time_, latest_odom_time_;//std::chrono::steady_clock::time_point

    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_;

};

#endif 
