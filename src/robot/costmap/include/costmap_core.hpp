#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(double rs, int r, int c, int mc, double ir, double ox, double oy);

    std::vector<int> convertToGrid(double range, double angle);

    void markAndInflateObstacle(int x, int y);

    nav_msgs::msg::OccupancyGrid populateOccGrid();
    

  private:
    rclcpp::Logger logger_;

    std::vector<std::vector<double>> vals_;
    double res_;
    int rows_;
    int cols_;
    int max_cost_;
    double inf_rad_;
    geometry_msgs::msg::Pose origin_;



};

}  

#endif  