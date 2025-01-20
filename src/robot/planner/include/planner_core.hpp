#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <unordered_map>
#include <queue>
#include <vector>
#include <memory>

namespace robot
{

// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score, obs_cost;  // f = g + h
  std::shared_ptr<AStarNode> from;

  AStarNode(CellIndex idx, double f, double c, std::shared_ptr<AStarNode> fr) : index(idx), f_score(f), obs_cost(c), from(fr) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const std::shared_ptr<AStarNode> &a, const std::shared_ptr<AStarNode> &b)
  {
    // We want the node with the smallest f_score on top
    // double a_cost = global_map_[a->index.y * global_map_.info.width + a->index.x];
    // double b_cost = global_map_[b->index.y * global_map_.info.width + b->index.x];
    
    return a->f_score + a->obs_cost > b->f_score + b->obs_cost;

  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    bool pathDone(int i, int j);

    double calc_f(CellIndex cell, CellIndex from, double f1);
    void updateCells(std::unordered_map<CellIndex, std::shared_ptr<AStarNode>, CellIndexHash> &cells, std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareF> &nodes, CellIndex curr_ind);
    std::vector<CellIndex> aStar();

    int to_index(double coord, int xy, nav_msgs::msg::OccupancyGrid &grid);
    double to_coord(int index, int xy, nav_msgs::msg::OccupancyGrid &grid);

    double euc_dist(double x1, double y1, double x2, double y2);

    nav_msgs::msg::OccupancyGrid global_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose pose_;

  private:
    rclcpp::Logger logger_;

    const double cost_threshold = 20; // Max cost a cell can have before being considered an obstacle

};

}  

#endif  
