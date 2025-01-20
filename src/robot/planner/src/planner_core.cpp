#include "planner_core.hpp"
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

double PlannerCore::calc_f(CellIndex cell, CellIndex from, double f1) {
    double cell_x = to_coord(cell.x, 0, global_map_);//j * global_map_.info.resolution + global_map_.info.origin.position.x;
    double cell_y = to_coord(cell.y, 1, global_map_);//i * global_map_.info.resolution + global_map_.info.origin.position.y;
    
    double from_x = to_coord(from.x, 0, global_map_);
    double from_y = to_coord(from.y, 1, global_map_);

    // double dx_rbt = cell_x - pose_.position.x;
    // double dy_rbt = cell_y - pose_.position.y;
    // double g = std::sqrt(dx_rbt*dx_rbt + dy_rbt*dy_rbt);
    double h1 = euc_dist(from_x, from_y, goal_.point.x, goal_.point.y);
    double g1 = f1 - h1;
    double dg = euc_dist(from_x, from_y, cell_x, cell_y);
    double g2 = g1 + dg;

    // double dx_gl = cell_x - goal_.point.x;
    // double dy_gl = cell_y - goal_.point.y;
    // double h = std::sqrt(dx_gl*dx_gl + dy_gl*dy_gl);
    double h2 = euc_dist(cell_x, cell_y, goal_.point.x, goal_.point.y);

    // goal_.point refers to indices or coord?
    return g2 + h2;
}

void PlannerCore::updateCells(std::unordered_map<CellIndex, std::shared_ptr<AStarNode>, CellIndexHash> &cells, std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareF> &nodes, CellIndex curr_ind) {
    nodes.pop();

    if (cells[curr_ind]->f_score < 0) {
        return;
    }

    for (int i = curr_ind.y - 1; i <= curr_ind.y + 1; i++) {
        for (int j = curr_ind.x - 1; j <= curr_ind.x + 1; j++) {
            // RCLCPP_INFO(logger_, "i: %d, j: %d", i, j);
            if (0 <= i && i < global_map_.info.height && 0 <= j && j < global_map_.info.width && !(i == curr_ind.y && j == curr_ind.x)) {
                // RCLCPP_INFO(logger_, "data: %.3f", global_map_.data[i * global_map_.info.width + j]);
                double cost = global_map_.data[i * global_map_.info.width + j];
                // if (cost <= 0.0) {
                CellIndex ind = CellIndex(j, i);
                double f = calc_f(ind, curr_ind, cells[curr_ind]->f_score);
                // RCLCPP_INFO(logger_, "f: %.3f", f);
                if (cells.find(ind) == cells.end()) {
                    cells[ind] = std::make_shared<AStarNode>(AStarNode(ind, f, cost, cells[curr_ind]));
                    nodes.push(cells[ind]);
                    // cells[ind] = nodes[nodes.size() - 1];
                } else if (f < cells[ind]->f_score) {
                    cells[ind]->f_score = f;
                    cells[ind]->obs_cost = cost;
                    cells[ind]->from = cells[curr_ind];
                }
                
                    // cells[curr_ind]->f_score = -1;
                // }
            }
        }
    }
}

bool PlannerCore::pathDone(int i, int j) {
    double dx = goal_.point.x - to_coord(j, 0, global_map_);//j * global_map_.info.resolution + global_map_.info.origin.position.x;
    double dy = goal_.point.y - to_coord(i, 1, global_map_);//i * global_map_.info.resolution + global_map_.info.origin.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.2;
}

std::vector<CellIndex> PlannerCore::aStar() {
    // int goal_j = to_index(goal_.point.x, 0, global_map_);//std::floor((goal_.point.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
    // int goal_i = to_index(goal_.point.y, 1, global_map_);//std::floor((goal_.point.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
    // CellIndex goal_ind = CellIndex(goal_j, goal_i);
    // RCLCPP_INFO(logger_, "Goal: (%d, %d)", goal_i, goal_j);

    // Initialize
    std::unordered_map<CellIndex, std::shared_ptr<AStarNode>, CellIndexHash> cells;
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareF> nodes;
    int curr_j = to_index(pose_.position.x, 0, global_map_);//std::floor((pose_.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
    int curr_i = to_index(pose_.position.y, 1, global_map_);//std::floor((pose_.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
    
    CellIndex curr_ind = CellIndex(curr_j, curr_i);
    // RCLCPP_INFO(logger_, "Curr: (%d, %d)", curr_i, curr_j);
    // maybe nullptr causes error?
    cells[curr_ind] = std::make_shared<AStarNode>(AStarNode(curr_ind, 0, 0, nullptr));
    nodes.push(cells[curr_ind]);
    // cells[curr_ind] = nodes[0];
    // RCLCPP_INFO(logger_, "Set nullptr");

    updateCells(cells, nodes, curr_ind);
    // RCLCPP_INFO(logger_, "First update");
    // for (int i = 0; i < global_map_.info.height; i++) {
    //     for (int j = 0; j < global_map_.info.width; j++) {
    //         if (global_map_.data[i][j] == 0) {
    //             double f = calc_f(i, j);
    //             CellIndex ind = CellIndex(j, i);

    //             cells[ind] = AStarNode(ind, f);
    //         }
    //     }
    // }
    
    while (!pathDone(curr_i, curr_j)) { // curr_ind != goal_ind
        // RCLCPP_INFO(logger_, "at: row %d, col %d, f %.3f, prev %d, %d", curr_i, curr_j, cells[CellIndex(curr_j, curr_i)]->f_score, cells[CellIndex(curr_j, curr_i)]->index.x, cells[CellIndex(curr_j, curr_i)]->index.y);
        // std::sort(nodes.begin(), nodes.end(), CompareF());
        // int s = nodes.size();
        // while (s > 0 && nodes[s - 1]->f_score <= 0) {
        //     nodes.pop_back();
        //     s--;
        // }
        // RCLCPP_INFO(logger_, "first: %.3f", nodes[0]->f_score);
        if (nodes.size() == 0) {
            return {};
        }

        curr_i = nodes.top()->index.y;
        curr_j = nodes.top()->index.x;
        updateCells(cells, nodes, nodes.top()->index);
    }
    // RCLCPP_INFO(logger_, "path found");

    curr_ind.x = curr_j;
    curr_ind.y = curr_i;

    std::shared_ptr<AStarNode> curr_node = cells[curr_ind];
    std::vector<CellIndex> path = {curr_ind};
    // RCLCPP_INFO(logger_, "Backtracker set, next: %d, %d");

    while (curr_node->from != nullptr) {
        // RCLCPP_INFO(logger_, "curr_at: x - %d, y - %d", curr_node->index.x, curr_node->index.y);
        // RCLCPP_INFO(logger_, "next: x - %d, y - %d", curr_node->from->index.x, curr_node->from->index.y);
        // RCLCPP_INFO(logger_, "next next: x - %d, y - %d", curr_node->from->from->index.x, curr_node->from->from->index.y);
        path.push_back(curr_node->from->index);
        curr_node = curr_node->from;
    }
    // RCLCPP_INFO(logger_, "Backtracking done");

    std::reverse(path.begin(), path.end());
    // RCLCPP_INFO(logger_, "Reversed");
    return path;

}

int PlannerCore::to_index(double coord, int xy, nav_msgs::msg::OccupancyGrid &grid) {
    if (xy == 0) {
        return std::floor((coord - grid.info.origin.position.x) / grid.info.resolution);
    } else {
        return std::floor((coord - grid.info.origin.position.y) / grid.info.resolution);
    }
}

double PlannerCore::to_coord(int index, int xy, nav_msgs::msg::OccupancyGrid &grid) {
    if (xy == 0) {
        return index * grid.info.resolution + grid.info.origin.position.x;
    } else {
        return index * grid.info.resolution + grid.info.origin.position.y;
    }
}

double PlannerCore::euc_dist(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

} 

