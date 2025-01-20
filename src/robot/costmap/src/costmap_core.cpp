#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(double rs, int r, int c, int mc, double ir, double x_rat, double y_rat) {
    res_ = rs;
    rows_ = r;
    cols_ = c;
    max_cost_ = mc;
    inf_rad_ = ir;
    origin_.position.x = cols_ * res_ * x_rat;
    origin_.position.y = rows_ * res_ * y_rat;
    
    vals_.resize(rows_);
    for (std::vector<double> &i : vals_) {
        i.resize(cols_);
    }
}

std::vector<int> CostmapCore::convertToGrid(double range, double angle) {
    std::vector<int> coords = {0, 0};
    coords[0] = std::floor((range * std::cos(angle) - origin_.position.x) / res_);
    coords[1] = std::floor((range * std::sin(angle) - origin_.position.y) / res_);
    return coords;
}

void CostmapCore::markAndInflateObstacle(int x, int y) {
    if (0 <= x && x < cols_ && 0 <= y && y < rows_) {
        vals_[y][x] = max_cost_;

        double ir_grid = inf_rad_ / res_;

        for (int i = y - ir_grid; i <= y + ir_grid; i++) {
            for(int j = x - ir_grid; j <= x + ir_grid; j++) {
                if (0 <= j && j < cols_ && 0 <= i && i < rows_) {
                    double dx = (x - j) * res_;
                    double dy = (y - i) * res_;

                    double d = std::sqrt(dx*dx + dy*dy);

                    double cost = max_cost_ * (1 - d/inf_rad_);
                    
                    if (cost > vals_[i][j]) {
                        vals_[i][j] = cost;
                    }
                }
            }

        }
    }
}

nav_msgs::msg::OccupancyGrid CostmapCore::populateOccGrid() {   
    nav_msgs::msg::OccupancyGrid occ_grid;

    occ_grid.info.resolution = res_;
    occ_grid.info.origin = origin_;
    occ_grid.info.width = cols_;
    occ_grid.info.height = rows_;
    
    occ_grid.data.assign(rows_ * cols_, -1);
    
    for (int i = 0; i < rows_; i++) {
        for (int j = 0; j < cols_; j++) {
            occ_grid.data[i * cols_ + j] = vals_[i][j];
            vals_[i][j] = 0; // Reset vals_ to 0
        }
    }

    return occ_grid;
}
}