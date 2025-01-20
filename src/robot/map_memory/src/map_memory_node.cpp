#include "map_memory_node.hpp"
#include <chrono>

using std::placeholders::_1;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  cost_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, _1));
  // vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MapMemoryNode::velCallback, this, _1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));

  initializeMap(0.2, 150, 150, -0.5, -0.5);
}

void MapMemoryNode::initializeMap(double res, double w, double h, double x_rat, double y_rat) {
  global_map_.header.stamp = this->get_clock()->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = res;
  global_map_.info.width = w;
  global_map_.info.height = h;
  global_map_.info.origin = geometry_msgs::msg::Pose();
  global_map_.info.origin.position.x = res * w * x_rat;
  global_map_.info.origin.position.y = res * h * y_rat;

  global_map_.data.assign(w*h, -1);
  times_seen_.assign(w*h, 0);

  prev_y = res * h / 2 + distance_threshold + 1;
  new_costmap_available_ = false;
  should_update_map_ = false;

}

void MapMemoryNode::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  // Store new costmap for later
  latest_costmap_time_ = costmap->header.stamp.sec + 1e-9 * costmap->header.stamp.nanosec;//std::chrono::steady_clock::now();;

  latest_costmap_ = *costmap;
  new_costmap_available_ = true;
}

void MapMemoryNode::velCallback(geometry_msgs::msg::Twist::SharedPtr vel) {
  ang_vel = vel->angular.z;
  lin_vel = vel->linear.x;
}

void MapMemoryNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
  latest_odom_time_ = odom->header.stamp.sec + 1e-9 * odom->header.stamp.nanosec;//std::chrono::steady_clock::now();
  lin_vel = odom->twist.twist.linear.x;
  ang_vel = odom->twist.twist.angular.z;

  curr_x = odom->pose.pose.position.x;
  curr_y = odom->pose.pose.position.y;

  double dx = curr_x - prev_x;
  double dy = curr_y - prev_y;
  double dxy = std::sqrt(dx*dx + dy*dy);

  double qx = odom->pose.pose.orientation.x;
  double qy = odom->pose.pose.orientation.y;
  double qz = odom->pose.pose.orientation.z;
  double qw = odom->pose.pose.orientation.w;

  curr_yaw = std::atan2(2*(qx*qy + qz*qw), 1 - 2*(qy*qy + qz*qz));

  // might switch to eigen lib
  double roll = std::atan2(2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy));
  double pitch = std::atan2(-2*(qx*qz - qy*qw), std::sqrt(std::pow(2*(qy*qz + qx*qw), 2) + std::pow(1 - 2*(qx*qx + qy*qy), 2)));
  double z = odom->pose.pose.position.z;

  double roll_var = std::abs(roll);
  double pitch_var = std::abs(pitch);
  double z_var = std::abs(z - z_expect);

  double prev_roll_var = std::abs(prev_roll);
  double prev_pitch_var = std::abs(prev_pitch);
  double prev_z_var = std::abs(prev_z - z_expect);

  prev_roll = roll;
  prev_pitch = pitch;
  prev_z = z;

  // RCLCPP_INFO(this->get_logger(), "new x: %.3f", x);
  // RCLCPP_INFO(this->get_logger(), "new y: %.3f", y);
  // RCLCPP_INFO(this->get_logger(), "old x: %.3f", prev_x);
  // RCLCPP_INFO(this->get_logger(), "old y: %.3f", prev_y);
  // RCLCPP_INFO(this->get_logger(), "distance: %.3f", d);
  // RCLCPP_INFO(this->get_logger(), "theta: %.3f", 2*std::acos(qw));
  // RCLCPP_INFO(this->get_logger(), "d: %.3f, z: %.3f, r: %.3f, y: %.3f", dxy, z_var, roll_var, pitch_var);
  if (dxy >= distance_threshold && z_var <= max_z_var && roll_var <= max_roll_var && pitch_var <= max_pitch_var
                                && prev_z_var <= max_z_var && prev_roll_var <= max_roll_var && prev_pitch_var <= max_pitch_var && std::abs(ang_vel) <= max_ang_spd) {
    prev_x = curr_x;
    prev_y = curr_y;
    prev_yaw = curr_yaw;//2 * std::acos(qw / std::sqrt(qw*qw + qz*qz)); // Assuming robot only rotates around z-axis
    //RCLCPP_INFO(this->get_logger(), "qw: %.3f, yaw: %.3f", qw, prev_yaw);
    should_update_map_ = true;
  }
}

void MapMemoryNode::publishMap() {
  if (should_update_map_ && new_costmap_available_) {
    integrateCostmap();
    // RCLCPP_INFO(this->get_logger(), "yaw: %.3f", curr_yaw);
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    // RCLCPP_INFO(this->get_logger(), "New global map published successfully");
  }

}

void MapMemoryNode::integrateCostmap() {
  // Account for velocities
  double est_x, est_y, est_yaw;
  double dt_sec = latest_costmap_time_ - latest_odom_time_;//std::chrono::duration_cast<std::chrono::seconds>(latest_costmap_time_ - latest_odom_time_).count();
  // est_x = curr_x + std::cos() * lin_vel * dt_sec;
  // esy_y = curr_y + lin_v
  est_yaw = curr_yaw + ang_vel * dt_sec;
  // RCLCPP_INFO(this->get_logger(), "curr: %.3f, est: %.3f, vel: %.3f, dt_sec: %.3f", 180 * curr_yaw / M_PI, 180 * est_yaw / M_PI, 180 * ang_vel / M_PI, dt_sec);
  
  for (int r = 0; r < latest_costmap_.info.height; r++) {
    for (int c = 0; c < latest_costmap_.info.width; c++) {
      // if (r == 50 && c == 70 && prev_yaw == M_PI/2) {
      //   int ewq;
      // }

      double cost_x = c * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.x;
      double cost_y = r * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.y;

      double glob_x = curr_x + std::cos((est_yaw + curr_yaw) / 2) * cost_x - std::sin((est_yaw + curr_yaw) / 2) * cost_y;
      double glob_y = curr_y + std::sin((est_yaw + curr_yaw) / 2) * cost_x + std::cos((est_yaw + curr_yaw) / 2) * cost_y;
      
      int glob_c = std::floor((glob_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int glob_r = std::floor((glob_y - global_map_.info.origin.position.y) / global_map_.info.resolution);
      int index = glob_r * global_map_.info.width + glob_c;
      // if (index == 11307) {
      //   RCLCPP_INFO(this->get_logger(), "HERE. r: %d, c: %d", r, c);
      // }
      double cost = latest_costmap_.data[r * latest_costmap_.info.width + c];

      // // todo: keep track of how many times seen
      // todo: keep track of cells visited
      // if (c == 67 && r == 50) {
      //   RCLCPP_INFO(this->get_logger(), "Info: glob_r - %d, glob_c - %d, index - %d, cost - %.3f", glob_r, glob_c, index, cost);
      // }
      // if (45 <= r && r <= 55 && 65 <= c && c <= 75) {
      //   if (r == 50 && c == 70) {
      //     RCLCPP_INFO(this->get_logger(), "Info: glob_r - %d, glob_c - %d, index - %d, yaw - %.3f", glob_r, glob_c, index, prev_yaw);
          
      //   }
      //   global_map_.data[index] = 75;
      // }
      if (0 <= glob_c && glob_c < global_map_.info.width && 0 <= glob_r && glob_r < global_map_.info.height) {
        double new_cost;

        if (cost > 0) {
          times_seen_[index] += 1;
        }

        if (cost >= global_map_.data[index]) {
          new_cost = cost;
        } else if (times_seen_[index] > 0) {
          double adj_recency_bias =  recency_bias / std::pow(times_seen_[index], times_seen_effectiveness);
          new_cost = adj_recency_bias * cost + (1 - adj_recency_bias) * global_map_.data[index];
          // if (index == 11307) {
          //   RCLCPP_INFO(this->get_logger(), "%.10f * %.3f + %.10f * %.3f = %.3f", adj_recency_bias, cost, 1 - adj_recency_bias, global_map_.data[index], new_cost);
          // }
          // if (index == 11307) {
          //   RCLCPP_INFO(this->get_logger(), "cost_test: %.3f", new_cost);
          // }
        } else {
          new_cost = 0;
        }
        // if (index == 11307) {
        //   RCLCPP_INFO(this->get_logger(), "cost: %.3f", new_cost);
        // }
        global_map_.data[index] = new_cost;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
