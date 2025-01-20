#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void ControlCore::initControlParams(double ld, double gt, double ls) {
  lookahead_distance_ = ld;
  goal_tolerance_ = gt;
  linear_speed_ = ls;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
  int s = current_path_.poses.size();

  geometry_msgs::msg::Point rbt_pos = robot_odom_.pose.pose.position;

  // geometry_msgs::msg::PoseStamped desired;
  // desired.header.frame_id = "sim_world";

  // // TODO: remove
  // desired.pose.position.x = -13;
  // desired.pose.position.y = -2;
  // return desired;
  // // TODO: testing

  if (s == 0) {
    return std::nullopt;
  } else if (s == 1) {
    return current_path_.poses[0];
  }

  // if (computeDistance(rbt_pos, current_path_.poses[0].pose.position) <= computeDistance(rbt_pos, current_path_.poses[s - 1].pose.position)) {
  //   desired = current_path_.poses[0];
  // }

  double curr_d, next_d;
  double curr_err, next_err;

  for (int i = 0; i < s - 1; i++) {
    // curr_d = computeDistance(rbt_pos, current_path_.poses[i-1].pose.position);
    curr_d = computeDistance(rbt_pos, current_path_.poses[i].pose.position);
    next_d = computeDistance(rbt_pos, current_path_.poses[i+1].pose.position);

    curr_err = std::abs(curr_d - lookahead_distance_);
    next_err = std::abs(next_d - lookahead_distance_);

    if (curr_d <= lookahead_distance_ && next_d >= lookahead_distance_) {
      if (curr_err <= next_err) {
        return current_path_.poses[i];
      } else {
        return current_path_.poses[i+1];
      }
    }
    // curr_err = curr_d - lookahead_distance_;

    // if (i == 1) {
    //   next_d = computeDistance(rbt_pos, current_path_.poses[i].pose.position);
    //   next_err = next_d - lookahead_distance_;
    //   prev_err = curr_err + 1;
    // } else if (i == s) {
    //   prev_d = computeDistance(rbt_pos, current_path_.poses[i-2].pose.position);
    //   prev_err = prev_d - lookahead_distance_;
    //   next_err = curr_err + 1;
    // } else {
    //   prev_d = computeDistance(rbt_pos, current_path_.poses[i-2].pose.position);
    //   next_d = computeDistance(rbt_pos, current_path_.poses[i].pose.position);
    //   prev_err = prev_d - lookahead_distance_;
    //   next_err = next_d - lookahead_distance_;
    // }

    // if (curr_err <= prev_err && curr_err <= next_err) {
    //   desired = current_path_.poses[i];
    // }
  }

  return current_path_.poses[s - 1];
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;

  geometry_msgs::msg::Point rbt_pos = robot_odom_.pose.pose.position;
  geometry_msgs::msg::Point targ_pos = target.pose.position;
// // TODO: add cost in planning
// TODO: also check if memory map lines up with odom
  if (computeDistance(rbt_pos, current_path_.poses[current_path_.poses.size() - 1].pose.position) <= goal_tolerance_) {
    return cmd_vel;
  }

  cmd_vel.linear.x = linear_speed_;
  
  // let theta be yaw of robot, alpha be angle of vector RT, x be angle between RT and RC, y be angle between CR and CT
  double robot_ang = extractYaw(robot_odom_.pose.pose.orientation);
  double ang_to_targ = std::atan2(targ_pos.y - rbt_pos.y, targ_pos.x - rbt_pos.x);
  // RCLCPP_INFO(logger_, "ang_to_targ: %.3f, robot_ang: %.3f", 180 * ang_to_targ / M_PI, 180 * robot_ang / M_PI);
  double d_ang = ang_to_targ - robot_ang;
  if (std::abs(d_ang) > M_PI) {
    if (d_ang < 0) {
      d_ang += 2*M_PI;
    } else {
      d_ang -= 2*M_PI;
    }
  }
  // RCLCPP_INFO(logger_, "d_ang: %.3f - %.3f = %.3f", 180 * ang_to_targ / M_PI, 180 * robot_ang / M_PI, 180 * d_ang / M_PI);
  if (std::abs(d_ang) >= M_PI/2) {
    cmd_vel.angular.z = d_ang * linear_speed_ * 20;
    return cmd_vel;
  }

  double x, y;

  // x = pi/2 - (alpha - theta)
  //   = pi/2 + theta - alpha
  // y = pi - 2x
  x = M_PI/2 - d_ang;//robot_ang - ang_to_targ;
  y = M_PI - 2 * x;

  // Prevent division by 0
  if (std::cos(y) == 1) {
    // Means robot is already pointed straight at target
    return cmd_vel;
  }
  
  // let d be distance between R and T, r be radius of circular path to take
  double d = computeDistance(rbt_pos, targ_pos);
  double r;
  
  // r = d / sqrt(2(1 - cosy))
  r = d / std::sqrt(2 * (1 - std::cos(y)));

  // let d_path be length of circular path, t be time to target, ang_vel be required angular velocity
  double ang_vel;

  // d_path = |yr| = |y|r
  // t = d_path / lin_vel
  // ang_vel = y / t
  //         = y / (d_path / lin_vel)
  //         = lin_vel * (y / d_path)
  //         = lin_vel * (y / |y|r)
  //         = + or - lin_vel / r, depending on sign of y

  if (y < 0) {
    ang_vel = -linear_speed_ / r;
  }
  else {
    ang_vel = linear_speed_ / r;
  }

  // RCLCPP_INFO(logger_, "ang_to_targ: %.3f, robot_ang: %.3f", 180 * ang_to_targ / M_PI, 180 * robot_ang / M_PI);
  // // why y/t?
  // // let beta be angle of linear vel at T
  // // let d_ang = beta - theta
  // // let P be point on RC horizontal from T
  // // let z be angle PTC
  // // angle TPC = pi/2 + theta
  // // z = pi - (pi/2 + theta) - y
  // //   = pi/2 - theta - y
  // // beta = pi - (pi/2 + z)
  // //      = pi/2 - z
  // //      = pi/2 - (pi/2 - theta - y)
  // //      = theta + y
  // // d_ang = theta + y - theta
  // //       = y

  cmd_vel.angular.z = ang_vel;

  return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;
  
  return std::atan2(2*(x*y + z*w), 1 - 2*(y*y + z*z));
}

}