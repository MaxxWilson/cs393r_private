//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "ros/ros.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct CommandStamped{
  double velocity = 0.0;
  double curvature = 0.0;
  uint64_t stamp = 0.0;

  CommandStamped(){};

  CommandStamped(double velocity, float curvature, uint64_t stamp){
    this->velocity = velocity;
    this->curvature = curvature;
    this->stamp = stamp;
  }

  bool operator <(const uint64_t time_compare)
  {
    return this->stamp < time_compare;
  }
};

struct TimeShiftedTF{
  Eigen::Vector2f position = Eigen::Vector2f(0, 0);
  double theta = 0.0;
  double speed = 0.0;
  uint64_t stamp = 0;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel,
                      uint64_t time);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         uint64_t time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  // Use time optimal strategy to control the car
  void TimeOptimalControl(const PathOption& path);

  std::vector<CommandStamped> vel_commands_;

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;

  // Last odometry timestamp
  uint64_t odom_stamp_;
  uint64_t last_odom_stamp_ = 0;
  //Updates if odometry has new data
  bool has_new_odom_;

  TimeShiftedTF odom_state_tf;

  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  std::vector<Eigen::Vector2f> transformed_point_cloud_;

  //Point cloud timestamp
  uint64_t point_cloud_stamp_;
  //True if point cloud is updated
  bool has_new_points_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  bool first_cycle = true;

  void TransformPointCloud(TimeShiftedTF transform);
};

}  // namespace navigation

#endif  // NAVIGATION_H
