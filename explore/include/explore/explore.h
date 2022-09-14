/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "tf/tf.h"
#include <natnet_pkg/PoseArrayID.h>

#include <regex>
#include <iterator>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <random>
#include <pwd.h>
#include <time.h>

namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore
{
public:
  Explore();
  ~Explore();

  void start();
  void stop();

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief  Publish a frontiers as markers
   */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& frontier_goal);

  bool goalOnBlacklist(const geometry_msgs::Point& goal);

  void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

  void optitrackMocapCB(const natnet_pkg::PoseArrayID::ConstPtr& msg);

  bool IsMatch(std::string& val);

  void writeToFile(std::vector<frontier_exploration::Frontier>& default_frontiers, 
                            std::vector<frontier_exploration::Frontier>&wsr_frontiers,
                            std::string& fn);

  void explorationStatusCB(const std_msgs::Bool::ConstPtr& msg);
  void uwbCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
  std::vector<std::vector<double>> generate_range();
  std::pair<std::vector<std::string>, std::vector<std::vector<double>>> generate_aoa();
  void positionCallbackT265(const nav_msgs::Odometry::ConstPtr& t265_msg);
  double quaternionToYaw(const tf::Quaternion& q);
  bool validateQuaternion(const tf::Quaternion& quat);


  double wrap0to360(double val);
  void particle_filter();

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_, velocityPub_, get_csi_Pub_;
  ros::Subscriber modelStateSub_, exploration_, optitrackSub_,neighbor_distance_,t265_position_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;
  std_msgs::Bool get_csi_data_;
  ros::Time last_progress_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_=0, noise_mean_=0, noise_std_=0,
         potential_scale_, orientation_scale_=0, gain_scale_=0, sensor_range_=0, 
         decay_rate_=0,robot_orientation_=0,robot_position_x_=0,robot_position_y_=0,
         prev_distance_=0,antenna_angular_offset_=0,robot_orientation_before_=0;
  ros::Duration progress_timeout_;
  bool visualize_;
  std::string robot_name_, neighbor_name_, config_file_,displacement_type_, reverse_csi_, displacement_file_,output_,
              robot_csi_,robot_displacement_;
  std::vector<int>neighbor_id_;
  std::vector<geometry_msgs::Point> neighbor_pose_vec_;
  bool FLAG_getting_next_frontier_ = true, FLAG_WSR_ = false, exploration_completed_=false, exploration_done_ = false,FLAG_GET_POS=true
      ,Flag_get_range_ = false;
  std::vector<std::vector<float>> wsr_frontiers_stats_, default_frontier_stats_;
  int robot_id_ = -1, iterations__=0;
  geometry_msgs::Twist velocity_cmd_;
  std::vector<std::vector<double>> range_vector_;
  int neighbor_count_=1;
  void writePosToFile(std::vector<std::pair<double,double>>& pos_file,
                      std::string fn);

  //Particle filter parameters
  std::vector<std::vector<std::pair<double,double>>> neighbor_robot_init_pos_;
  std::vector<std::vector<std::pair<double,double>>> neighbor_robot_est_pos_;
  int particle_threshold_ = 540, init_angle_samples_ = 180;
  std::vector<int> aoa_init_;
  std::vector<std::vector<std::pair<double,double>>> neighbor_best_position_esimtate_;

};
}

#endif
