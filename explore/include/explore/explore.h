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

#include <explore/utils.h>

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
  ros::NodeHandle public_nh_;

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

  void modelStateCallbackFilter(const gazebo_msgs::ModelStates::ConstPtr& msg);

  void ViconCombinedStateCallbackTruePositionBaseline(const geometry_msgs::PoseArray::ConstPtr& input_msg); //Use for hardware experiments with Vicon

  void modelStateCallbackTruePositionForBaseline(const gazebo_msgs::ModelStates::ConstPtr& input_msg);

  void SensingCallbackFilter(const wiserx_explore_lite::LocalMeasurement::ConstPtr& input_msg);

  bool IsMatch(std::string& val);

  bool IsMatchDim(std::string& val);

  void writeToFile(std::vector<frontier_exploration::Frontier>& wsr_frontiers,
                            std::string& fn,
                            std::chrono::seconds& elapsed_time__);

  void explorationStatusCB(const std_msgs::Bool::ConstPtr& msg);
  
  void uwbCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
  
  void setFailedRobotStatus(const std_msgs::String::ConstPtr& msg);

  std::vector<std::vector<double>> generate_range();
  std::pair<std::vector<std::string>, std::vector<std::vector<double>>> generate_aoa();
  
  void positionCallbackT265(const nav_msgs::Odometry::ConstPtr& t265_msg);


  bool GetPlanPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal, float tolerance,
                  nav_msgs::Path& plan); 

  double calculatePathLength(const nav_msgs::Path& path); 
  
  void particle_filter();

  // void CollectOwnPoseCB(const std_msgs::Bool::ConstPtr& msg);
  void CollectOwnPoseCB(); 

  void TruePoseCB(const geometry_msgs::PoseArray::ConstPtr& msg);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_, velocityPub_, get_csi_Pub_, quadmapPub_,exploration_eval_stop_;
  ros::Subscriber modelStateSub_, exploration_, optitrackSub_,neighbor_distance_,t265_position_,setFailedRobotTau_,saveRobotPose_, true_positions_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;
  ros::Timer ownSLAMposearray_;
  unsigned int mx__, my__;
  unsigned int fmx__, fmy__;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;
  std_msgs::Bool get_csi_data_;
  ros::Time last_progress_;
  ros::Time progress_start_time_;
  size_t last_markers_count_;

  // parameters
  std::string fn__ = "";
  double planner_frequency_=0, 
          noise_mean_=0, 
          noise_std_=0,
          potential_scale_, 
          orientation_scale_=0, 
          gain_scale_=0, 
          sensor_range_=0,
          robot_orientation_=0,
          robot_position_x_=0,
          robot_position_y_=0,
          prev_distance_=0,
          antenna_angular_offset_=0,
          robot_orientation_before_=0,
          utility_alpha_parameter_=1, 
          utility_beta_parameter_=1;
  float noise_x_ = 0 ;
  float noise_y_ = 0 ;
  ros::Duration progress_timeout_;
  bool visualize_;
  std::string robot_name_, 
              neighbor_name_, 
              config_file_,
              displacement_type_, 
              reverse_csi_, 
              displacement_file_,
              output_,
              robot_csi_,
              robot_displacement_, 
              __dim_object_name, 
              homedir_,
              servo_output_file_reader_command_;
  std::vector<int>neighbor_id_;
  std::vector<geometry_msgs::Point> current_neighbor_pose_vec_;
  std::vector<geometry_msgs::Point> current_neighbor_NODE_vec_;
  bool FLAG_WSR_ = false, exploration_completed_=false, exploration_done_ = false,FLAG_GET_POS=true,Flag_get_range_ = false;
  bool FLAG_SIM_ = true;
  bool FLAG_use_real_sensors__ = true;
  bool FLAG_DEBUG_OWN_TRUE_POSE__ = false;
  std::vector<std::vector<float>> wsr_frontiers_stats_, default_frontier_stats_;
  int robot_id_ = -1, iterations__=0;
  geometry_msgs::Twist velocity_cmd_;
  std::vector<std::vector<double>> range_vector_;
  int neighbor_count_=1;
  void writePosToFile(std::vector<std::pair<double,double>>& pos_file,
                      std::string fn);
  float __Flag_set_home = false;
  geometry_msgs::Point  __left_bottom, __right_bottom, __left_top, __right_top;
  std::vector<geometry_msgs::Point> __envBoundary;
  geometry_msgs::Point __home_position;
  ros::ServiceClient move_bas_path_client__; 
  nav_msgs::Path frontier_centroid_path__;
  geometry_msgs::PoseStamped start__; 
  geometry_msgs::PoseStamped goal__;
  float tolerance__ = 0.5; //in meters
  int baseline_1_frontier_selection_threshold__ = 60;
  int pose_size_limit__ = 300;
  double robot_speed_ = 0.15;
  float bearing_angle_radians__ = 0;
  std::vector<float> bearing_angle_radians_vec__;
  float own_orientation_deg__ = 0;
  float range_to_use__ = 0.0;
// float __bearing_to_use =  0.0;
  std::deque<std::pair<double, geometry_msgs::Pose>> own_pose_deque_;
  std::deque<std::pair<double, geometry_msgs::Pose>> own_true_pose_deque_;
  
  std::mutex own_pose_mutex;
  wiserx_explore_lite::RangeBearing neighbor_robot_rb__;

  //Quadmap parameters
  quadmap::QuadMap base_quadmap_;
  double Quadmap_width_;
  double Quadmap_height;
  std::unordered_map<std::string, quadmap::Robot> robot_information__; //Stores information of a neighoring robot j.
  int timestep__ = 0 ;
  float cell_count__ = 0;
  float filled_cell_count__=0;
  int my_tau__ = 1;
  bool __FLAG_can_stop_now__ = false;
  double fill_percentage_threshold__ = 75;
  double map_resolution__ = 0 ;
  bool __FLAG_publish_once = false;
  int diff_between_termination_thresholds__ = 5;
  unsigned int x_env_map_max_limit__=1000; 
  unsigned int y_env_map_max_limit__=1000;
  unsigned int x_env_map_min_limit__=0;
  unsigned int y_env_map_min_limit__=0;

  //Filter parameters
  std::vector<double> measurement_output__;
  std::unordered_map<std::string, wsr_state_estimation::ExtendedKalmanFilter> ekf_robot_track__;
  std::unordered_map<std::string, wsr_state_estimation::ParticleFilter> pf_robot_track__;
  std::vector<std::vector<std::pair<double,double>>> neighbor_robot_init_pos_;
  std::vector<std::vector<std::pair<double,double>>> neighbor_robot_est_pos_;
  int particle_threshold_ = 540;
  int init_angle_samples_ = 180;
  int frame__ = 0;
  std::vector<int> aoa_init_;
  std::vector<std::vector<std::pair<double,double>>> neighbor_best_position_esimtate_;
  bool same_goal__ = false, reached_goal__=true,__checked_for_new_frontiers=false;
  double robot_position_x_before_=0.0, robot_position_y_before_=0.0;
  double measurement_interval__ = 0;
  std::vector<frontier_exploration::Frontier> frontiers__, frontier_temp__;
  std::vector<frontier_exploration::Frontier>::iterator frontier_itr;
  std::vector<double> cov_array_prev{0, 0};
  std::vector<geometry_msgs::Point> current_rel_positions__;
  std::vector<double> current_angle_vec__;
  int other_robot_id__=-1;
  float min_diff__ = 0;
  float diff__ = 0;
  float previous_angle__ = 0;
  float current_angle__ = 0;
  bool __FLAG_first_measurement = true;
  geometry_msgs::Pose prev_neighboring_position;
};
}


#endif
