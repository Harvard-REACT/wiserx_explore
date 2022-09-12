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
 * 
 *  Modified by:
 *  Ninad Jadhav
 *  Harvard REACT Lab
 * 
 *********************************************************************/

#include <explore/explore.h>
#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <thread>
#include <chrono>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

std::string fn = "/home/jadhav/catkin_ws/src/wsr_exploration/data/mexplore_data/";
std::default_random_engine generator;
bool FLAG_noise = false;
struct passwd *pw = getpwuid(getuid());
std::string homedir = pw->pw_dir;


namespace explore
{

  bool  Explore::IsMatch(std::string& val)
  {
    return (val.find(neighbor_name_) != std::string::npos);
  }

  void Explore::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    int itr = 0, n_count = 0;
    // std::cout << FLAG_getting_next_frontier_ << std::endl;
    if(FLAG_getting_next_frontier_) //Control the update rate, but does not work
    {
      std::vector<std::string> name = msg->name;
      neighbor_id_.clear();
      for(std::string& val : name)
      {
        if (IsMatch(val) && val!=robot_name_) 
        {
          // std::cout << val << std::endl;
          neighbor_id_.push_back(n_count); 
        }
        // else
        // {
          // std::cout << "not found" << std::endl;
        // }
        n_count+=1;
      }

      std::vector<geometry_msgs::Pose> pose_vec = msg->pose;
      neighbor_pose_vec_.clear();
      
      //Store the positions of the neighboring robot
      for (itr=0; itr<neighbor_id_.size(); itr++)
      {
        // std::cout << robot_name_ << std::endl;
        // std::cout << neighbor_id_[itr] << std::endl;
        // std::cout << pose_vec[neighbor_id_[itr]].position << std::endl;
        
        if(FLAG_noise)
        {
          static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
          pose_vec[neighbor_id_[itr]].position.x = pose_vec[neighbor_id_[itr]].position.x + gaussian_noise_(generator);
          pose_vec[neighbor_id_[itr]].position.y = pose_vec[neighbor_id_[itr]].position.y + gaussian_noise_(generator);
        }
        neighbor_pose_vec_.push_back(pose_vec[neighbor_id_[itr]].position );
      }
      
    }
  }


  void Explore::optitrackMocapCB(const natnet_pkg::PoseArrayID::ConstPtr& msg)
  {
    if(FLAG_GET_POS)
    {  
    for (int i=0; i<msg->poses.size(); i++) 
      {
        //std::cout << "Streaming ID: " << msg->poses[i].ID << std::endl;
        //std::cout << "Position: " << msg->poses[i].position << std::endl;
        //std::cout << "Orientation: " << msg->poses[i].orientation << std::endl;
        //std::cout << "\n" << std::endl;
        //neighbor_pose_vec_.clear();
        if(msg->poses[i].ID != robot_id_)
        {
            //ROS_INFO("Got neighbor");
	    geometry_msgs::Point temp = msg->poses[i].position;
            //std::cout << "Position: " << temp << std::endl;
            if(FLAG_noise)
            {
              static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
              temp.x = temp.x + gaussian_noise_(generator);
              temp.y = temp.y + gaussian_noise_(generator);
            }
            neighbor_pose_vec_.push_back(temp);
	    for (int itr=0; itr<neighbor_pose_vec_.size(); itr++)
    	    {   
      		ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, neighbor_pose_vec_[itr].x, neighbor_pose_vec_[itr].y);
    	    }
        }
      }
     FLAG_GET_POS = false;
    }
  }

  void Explore::uwbCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(Flag_get_range_)
    {  
      range_vector_[0].push_back(msg->data[0]);
      range_vector_[1].push_back(msg->data[0]);
    }
  }

  void Explore::explorationStatusCB(const std_msgs::Bool::ConstPtr& msg)
  {
    exploration_done_ = msg->data;
  }

  Explore::Explore()
    : private_nh_("~")
    , tf_listener_(ros::Duration(10.0))
    , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
    , move_base_client_("move_base")
    , prev_distance_(0)
    , last_markers_count_(0)
  {
    double timeout;
    double min_frontier_size;
    
    //Explore parameters
    private_nh_.param("planner_frequency", planner_frequency_, 1.0);
    private_nh_.param("progress_timeout", timeout, 30.0);
    progress_timeout_ = ros::Duration(timeout);
    private_nh_.param("visualize", visualize_, false);
    private_nh_.param("potential_scale", potential_scale_, 1e-3);
    private_nh_.param("orientation_scale", orientation_scale_, 0.0);
    private_nh_.param("gain_scale", gain_scale_, 1.0);
    private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

    //Robot details
    private_nh_.param("robot_name", robot_name_, std::string("tb3_0"));
    private_nh_.param("robot_id", robot_id_, 0);
    private_nh_.param("neighbor_name", neighbor_name_, std::string("tb3_"));

    //WSR related parameters:
    private_nh_.param("WSR_config_file", config_file_); 
    private_nh_.param("WSR_robot_displacement", displacement_type_, std::string("odom")); 
    private_nh_.param("use_WSR", FLAG_WSR_, true);
    private_nh_.param("noise_WSR", FLAG_noise, false);
    private_nh_.param("WSR_noise_mean", noise_mean_, 0.0);
    private_nh_.param("WSR_noise_std", noise_std_, 1.0); 
    private_nh_.param("sensor_range", sensor_range_, 1.0); 
    private_nh_.param("decay_rate", decay_rate_, 0.25);

    
    //Subscribe to the gazebo state to get the position of the other robot
    modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
    optitrackSub_ = private_nh_.subscribe<natnet_pkg::PoseArrayID> ("/optitrack_pose", 10, &Explore::optitrackMocapCB, this);
    exploration_ = private_nh_.subscribe<std_msgs::Bool> ("/true_exploration_status", 10, &Explore::explorationStatusCB, this);
    velocityPub_ = private_nh_.advertise<geometry_msgs::Twist> ("/"+robot_name_+"/cmd_vel", 10);

    //initialize UWB data structure based on number of neighbors
    std::vector<double> temp0, temp1;
    range_vector_.push_back(temp0);
    range_vector_.push_back(temp1);

    //Initialize exploration
    ROS_INFO("Sensor range = %f", sensor_range_);
    ROS_INFO("min_frontier_size = %f", min_frontier_size);
    search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                  potential_scale_, gain_scale_,
                                                  min_frontier_size, sensor_range_,decay_rate_);

    if (visualize_) {
      marker_array_publisher_ =
          private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
    }

    ROS_INFO("Waiting to connect to move_base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");

    exploring_timer_ =
        relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                [this](const ros::TimerEvent&) { makePlan();});
  }

  Explore::~Explore()
  {
    stop();
  }

/**
 * @brief Visualize the frontiers in rviz 
 * 
 * @param frontiers 
 */
  void Explore::visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers)
  {
    std_msgs::ColorRGBA blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    std_msgs::ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    std_msgs::ColorRGBA green;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

    ROS_DEBUG("visualising %lu frontiers", frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    // weighted frontiers are always sorted
    // double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
    double min_cost = frontiers.empty() ? 0. : frontiers.back().cost; //If its 0 if the frontier cal freq is less
    double max_cost = frontiers.empty() ? 0. : frontiers.front().cost;
    // double min_cost = frontiers.empty() ? 0. : 1 - frontiers.front().cost; //For normalized cost to utilize the visualization.

    m.action = visualization_msgs::Marker::ADD;
    size_t id = 0;
    for (auto& frontier : frontiers) 
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      if (goalOnBlacklist(frontier.centroid)) {
        m.color = red;
      } else {
        m.color = green;
      }
      markers.push_back(m);
      ++id;
      break;
      // m.type = visualization_msgs::Marker::SPHERE;
      // m.id = int(id);
      // m.pose.position = frontier.centroid;
      // // scale frontier according to its cost (costier frontiers will be smaller)
      // // double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      // double scale = std::min((frontier.cost - min_cost) / (max_cost - min_cost), 0.2); //For new info gain formulation
      // // double scale = std::min(std::abs(min_cost * 0.4 / (frontier.cost+0.001)), 0.5); //For normalized cost to utilize the visualization.
      // m.scale.x = scale;
      // m.scale.y = scale;
      // m.scale.z = scale;
      // m.points = {};
      // m.color = green;
      // markers.push_back(m);
      // ++id;
    }
    size_t current_markers_count = markers.size();

    // delete previous markers, which are now unused
    m.action = visualization_msgs::Marker::DELETE;
    for (; id < last_markers_count_; ++id) {
      m.id = int(id);
      markers.push_back(m);
    }

    last_markers_count_ = current_markers_count;
    marker_array_publisher_.publish(markers_msg);
  }

/**
 * @brief Generate the exploration plan for the current timestep
 * 
 */
  void Explore::makePlan()
  {
    // find frontiers
    auto pose = costmap_client_.getRobotPose();

    // get frontiers sorted according to cost
    std::vector<frontier_exploration::Frontier> frontiers, frontier_temp;
    ROS_DEBUG("found %lu frontiers", frontiers.size());

    if(FLAG_WSR_)
    {
      
      //===== Start: Getting WiFi CSI data and UWB Range measurements as robot rotates in place =====
      
      //Save range data
      Flag_get_range_ = true;

      //Start CSI
      std::string csi_start_local_cmd = homedir+"/catkin_ws/src/adaptive_navigation_using_aoa/pipeline_scripts/start_csi.sh rx &";  
      system(csi_start_local_cmd.c_str());

      //Start motion
      int duration_val = 8;//seconds
      auto starttime = std::chrono::high_resolution_clock::now();
      auto endtime = std::chrono::high_resolution_clock::now();
      float exp_duration;

      while(true)
      {
        velocity_cmd_.angular.z = 2.2;
        velocityPub_.publish(velocity_cmd_);
        exp_duration = std::chrono::duration<float, std::milli>(endtime - starttime).count() * 0.001;
        if(exp_duration < duration_val) break; 
      }

      //Stop motion
      velocity_cmd_.linear.x = 0.0;
      velocity_cmd_.angular.z = 0.0;
      velocityPub_.publish(velocity_cmd_);

      //Stop CSI
      std::string csi_stop_cmd = homedir+"/catkin_ws/src/adaptive_navigation_using_aoa/pipeline_scripts/stop_csi.sh";
      system(csi_stop_cmd.c_str()); //TODO: Check correct command from robot
      
      //Stop range
      Flag_get_range_=false;

      //===== Finished: Getting CSI data and Range measurements as robot rotates in place =====

      //Compute AOA and then initial position estimates
      std::pair<std::vector<std::string>, std::vector<std::vector<double>>> WSR_val = generate_aoa();
      std::vector<std::vector<double>> aoa_val = WSR_val.second;
      std::vector<std::vector<double>> range_val = generate_range();    
      range_vector_[0].clear();
      range_vector_[1].clear();

      std::vector<std::vector<std::pair<double,double>>> initial_position_estimates;
      int num_neighbors = aoa_val.size();
      for(int k=0;k<num_neighbors;k++)
      {
        std::vector<std::pair<double,double>> estimates;
        double angle, dist, position_x, position_y;
        for(int i=0;i<range_val[k].size();i++)
        {
          for(int j=0;j<aoa_val[k].size();j++)
          {
            dist = range_val[k][i];
            angle = wrap0to360(wrap0to360(aoa_val[k][j]) + wrap0to360(robot_orientation)); //position with respect to 0 degrees 
            position_x = (cos(angle*M_PI/180) * dist) + robot_position_x;
            position_y = (cos(angle*M_PI/180) * dist) + robot_position_y;
            estimates.push_back(std::make_pair(position_x,position_y));
          }
        }
        initial_position_estimates.push_back(estimates);
      }

      /**
       * @brief Update neighbor_pose_vec_ with the latest position estimates of the neighboring robots
       * 
       */
      frontier_temp = search_.searchFromNew(pose.position, neighbor_pose_vec_);
      ROS_DEBUG("Original cost");
      for (size_t i = 0; i < frontier_temp.size(); ++i) 
      {
        ROS_DEBUG("frontier %zd cost: %f", i, frontier_temp[i].cost);
	      ROS_DEBUG("frontier neighbors %d", frontier_temp[i].neighbors);
        ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontier_temp[i].centroid.x, frontier_temp[i].centroid.y);
      }


      ROS_INFO("Neighbors count = %d", neighbor_pose_vec_.size());
      for (int itr=0; itr<neighbor_pose_vec_.size(); itr++)
      {
        ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, neighbor_pose_vec_[itr].x, neighbor_pose_vec_[itr].y);
      }
      
      frontiers = search_.searchFromWithNeighorInfo(pose.position, neighbor_pose_vec_);
      ROS_DEBUG("New frontier frontier cost");
      for (size_t i = 0; i < frontiers.size(); ++i) 
      {
        ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
        ROS_DEBUG("frontier neighbors %d", frontier_temp[i].neighbors);
        ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontiers[i].centroid.x, frontiers[i].centroid.y);
      }
      FLAG_GET_POS = true;
      neighbor_pose_vec_.clear();
      writeToFile(frontier_temp, frontiers, fn);
    }
    else
    {
      // frontiers = search_.searchFrom(pose.position); //original code.
      
      // using neighrbor info just to collect stats and not for utility calculation
      frontiers = search_.searchFromNew(pose.position, neighbor_pose_vec_); 
      
      ROS_DEBUG("Original cost");
      for (size_t i = 0; i < frontiers.size(); ++i) 
      {
        ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
        ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontiers[i].centroid.x, frontiers[i].centroid.y);
      }

      writeToFile(frontiers,frontier_temp,fn);
    }
    
    
    if (frontiers.empty() || exploration_done_) 
    {
      stop();
      writeToFile(frontier_temp,frontiers, fn);
      return;
    }

    // publish frontiers as visualization markers
    if (visualize_) {
      visualizeFrontiers(frontiers);
    }

    // find non blacklisted frontier
    auto frontier =
        std::find_if_not(frontiers.begin(), frontiers.end(),
                        [this](const frontier_exploration::Frontier& f) {
                          return goalOnBlacklist(f.centroid);
                        });
    
    
    if (frontier == frontiers.end()) 
    {
      stop();
      return;
    }
    
    // if(frontiers.size() == 1)
    // {
    //   frontier->centroid = frontier->furthest;
    //   frontier->centroid_distance = frontier->min_distance; //just a heuristic
    // }

    geometry_msgs::Point target_position = frontier->centroid;

    // time out if we are not making any progress
    bool same_goal = prev_goal_ == target_position;
    prev_goal_ = target_position;
    if (!same_goal || prev_distance_ > frontier->min_distance) {
      // we have different goal or we made some progress
      last_progress_ = ros::Time::now();
      prev_distance_ = frontier->min_distance;
    }
    // black list if we've made no progress for a long time
    if (ros::Time::now() - last_progress_ > progress_timeout_) {
      frontier_blacklist_.push_back(target_position);
      ROS_DEBUG("Adding current goal to black list");
      makePlan();
      return;
    }

    // we don't need to do anything if we still pursuing the same goal
    if (same_goal) {
      return;
    }

    // send goal to move_base if we have something new to pursue
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = target_position;
    goal.target_pose.pose.orientation.w = 1.;
    goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();
    move_base_client_.sendGoal(
        goal, [this, target_position](
                  const actionlib::SimpleClientGoalState& status,
                  const move_base_msgs::MoveBaseResultConstPtr& result) {
          reachedGoal(status, result, target_position);
        });
  }

  /**
   * @brief Add a non-reachable frontier to backlist
   * 
   * @param goal 
   * @return true 
   * @return false 
   */

  bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
  {
    constexpr static size_t tolerace = 5;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

    // check if a goal is on the blacklist for goals that we're pursuing
    for (auto& frontier_goal : frontier_blacklist_) {
      double x_diff = fabs(goal.x - frontier_goal.x);
      double y_diff = fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerace * costmap2d->getResolution() &&
          y_diff < tolerace * costmap2d->getResolution())
        return true;
    }
    return false;
  }

/**
 * @brief Check if robot has reached a frontier
 * 
 * @param status 
 * @param frontier_goal 
 */
  void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                            const move_base_msgs::MoveBaseResultConstPtr&,
                            const geometry_msgs::Point& frontier_goal)
  {
    ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
    if (status == actionlib::SimpleClientGoalState::ABORTED) {
      frontier_blacklist_.push_back(frontier_goal);
      ROS_DEBUG("Adding current goal to black list");
    }

    // find new goal immediatelly regardless of planning frequency.
    // execute via timer to prevent dead lock in move_base_client (this is
    // callback for sendGoal, which is called in makePlan). the timer must live
    // until callback is executed.
    oneshot_ = relative_nh_.createTimer(
        ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
        true);
  }

  void Explore::start()
  {
    exploring_timer_.start();
  }

  void Explore::stop()
  {
    move_base_client_.cancelAllGoals();
    exploring_timer_.stop();
    exploration_completed_ = true;
    ROS_INFO("Exploration stopped.");
  }

/**
 * @brief Write the experiment data in a file 
 * 
 * @param default_frontiers 
 * @param wsr_frontiers 
 * @param fn 
 */
void Explore::writeToFile(std::vector<frontier_exploration::Frontier>& default_frontiers, 
                            std::vector<frontier_exploration::Frontier>&wsr_frontiers,
                            std::string& fn)
  {

    for(int i=0; i<default_frontiers.size(); i++)
    { 
      std::vector<float> temp {float(default_frontiers[i].pos_id), default_frontiers[i].information_gain, 
                              default_frontiers[i].effort, default_frontiers[i].neighbor_distance, 
                              float(default_frontiers[i].neighbors), float(i+1)}; 

      default_frontier_stats_.push_back(temp);
    }

    if(FLAG_WSR_)
    { 
      for(int i=0; i<wsr_frontiers.size(); i++)
      {
        std::vector<float> temp{ float(wsr_frontiers[i].pos_id), wsr_frontiers[i].information_gain, 
                            wsr_frontiers[i].effort, wsr_frontiers[i].neighbor_distance, 
                            float(wsr_frontiers[i].neighbors), float(i+1)};
        wsr_frontiers_stats_.push_back(temp);
      }
    }

    if(exploration_completed_)
    {
      ROS_INFO("Saving exploration stats to file.");
      std::cout.precision(10);
      const auto p1 = std::chrono::system_clock::now();
      std::string ts = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());
      std::string fn1 = fn+"def_frontiers_stats_"+robot_name_+"_"+ts+".csv";
      std::string fn2 = fn+"wsr_frontiers_stats_"+robot_name_+"_"+ts+".csv";
      if(FLAG_WSR_)
      {
        fn1 = fn+"def_frontiers_stats_wsr_"+robot_name_+"_"+ts+".csv";
        fn2 = fn+"wsr_frontiers_stats_wsr_"+robot_name_+"_"+ts+".csv";
      }
      std::ofstream myfile_def (fn1);
      std::ofstream myfile_wsr (fn2);
      std::vector<double> temp;
      std::vector<std::string> details {"pos_id", "info_gain", "effort", "j_dist", "j_count", "index"}; //index 1 means the top most frontier at each iteration which will then be selected

      if (myfile_def.is_open())
      {
          for(int j=0; j< details.size(); j++)
          {
              myfile_def << std::fixed << details[j] << ",";
          }
          myfile_def << "\n";


          for(size_t i = 0; i < default_frontier_stats_.size(); i++)
          {
              for(int j=0; j< default_frontier_stats_[i].size(); j++)
              {
                  myfile_def << std::fixed << default_frontier_stats_[i][j] << ",";
              }
              myfile_def << "\n";
          }
          
      }
      myfile_def.close();

      if(FLAG_WSR_)
      {
        if (myfile_wsr.is_open())
        {
            for(int j=0; j< details.size(); j++)
            {
                myfile_wsr << std::fixed << details[j] << ",";
            }
            myfile_wsr << "\n";
            
            for(size_t i = 0; i < wsr_frontiers_stats_.size(); i++)
            {
                for(int j=0; j< wsr_frontiers_stats_[i].size(); j++)
                {
                    myfile_wsr << std::fixed << wsr_frontiers_stats_[i][j] << ",";
                }
                myfile_wsr << "\n";
            }
          
        }
        myfile_wsr.close();
      }
    }
  }

/**
 * @brief Get AOA from the WSR Toolbox
 * 
 */
  std::pair<std::vector<std::string>, std::vector<std::vector<double>>> Explore::generate_aoa()
  {

    WSR_Util utils;
    WSR_Module run_module(config_file_);
    reverse_csi_ = run_module.__precompute_config["input_RX_channel_csi_fn"]["value"]["csi_fn"].dump();
    
    if(displacement_type_ == "gt")
        displacement_file_ = run_module.__precompute_config["input_trajectory_csv_fn_rx"]["value"].dump();
    else if(displacement_type_ == "t265")
        displacement_file_ = run_module.__precompute_config["input_trajectory_csv_fn_rx_t265"]["value"].dump();
    else if(displacement_type_ == "odom")
        displacement_file_ = run_module.__precompute_config["input_trajectory_csv_fn_rx_odom"]["value"].dump();

    output_ = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();

    reverse_csi_.erase(remove( reverse_csi_.begin(), reverse_csi_.end(), '\"' ),reverse_csi_.end());
    displacement_file_.erase(remove( displacement_file_.begin(), displacement_file_.end(), '\"' ),displacement_file_.end());
    output_.erase(remove( output_.begin(), output_.end(), '\"' ),output_.end());
    robot_csi_ = utils.__homedir + reverse_csi_;
    robot_displacement_ = utils.__homedir + displacement_file_;

    //Neighboring robot details : MAC-ID and associated name
    for (auto it = run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].begin(); 
    it != run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].end(); ++it)
    {
        const string& tx_name =  it.key();
        auto temp =  it.value();
        std::string tx_mac_id = temp["mac_id"];
        run_module.tx_name_list[tx_mac_id] = tx_name;
    }

    std::vector<std::vector<double>> robot_displacement_data = utils.loadTrajFromCSV(displacement_file_);
    nc::NdArray<double> displacement;
    nc::NdArray<double> displacement_timestamp;
    std::vector<double> antenna_offset, antenna_offset_true;
    
    //This is not necessary for 2-antenna approach
    antenna_offset_true = run_module.__precompute_config["antenna_position_offset"]["mocap_offset"].get<std::vector<double>>(); 
    antenna_offset = run_module.__precompute_config["antenna_position_offset"]["odom_offset"].get<std::vector<double>>();
    std::cout << "log [Get_AOA]: Got offset " << std::endl;

    nc::NdArray<double> pos;
    
    auto processed_displacment = utils.formatTrajectory_v2(robot_displacement_data,antenna_offset,
                                                pos,displacement_type_,false,true);
    displacement_timestamp = processed_displacment.first;
    displacement = processed_displacment.second;
    
    run_module.calculate_AOA_using_csi_conjugate_multiple(robot_csi_,displacement,displacement_timestamp);
    
    auto all_aoa_profile = run_module.get_all_aoa_profile();
    auto all_topN_angles = run_module.get_TX_topN_angles();
    std::vector<std::string> tx_ids_all;
    std::vector<vector<double>> tx_top_aoa_peak;
    std::vector<int> AOA_angles_above_threshold;
    
    //Latest timestamp
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H%M%S",timeinfo);
    std::string time_str(buffer);


    for(auto & itr : all_aoa_profile)
    {          
        std::string tx_id = itr.first;
        auto profile = itr.second;
        std::string profile_op_fn = utils.__homedir+output_+"/"+run_module.tx_name_list[tx_id]+"_aoa_profile_"+time_str+".csv";
        // std::cout << profile_op_fn << std::endl;
        utils.writeToFile(profile,profile_op_fn);

        auto topN_angles = all_topN_angles[tx_id];
        
        tx_ids_all.push_back(run_module.tx_name_list[tx_id]);
        tx_top_aoa_peak.push_back(topN_angles.first);
    }
    
    std::cout << "log [Get_AOA]: Completed getting raw AOA" << std::endl;
    
    std::cout << "log [Get_AOA]: Filter out potential multipath angles" << std::endl;
    std::vector<vector<double>> tx_filtered_top_aoa_peak;    
    for(int val=0; val<tx_top_aoa_peak.size(); val++)
    {
      std::set<double> filtered_angles;
      for(int ii=0; ii<tx_top_aoa_peak[val].size(); ii++)
      {
        for(int jj=ii+1; jj<tx_top_aoa_peak[val].size(); jj++)
        {
          if(abs(tx_top_aoa_peak[val][ii]-tx_top_aoa_peak[val][jj]) < 15)
          {
            filtered_angles.insert(tx_top_aoa_peak[val][ii]);
            filtered_angles.insert(tx_top_aoa_peak[val][jj]);
          }
        }
      }
      std::vector<double> temp_angles;
      for(auto& aoa_angle: filtered_angles)
      {
        temp_angles.push_back(aoa_angle);
      }
      tx_filtered_top_aoa_peak.push_back(temp_angles);
    }

    return std::make_pair(tx_ids_all, tx_filtered_top_aoa_peak);
  }

  /**
   * @brief 
   * 
   * @return std::pair<std::vector<std::string>, std::vector<std::vector<double>>> 
   */
  std::vector<std::vector<double>> Explore::generate_range()
  {
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, int(range_vector_[0].size())); // define the range
    
    std::vector<double> range1, range2;
    std::vector<std::vector<double>> op;
    op.push_back(range1);
    op.push_back(range2);

    for(int n=0; n<20; n++) //Get 20 random samples from UWB node
    {
      op[0].push_back(range_vector_[0][distr(gen)]); // randomly sample uwb range value
      op[1].push_back(range_vector_[1][distr(gen)]); // 
    }

    return op;   
  }

  double Explore::wrap0to360(double val) 
  {
    val = fmod(val, 360);

    if (val < 0)
        val += 360;

    return val;
  }


}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
