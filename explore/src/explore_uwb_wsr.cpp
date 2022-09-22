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

bool sort_func(double i, double j){return (i>j);}

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

  // void Explore::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  // {
  //   int itr = 0, n_count = 0;
  //   // std::cout << FLAG_getting_next_frontier_ << std::endl;
  //   if(FLAG_getting_next_frontier_) //Control the update rate, but does not work
  //   {
  //     std::vector<std::string> name = msg->name;
  //     neighbor_id_.clear();
  //     for(std::string& val : name)
  //     {
  //       if (IsMatch(val) && val!=robot_name_) 
  //       {
  //         // std::cout << val << std::endl;
  //         neighbor_id_.push_back(n_count); 
  //       }
  //       // else
  //       // {
  //         // std::cout << "not found" << std::endl;
  //       // }
  //       n_count+=1;
  //     }

  //     std::vector<geometry_msgs::Pose> pose_vec = msg->pose;
  //     neighbor_pose_vec_.clear();
      
  //     //Store the positions of the neighboring robot
  //     for (itr=0; itr<neighbor_id_.size(); itr++)
  //     {
  //       // std::cout << robot_name_ << std::endl;
  //       // std::cout << neighbor_id_[itr] << std::endl;
  //       // std::cout << pose_vec[neighbor_id_[itr]].position << std::endl;
        
  //       if(FLAG_noise)
  //       {
  //         static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
  //         pose_vec[neighbor_id_[itr]].position.x = pose_vec[neighbor_id_[itr]].position.x + gaussian_noise_(generator);
  //         pose_vec[neighbor_id_[itr]].position.y = pose_vec[neighbor_id_[itr]].position.y + gaussian_noise_(generator);
  //       }
  //       neighbor_pose_vec_.push_back(pose_vec[neighbor_id_[itr]].position );
  //     }
      
  //   }
  // }


  // void Explore::optitrackMocapCB(const natnet_pkg::PoseArrayID::ConstPtr& msg)
  // {
  //   if(FLAG_GET_POS)
  //   {  
  //   for (int i=0; i<msg->poses.size(); i++) 
  //     {
  //       //std::cout << "Streaming ID: " << msg->poses[i].ID << std::endl;
  //       //std::cout << "Position: " << msg->poses[i].position << std::endl;
  //       //std::cout << "Orientation: " << msg->poses[i].orientation << std::endl;
  //       //std::cout << "\n" << std::endl;
  //       //neighbor_pose_vec_.clear();
  //       if(msg->poses[i].ID != robot_id_)
  //       {
  //           //ROS_INFO("Got neighbor");
	//     geometry_msgs::Point temp = msg->poses[i].position;
  //           //std::cout << "Position: " << temp << std::endl;
  //           if(FLAG_noise)
  //           {
  //             static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
  //             temp.x = temp.x + gaussian_noise_(generator);
  //             temp.y = temp.y + gaussian_noise_(generator);
  //           }
  //           neighbor_pose_vec_.push_back(temp);
	//     for (int itr=0; itr<neighbor_pose_vec_.size(); itr++)
  //   	    {   
  //     		ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, neighbor_pose_vec_[itr].x, neighbor_pose_vec_[itr].y);
  //   	    }
  //       }
  //     }
  //    FLAG_GET_POS = false;
  //   }
  // }

  void Explore::uwbCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(Flag_get_range_)
    {  
      for(int i=0;i<neighbor_count_;i++)
        range_vector_[i].push_back(msg->data[i]);
      
    }
  }


  /**==============================================================================================
   * 
   * 
   * */
  void Explore::positionCallbackT265(const nav_msgs::Odometry::ConstPtr& t265_msg) {

      tf::Quaternion q(
              t265_msg->pose.pose.orientation.x,
              t265_msg->pose.pose.orientation.y,
              t265_msg->pose.pose.orientation.z,
              t265_msg->pose.pose.orientation.w
      );

      robot_orientation_ = quaternionToYaw(q);
      robot_position_x_ = t265_msg->pose.pose.position.x;
      robot_position_y_ = t265_msg->pose.pose.position.y;
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
    private_nh_.param("neighbor_count", neighbor_count_, 1);

    //WSR related parameters:
    private_nh_.param("WSR_config_file", config_file_, std::string("/home/react-ws-1/catkin_ws/src/m-explore/explore/config/WSR_param_config.json")); 
    private_nh_.param("WSR_robot_displacement", displacement_type_, std::string("odom")); 
    private_nh_.param("use_WSR", FLAG_WSR_, true);
    private_nh_.param("noise_WSR", FLAG_noise, false);
    private_nh_.param("WSR_noise_mean", noise_mean_, 0.0);
    private_nh_.param("WSR_noise_std", noise_std_, 1.0); 
    private_nh_.param("sensor_range", sensor_range_, 1.0); 
    private_nh_.param("decay_rate", decay_rate_, 0.25);
    private_nh_.param("antenna_angular_offset", antenna_angular_offset_, 0.0);

    std::cout << antenna_angular_offset_ << std::endl;
     std::cout << sensor_range_ << std::endl;    
    //Subscribe to the gazebo state to get the position of the other robot
    // modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
    // optitrackSub_ = private_nh_.subscribe<natnet_pkg::PoseArrayID> ("/optitrack_pose", 10, &Explore::optitrackMocapCB, this);
    neighbor_distance_ = private_nh_.subscribe<std_msgs::Float64MultiArray> ("/"+robot_name_+"/uwb_neighbors", 10, &Explore::uwbCB, this);
    t265_position_ = private_nh_.subscribe<nav_msgs::Odometry> ("/"+robot_name_+"/camera/odom/sample", 10, &Explore::positionCallbackT265, this);
    exploration_ = private_nh_.subscribe<std_msgs::Bool> ("/true_exploration_status", 10, &Explore::explorationStatusCB, this);
    velocityPub_ = private_nh_.advertise<geometry_msgs::Twist> ("/"+robot_name_+"/cmd_vel", 10);
    get_csi_Pub_ = private_nh_.advertise<std_msgs::Bool> ("/"+robot_name_+"/collect_csi_data", 2);
    get_csi_data_.data = true;

    //initialize UWB data structure based on number of neighbors
    for(int i=0;i<neighbor_count_;i++)
    {
      std::vector<double> temp;
      range_vector_.push_back(temp);
    }


    //======== Initialize particle filter states===========
    //uniform AOA samples
    int icr = 360/init_angle_samples_;
    for(int i=0;i<360;)
    {
      aoa_init_.push_back(i);
      i=i+icr;
    }

    //Range samples from UWB
    ROS_INFO("Generating initial states");
    ros::Rate r(10); //match the frequency of UWB
    Flag_get_range_ = true;
    for(int i=0;i<20;i++)
    {
      ros::spinOnce();
      r.sleep();
    }
    Flag_get_range_ = false;

    // Initial position estimates for the particle filter
    for(int i=0;i<neighbor_count_;i++)
    {
      std::vector<std::pair<double,double>> init_pos, temp;
      neighbor_best_position_esimtate_.push_back(temp);
      std::vector<double> range_samples;
      auto first = range_vector_[i].begin();
      
      ROS_INFO("Subsampling range data");
      for(int j=0;j<range_vector_[i].size();)
      {
        auto last = first + 10;
        std::vector<double> temp(first,last);
        std::sort(temp.begin(), temp.end(),sort_func);
        range_samples.push_back((temp[4]+temp[5])/2); //Get median range value
        j=j+10;
        first =  range_vector_[i].begin() + j;
      }

      ROS_INFO("Generating state");
      for(int k=0;k<range_samples.size();k++)
      {
        for(int l=0;l<aoa_init_.size();l++)
        {
          // std::cout << aoa_init_[l] <<", " << range_samples[k] << std::endl;
          double init_pos_x = (cos(aoa_init_[l]*M_PI/180) * range_samples[k]) + robot_position_x_;
          double init_pos_y = (sin(aoa_init_[l]*M_PI/180) * range_samples[k]) + robot_position_y_;
          // std::cout << init_pos_x <<", " << init_pos_y << std::endl;
          // std::cout << "--------------------------------------" << std::endl;
          init_pos.push_back(std::make_pair(init_pos_x, init_pos_y));
        }
      }

      neighbor_robot_init_pos_.push_back(init_pos);
    }

    ROS_INFO("Got initial state estimates");

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

    ROS_INFO("DUration");
    std::cout << ros::Duration(1. / planner_frequency_) << std::endl;
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
      // m.scale.x = 0.2;//scale;
      // m.scale.y = 0.2;
      // m.scale.z = 0.2;
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
    ROS_DEBUG("found %lu frontiers", frontiers.size());

    
    if(FLAG_WSR_)
    {

      //Save range data
      Flag_get_range_ = true;
      robot_orientation_before_ = robot_orientation_ * 180/M_PI ; //Get robot orientation before data collection starts
      robot_position_x_before_ = robot_position_x_;
      robot_position_y_before_ = robot_position_y_;
      if (reached_goal__)
      { 
        frontiers.clear();  //Generate new frontiers when only using AOA
        frontier_temp.clear();
        reached_goal__ = false;
        sleep(1);
        //===== Start: Getting WiFi CSI data and UWB Range measurements as robot rotates in place =====
        //Start CSI
        ROS_INFO("Starting CSI data collection");
        for(int i=0; i<2; i++)
        {
          get_csi_Pub_.publish(get_csi_data_);
        }
        sleep(2);
        //Start motion
        ROS_INFO("Starting motion");
        ROS_INFO("Robot orientation: %f degrees", robot_orientation_before_);
        int duration_val = 10;//seconds
        auto starttime = std::chrono::high_resolution_clock::now();
        auto endtime = std::chrono::high_resolution_clock::now();
        float exp_duration;

        velocity_cmd_.angular.z = 1.8;
        velocityPub_.publish(velocity_cmd_);
        while(true)
        {
          exp_duration = std::chrono::duration<float, std::milli>(endtime - starttime).count() * 0.001;
          if(exp_duration > duration_val) break;
          endtime = std::chrono::high_resolution_clock::now(); 
        }

        //Stop motion
        ROS_INFO("Stopping motion");
        velocity_cmd_.linear.x = 0.0;
        velocity_cmd_.angular.z = 0.0;
        for(int i=0; i<100; i++)
        {
          velocityPub_.publish(geometry_msgs::Twist());
        }
      
        // Stop range collection and CSI and fetch data
        Flag_get_range_=false;    
        std::string fetch_data = homedir+"/catkin_ws/src/wsr_exploration/scripts/fetch_csi.sh up-board-10 192.168.1.27";
        sleep(3);
        ROS_INFO("Fetching data");
        system(fetch_data.c_str()); //TODO: Check correct command from robot
        sleep(5);

        //===== Finished: Getting CSI data and Range measurements as robot rotates in place =====

        //======Start: Compute AOA and then initial position estimates ==========================
        ROS_INFO("Getting AOA");
        std::pair<std::vector<std::string>, std::vector<std::vector<double>>> WSR_val = generate_aoa();
        std::vector<std::vector<double>> aoa_val = WSR_val.second;

        ROS_INFO("Getting Range");
        std::vector<std::vector<double>> range_val = generate_range();    
        
        //Remove old range estimates
        for(int i=0;i<neighbor_count_;i++)
          range_vector_[i].clear();
        
        ROS_INFO("Generating position observations"); 
        neighbor_robot_est_pos_.clear(); //Clear previous observations
        for(int k=0;k<neighbor_count_;k++)
        {
          std::vector<std::pair<double,double>> est_pos;
          double angle, dist;
          
          // std::cout << robot_orientation_ << ", " << robot_position_x_ << "," << robot_position_y_<<  std::endl;
          // std::cout << "-----------------------" << std::endl;

          for(int i=0;i<range_val[k].size();i++)
          {
              for(int j=0;j<aoa_val[k].size();j++)
              {
                dist = range_val[k][i];
                angle = aoa_val[k][j]; //position with respect to 0 degrees 
                // std::cout << dist <<", " << angle << std::endl;
                double position_x = (cos(angle*M_PI/180) * dist) + robot_position_x_before_;
                double position_y = (sin(angle*M_PI/180) * dist) + robot_position_y_before_;
                // std::cout << position_x <<", " << position_y << std::endl;
                // std::cout << "-----------------------------" << std::endl;
                est_pos.push_back(std::make_pair(position_x,position_y));
              }
          }
          neighbor_robot_est_pos_.push_back(est_pos);
        }
          //Particle filter to genererate new state estimates
          ROS_INFO("Running particle filter");
          
          neighbor_pose_vec_.clear();
          particle_filter();
          std::cout << "Robots X_pos: " << robot_position_x_before_ << ", Y_pos" << robot_position_y_before_ << std::endl;
          for(int i=0; i<neighbor_pose_vec_.size(); i++)
          {
            std::cout << "Neighbor X_pos = " << neighbor_pose_vec_[i].x << ", Y pos = " << neighbor_pose_vec_[i].y << " Confidence (weight) = " << neighbor_pose_vec_[i].z << std::endl;
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

      // ======Finished: Compute AOA and then initial position estimates ==========================
      }
      else //Keep refining particle filter using range only measurements till the frontier goal is reached
      {
        ROS_INFO("Just chill");
        // sleep(3); //Collect some range data
        // ROS_INFO("Getting Range");
        // std::vector<std::vector<double>> range_val = generate_range();    
        
        // //Remove old range estimates
        // for(int i=0;i<neighbor_count_;i++)
        //   range_vector_[i].clear();
        
        // ROS_INFO("Particle filter using range only measurements"); 
        // neighbor_robot_est_pos_.clear(); //Clear previous observations
        // for(int k=0;k<neighbor_count_;k++)
        // {
        //   std::vector<std::pair<double,double>> est_pos;
        //   double angle, dist;
          
        //   for(int i=0;i<range_val[k].size();i++)
        //   {
        //     for(int j=0;j<aoa_init_.size();j++)
        //     {
        //       double est_pos_x = (cos(aoa_init_[j]*M_PI/180) * range_val[k][i]) + robot_position_x_before_;
        //       double est_pos_y = (sin(aoa_init_[j]*M_PI/180) * range_val[k][i]) + robot_position_y_before_;
        //       est_pos.push_back(std::make_pair(est_pos_x, est_pos_y));
        //     }
        //   }
        //   neighbor_robot_est_pos_.push_back(est_pos);
        // }

        // //Particle filter to genererate new state estimates
        // ROS_INFO("Running particle filter");
        
        // neighbor_pose_vec_.clear();
        // particle_filter();
        // std::cout << "Robots X_pos: " << robot_position_x_before_ << ", Y_pos" << robot_position_y_before_ << std::endl;
        // for(int i=0; i<neighbor_pose_vec_.size(); i++)
        // {
        //   std::cout << "Neighbor X_pos = " << neighbor_pose_vec_[i].x << ", Y pos = " << neighbor_pose_vec_[i].y << " Confidence (weight) = " << neighbor_pose_vec_[i].z << std::endl;
        // }
      }
    }
    else
    {
      if(reached_goal__)
      {
        // reached_goal__=false;
        // sleep(5);
        // //Start motion
        // ROS_INFO("Starting motion");
        // ROS_INFO("Robot orientation: %f degrees", robot_orientation_before_);
        // int duration_val = 9;//seconds
        // auto starttime = std::chrono::high_resolution_clock::now();
        // auto endtime = std::chrono::high_resolution_clock::now();
        // float exp_duration;

        // velocity_cmd_.angular.z = 2.0;
        // velocityPub_.publish(velocity_cmd_);
        // while(true)
        // {
        //   exp_duration = std::chrono::duration<float, std::milli>(endtime - starttime).count() * 0.001;
        //   if(exp_duration > duration_val) break;
        //   endtime = std::chrono::high_resolution_clock::now(); 
        // }

        // //Stop motion
        // ROS_INFO("Stopping motion");
        // velocity_cmd_.linear.x = 0.0;
        // velocity_cmd_.angular.z = 0.0;
        // for(int i=0; i<100; i++)
        // {
        //   velocityPub_.publish(geometry_msgs::Twist());
        // }
        // sleep(5);
      
        // using neighrbor info just to collect stats and not for utility calculation
        frontiers.clear();  //Generate new only when a goal is reached
        frontiers = search_.searchFromNew(pose.position, neighbor_pose_vec_); 
        
        ROS_DEBUG("Original cost");
        for (size_t i = 0; i < frontiers.size(); ++i) 
        {
          ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
          ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontiers[i].centroid.x, frontiers[i].centroid.y);
        }

        writeToFile(frontiers,frontier_temp,fn);
      }
      else
      {
        ROS_INFO("Using PF with range only");
        sleep(5);
      }

    }
    
    // publish frontiers as visualization markers
    if (visualize_) {
      visualizeFrontiers(frontiers);
    }
    
    // if(frontiers.size() == 1)
    // {
    //   frontier->centroid = frontier->furthest;
    //   frontier->centroid_distance = frontier->min_distance; //just a heuristic
    // }

    if(frontiers.empty() || exploration_done_) 
      {
        ROS_INFO("Empty frontiers detected. Ending Exploration");
        stop();
        writeToFile(frontier_temp,frontiers, fn);
        return;
      }

    // find non blacklisted frontier
    frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                        [this](const frontier_exploration::Frontier& f) {
                          return goalOnBlacklist(f.centroid);
                        });
    
    
    if (frontier == frontiers.end()) 
    {
      stop();
      return;
    }
    geometry_msgs::Point target_position = frontier->centroid;
    frontier->min_distance =  sqrt(pow((double(target_position.x) - double(robot_position_x_)), 2.0) +
                                   pow((double(target_position.y) - double(robot_position_y_)), 2.0));

    // time out if we are not making any progress
    same_goal__ = prev_goal_ == target_position;
    prev_goal_ = target_position;
    if (!same_goal__ || prev_distance_ > frontier->min_distance) {
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
    if (same_goal__) {
      return;
    }
    
    // reached_goal__ = true;
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

    if (status == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("==============================REACHED GOAL SUCCESSFULLY==============================");
      reached_goal__=true;
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
    
    for(int k=0;k<neighbor_count_;k++)
    {
      std::vector<std::pair<double,double>> fina_est_posList = neighbor_best_position_esimtate_[k];
      std::string fn1 = "/home/react-ws-1/catkin_ws/src/wsr_exploration/data/init_pose_robot_"+std::to_string(k);
      writePosToFile(fina_est_posList,fn1);
    }
    exit(1);
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

    std::vector<std::vector<double>> robot_displacement_data = utils.loadTrajFromCSV(robot_displacement_);
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
    
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    for(int val=0; val<tx_top_aoa_peak.size(); val++)
    {
      std::cout << "[";
      for(int val2 =0; val2<tx_top_aoa_peak[val].size();val2++)
        std::cout << tx_top_aoa_peak[val][val2] <<",";

      std::cout << "]"<< std::endl;
      std::cout << "-----------------------------" << std::endl;
    }

    std::cout << "log [Get_AOA]: Filter out potential multipath angles" << std::endl;
    std::vector<vector<double>> tx_filtered_top_aoa_peak;    
    
    for(int val=0; val<tx_top_aoa_peak.size(); val++)
    {
      std::set<double> filtered_angles;
      double threshold = 1.0;

      while(filtered_angles.size()==0)
      {
        for(int ii=0; ii<tx_top_aoa_peak[val].size(); ii++)
        {
          for(int jj=ii+1; jj<tx_top_aoa_peak[val].size(); jj++)
          {
            if(abs(tx_top_aoa_peak[val][ii]-tx_top_aoa_peak[val][jj]) < threshold)
            {
              filtered_angles.insert(tx_top_aoa_peak[val][ii]);
              filtered_angles.insert(tx_top_aoa_peak[val][jj]);
            }
          }
        }
        threshold+=5; // iterately increment the threshold
      }

      std::vector<double> temp_angles;
      std::cout << "Offset: " << antenna_angular_offset_ << std::endl;
      std::cout << "Robot orientation: " << robot_orientation_before_ << std::endl;
      for(auto& aoa_angle: filtered_angles)
      {
        double aoa_antennta_offset = wrap0to360(wrap0to360(aoa_angle)+wrap0to360(antenna_angular_offset_));
        double aoa_antenna_offset_robot_orientation = wrap0to360(wrap0to360(aoa_angle)+wrap0to360(antenna_angular_offset_)+ wrap0to360(robot_orientation_before_));
        std::cout << "Angle  = " << aoa_angle << std::endl;
        std::cout << "Angle (wrap)  = " << wrap0to360(aoa_angle) << std::endl; 
        std::cout << "Angle (considering antenna_angular_offset) = " << aoa_antennta_offset << std::endl; 
        std::cout << "Angle (w.r.t 0 degrees after including robot orientation) = " << aoa_antenna_offset_robot_orientation << std::endl; 
        temp_angles.push_back(aoa_antenna_offset_robot_orientation);
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
    
    std::vector<std::vector<double>> op;
    for(int i=0; i<neighbor_count_; i++)
    {
      std::vector<double> range;
      op.push_back(range);
    }

    for(int n=0; n<20; n++) //Get 20 random samples from UWB node
    {
      for(int i=0; i<neighbor_count_; i++)
      {
        // std::cout << "Range Sample:" << range_vector_[i][distr(gen)] << std::endl;
        op[i].push_back(range_vector_[i][distr(gen)]);// randomly sample uwb range value
      } 
    }

    return op;   
  }

  /**
   * 
   * */
  void Explore::particle_filter()
  {
      for(int k=0;k<neighbor_count_;k++)
      {
        
        std::vector<std::pair<double,double>> init_pos = neighbor_robot_init_pos_[k];
        std::vector<std::pair<double,double>> est_pos = neighbor_robot_est_pos_[k];
        std::string fn1 = "/home/react-ws-1/init_pose_robot_"+std::to_string(k);
        std::string fn2 = "/home/react-ws-1/est_pose_"+std::to_string(k);

        // writePosToFile(init_pos,fn1);
        // writePosToFile(est_pos,fn2);
        
        std::vector<double> sample_weight;
        double p_z = 0;
        
        //Weights
        ROS_INFO("Generating Weights");
        for(int i=0;i<init_pos.size();i++)
        {
          for(int j=0; j<est_pos.size();j++)
          {
            auto state = init_pos[i];
            auto observation = est_pos[j];
            double euc_dist = sqrt(pow((state.first - observation.first),2) + pow((state.second - observation.second),2)); 
            p_z += exp(-euc_dist/2);
          }
          p_z = p_z / est_pos.size();
          // std::cout << p_z << std::endl;
          sample_weight.push_back(p_z);
        }

        //Resample
        ROS_INFO("Resampling");
        std::vector<std::pair<double,double>> new_samples;
        float threshold = 0, best_sample_weight=0;
        std::pair<double, double> best_sample;
        srand((unsigned)time(NULL));
        while(true)
        {
          threshold = (float)rand()/RAND_MAX;
          // std::cout << threshold << std::endl;
          for(int i=0;i<sample_weight.size();i++)
          {
            if(sample_weight[i] > threshold) //TODO: Improve sampling strategy later
            {
              new_samples.push_back(init_pos[i]);
              if(sample_weight[i] > best_sample_weight) 
              {
                best_sample_weight = sample_weight[i];  //Keep track of the most likely position estimate
                best_sample = init_pos[i];
              }
              
            }
          }
          // std::cout << new_samples.size() << std::endl;
          if(new_samples.size() >= particle_threshold_) break; //Keep the number of samples constant.
        }

        ROS_INFO("Subsampling from the new states");
        std::cout << "Initial Sample size: " << new_samples.size() << std::endl;
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> distr(0, int(new_samples.size())); // define the range
        
        std::vector<std::pair<double, double>> new_samples_sub;
        for(int n=0; n<particle_threshold_; n++) //Get 20 random samples from UWB node
        {
          new_samples_sub.push_back(new_samples[distr(gen)]);// randomly sample uwb range value
        }
        std::cout << "Sub Sampled state: " << new_samples_sub.size() << std::endl;

        ROS_INFO("Generating next control states");
        //Based on control, predict next states of the neighboring robot using the new samples
        //Control is assumed to be 1 meter displacement of the robot
        neighbor_robot_init_pos_[k].clear();//The new samples will be state etimates for the next iteration
        for(int i=0;i<new_samples_sub.size();i++)
        {
          auto val = new_samples_sub[i];
          neighbor_robot_init_pos_[k].push_back(val);
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first+2, val.second));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first-2, val.second));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first, val.second+2));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first, val.second-2));
        }

        ROS_INFO("Best estimated position = %f, %f", best_sample.first, best_sample.second);
        neighbor_best_position_esimtate_[k].push_back(best_sample);
        
        geometry_msgs::Point current_val;
        current_val.x = (float) best_sample.first;
        current_val.y = (float) best_sample.second;
        current_val.z = (float) best_sample_weight;

        ROS_INFO("Assigned");

        neighbor_pose_vec_.push_back(current_val);
      }
      
      ROS_INFO("Particle filter iteration Done");

  }

  double Explore::wrap0to360(double val) 
  {
    val = fmod(val, 360);

    if (val < 0)
        val += 360;

    return val;
  }

  double Explore::quaternionToYaw(const tf::Quaternion& q) 
  {
    double yaw = 0.0;

    if (validateQuaternion(q)) {
        tf::Matrix3x3 m(q);

        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    }

    return yaw;
  }

  bool Explore::validateQuaternion(const tf::Quaternion& quat) 
  {
    return (quat.getW() != 0 || quat.getX() != 0 || quat.getY() != 0 || quat.getZ() != 0);
  }

  /**
   * 
   * */
  void Explore::writePosToFile(std::vector<std::pair<double,double>>& pos_file,
                               std::string fn)
  { 
    std::cout.precision(4);
    const auto p1 = std::chrono::system_clock::now();
    std::string ts = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());
    std::string fn1 = fn+"_"+ts+".csv";
    std::ofstream myfile_def (fn1);
    std::cout << fn1 << std::endl;
    std::vector<std::string> details {"x", "y"}; //index 1 means the top most frontier at each iteration which will then be selected

    if (myfile_def.is_open())
    {
        for(int j=0; j< details.size(); j++)
        {
            myfile_def << std::fixed << details[j] << ",";
        }
        myfile_def << "\n";


        for(size_t i = 0; i < pos_file.size(); i++)
        {
          myfile_def << std::fixed << pos_file[i].first << "," << pos_file[i].second;
          myfile_def << "\n";
        }    
    }
    myfile_def.close();
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
