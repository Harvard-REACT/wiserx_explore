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
    private_nh_.param("antenna_orientation", antenna_orientation_, 0.0);

    std::cout << antenna_orientation_ << std::endl;    
    //Subscribe to the gazebo state to get the position of the other robot
    // modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
    // optitrackSub_ = private_nh_.subscribe<natnet_pkg::PoseArrayID> ("/optitrack_pose", 10, &Explore::optitrackMocapCB, this);
    neighbor_distance_ = private_nh_.subscribe<std_msgs::Float64MultiArray> ("/"+robot_name_+"/uwb_neighbors", 10, &Explore::uwbCB, this);
    t265_position_ = private_nh_.subscribe<nav_msgs::Odometry> ("/"+robot_name_+"/camera/odom/sample", 10, &Explore::positionCallbackT265, this);
    exploration_ = private_nh_.subscribe<std_msgs::Bool> ("/true_exploration_status", 10, &Explore::explorationStatusCB, this);
    velocityPub_ = private_nh_.advertise<geometry_msgs::Twist> ("/"+robot_name_+"/cmd_vel", 10);
    

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
      std::vector<std::pair<double,double>> init_pos;
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
    exploring_timer_ =
        relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                [this](const ros::TimerEvent&) { makePlan();});
  }

  Explore::~Explore()
  {
    stop();
  }


/**
 * @brief Generate the exploration plan for the current timestep
 * 
 */
  void Explore::makePlan()
  {

    //===== Start: Getting WiFi CSI data and UWB Range measurements as robot rotates in place =====
    
    //Save range data
    Flag_get_range_ = true;

    //Start CSI
    // std::string csi_start_cmd = homedir+"/catkin_ws/src/adaptive_navigation_using_aoa/control_scripts/start_csi.sh rx &";  
    // system(csi_start_cmd.c_str());

    //Start motion
    ROS_INFO("Starting motion");
    int duration_val = 8;//seconds
    auto starttime = std::chrono::high_resolution_clock::now();
    auto endtime = std::chrono::high_resolution_clock::now();
    float exp_duration;

    // while(true)
    // {
    //   velocity_cmd_.angular.z = 2.2;
    //   velocityPub_.publish(velocity_cmd_);
    //   exp_duration = std::chrono::duration<float, std::milli>(endtime - starttime).count() * 0.001;
    //   if(exp_duration > duration_val) break;
    //   endtime = std::chrono::high_resolution_clock::now(); 
    // }

    //Stop motion
    ROS_INFO("Stopping motion");
    velocity_cmd_.linear.x = 0.0;
    velocity_cmd_.angular.z = 0.0;
    for(int i=0; i<100; i++)
    {
      velocityPub_.publish(geometry_msgs::Twist());
    }
    ROS_INFO("MOtion should be stopped");
    //Stop CSI
    // std::string csi_stop_cmd = homedir+"/catkin_ws/src/adaptive_navigation_using_aoa/control_scripts/stop_csi.sh rx";
    // system(csi_stop_cmd.c_str()); //TODO: Check correct command from robot
    
    //Stop range
    Flag_get_range_=false;

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
          angle = wrap0to360(wrap0to360(aoa_val[k][j]) + wrap0to360(robot_orientation_)); //position with respect to 0 degrees 
          double position_x = (cos(angle*M_PI/180) * dist) + robot_position_x_;
          double position_y = (sin(angle*M_PI/180) * dist) + robot_position_y_;
          // std::cout << position_x <<", " << position_y << std::endl;
          est_pos.push_back(std::make_pair(position_x,position_y));
        }
    }
    neighbor_robot_est_pos_.push_back(est_pos);
    }
    //======Finished: Compute AOA and then initial position estimates ==========================

    //Particle filter to genererate new state estimates
    ROS_INFO("Running particle filter");
    particle_filter();

    for(int i=0; i<neighbor_pose_vec_.size(); i++)
    {
      std::cout << "X_pos = " << neighbor_pose_vec_[i].x << " Y pos = " << neighbor_pose_vec_[i].y << std::endl;
    }
    exit(1);
  }

  /**
   * 
   * */
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
        temp_angles.push_back(aoa_angle+antenna_orientation_);
        std::cout << "Angle (with antenna_orientation) = " << aoa_angle+antenna_orientation_ << std::endl; 
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
        std::vector<double> sample_weight;
        int p_z = 0;
        
        //Weights
        ROS_INFO("Generating Weights");
        for(int i=0;i<init_pos.size();i++)
        {
          for(int j=0; j<est_pos.size();j++)
          {
            auto state = init_pos[i];
            auto observation = est_pos[j];
            auto euc_dist = sqrt(pow((state.first - observation.first),2) + pow((state.second - observation.second),2)); 
            p_z += exp(euc_dist/2);
          }
          p_z = p_z / est_pos.size();
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
          if(new_samples.size() >= particle_threshold_) break; //Keep the number of samples constant.
        }

        ROS_INFO("New States");
        //Based on control, predict next states of the neighboring robot using the new samples
        neighbor_robot_est_pos_[k].clear();
        neighbor_robot_init_pos_[k].clear();//The new samples will be state etimates for the next iteration
        for(int i=0;i<new_samples.size();i++)
        {
          auto val = new_samples[i];
          neighbor_robot_init_pos_[k].push_back(val);
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first+1, val.second));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first-1, val.second));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first, val.second+1));
          neighbor_robot_init_pos_[k].push_back(std::make_pair(val.first, val.second-1));
        }

        ROS_INFO("Best estimated position = %f, %f", best_sample.first, best_sample.second);
        neighbor_best_position_esimtate_[k].push_back(best_sample);
        
        geometry_msgs::Point current_val;
        current_val.x = best_sample.first;
        current_val.y = best_sample.second;
        neighbor_pose_vec_.push_back(current_val);
      }

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
