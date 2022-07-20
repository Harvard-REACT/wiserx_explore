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

#include <explore/explore.h>

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
      for (int i=0; i<msg->poses.size(); i++) 
      {
        //std::cout << "Streaming ID: " << msg->poses[i].ID << std::endl;
        //std::cout << "Position: " << msg->poses[i].position << std::endl;
        //std::cout << "Orientation: " << msg->poses[i].orientation << std::endl;
        //std::cout << "\n" << std::endl;
        neighbor_pose_vec_.clear();
        if(msg->poses[i].ID != robot_id_)
        {
            geometry_msgs::Point temp = msg->poses[i].position;
            if(FLAG_noise)
            {
              static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
              temp.x = temp.x + gaussian_noise_(generator);
              temp.y = temp.y + gaussian_noise_(generator);
            }
            neighbor_pose_vec_.push_back(temp);
        }

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
    private_nh_.param("planner_frequency", planner_frequency_, 1.0);
    private_nh_.param("progress_timeout", timeout, 30.0);
    progress_timeout_ = ros::Duration(timeout);
    private_nh_.param("visualize", visualize_, false);
    private_nh_.param("use_WSR", FLAG_WSR_, true);
    private_nh_.param("noise_WSR", FLAG_noise, false);
    private_nh_.param("robot_name", robot_name_, std::string("tb3_0"));
    private_nh_.param("robot_id", robot_id_, 0);
    private_nh_.param("neighbor_name", neighbor_name_, std::string("tb3_"));
    private_nh_.param("potential_scale", potential_scale_, 1e-3);
    private_nh_.param("orientation_scale", orientation_scale_, 0.0);
    private_nh_.param("gain_scale", gain_scale_, 1.0);
    private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
    private_nh_.param("sensor_range", sensor_range_, 1.0); 
    private_nh_.param("decay_rate", decay_rate_, 0.25);
    private_nh_.param("WSR_noise_mean", noise_mean_, 0.0);
    private_nh_.param("WSR_noise_std", noise_std_, 1.0); 

    //Subscribe to the gazebo state to get the position of the other robot
    modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
    optitrackSub_ = private_nh_.subscribe<natnet_pkg::PoseArrayID> ("/optitrack_pose", 10, &Explore::optitrackMocapCB, this);
    exploration_ = private_nh_.subscribe<std_msgs::Bool> ("/true_exploration_status", 10, &Explore::explorationStatusCB, this);

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

  void Explore::makePlan()
  {
    // find frontiers
    auto pose = costmap_client_.getRobotPose();

    for (int itr=0; itr<neighbor_pose_vec_.size(); itr++)
    {
      ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, neighbor_pose_vec_[itr].x, neighbor_pose_vec_[itr].y);
    }

    ROS_DEBUG("New frontier frontier cost");
    

    // get frontiers sorted according to cost
    std::vector<frontier_exploration::Frontier> frontiers, frontier_temp;
    ROS_DEBUG("found %lu frontiers", frontiers.size());

    if(FLAG_WSR_)
    {
      frontier_temp = search_.searchFromNew(pose.position, neighbor_pose_vec_);
      ROS_DEBUG("Original cost");
      for (size_t i = 0; i < frontier_temp.size(); ++i) 
      {
        ROS_DEBUG("frontier %zd cost: %f", i, frontier_temp[i].cost);
        ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontier_temp[i].centroid.x, frontier_temp[i].centroid.y);
      }
      
      frontiers = search_.searchFromWithNeighorInfo(pose.position, neighbor_pose_vec_);
      ROS_DEBUG("New frontier frontier cost");
      for (size_t i = 0; i < frontiers.size(); ++i) 
      {
        ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
        ROS_DEBUG("frontier %zd position: (%f, %f )", i, frontiers[i].centroid.x, frontiers[i].centroid.y);
      }

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
