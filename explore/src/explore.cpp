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

std::string fn = "/home/react-ws-1/catkin_ws/src/wsr_exploration/data/mexplore_data/";
std::default_random_engine generator;
bool FLAG_noise = false;
auto start_val = std::chrono::high_resolution_clock::now();
auto stop_val = std::chrono::high_resolution_clock::now();
auto start_exploration = std::chrono::high_resolution_clock::now();
auto end_exploration = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
double _previous_range_measurement = 2.0;
double __range_to_use = 0.0;
double __bearing_to_use =  0.0;

// static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.3); //range noise 0  mean and 30cm stddev in meters 
// static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.3); //bearing noise  0 mean and 17 deg stddev in radians

//Used for flight lab vicon hardware experiments - somehow has issues with tb3_2
static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.1); //range noise 0  mean and 10cm stddev in meters 
static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.1); //bearing noise  0 mean and 5 deg stddev in radians

// static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.1); //range noise mean and stddev in meters 
// static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg

// static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.05); //range noise mean and stddev in meters 
// static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg

// static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.01); //range noise mean and stddev in meters 
// static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg


//Used mostly in sim
// static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.2); //range noise mean and stddev in meters 20cm
// static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.17); //bearing noise mean and stddev in radians 10 deg

std::vector<std::string> name_vicon_hardware = {"tb3_1", "tb3_2"};
float ekf_velocity_x = 0.1;
float ekf_velocity_y = 0.1;


std::string exec(const char* cmd) {
  char buffer[128];
  std::string result = "";
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
      throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
      result += buffer;
  }
  return result;
}


namespace explore
{

  /** 
   * Find matching value for neighbor names when using gazebo based positions
   * */
  bool  Explore::IsMatch(std::string& val)
  {
    return (val.find(neighbor_name_) != std::string::npos);
  }


  /** 
   * Find matching value for getting environment dimensions.
   * */
  bool  Explore::IsMatchDim(std::string& val)
  {
    return (val.find(__dim_object_name) != std::string::npos);
  }

  // /** 
  //  * @brief Get positions of neighboring robots in gazebo (global frame)
  //  * */
  // void Explore::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  // {
  //   int itr = 0, n_count = 0;
  //   std::vector<std::string> name = msg->name;
  //   neighbor_id_.clear();
    
  //   //Get the id of all other robots except itself
  //   for(std::string& val : name)
  //   {
  //     if (IsMatch(val) && val!=robot_name_) 
  //     {
  //       neighbor_id_.push_back(n_count); 
  //     }
  //     n_count+=1;
  //   }

  //   //Store the positions (can be modified to add noise) of the neighboring robot
  //   std::vector<geometry_msgs::Pose> pose_vec = msg->pose;    
    
  //   for (itr=0; itr<neighbor_id_.size(); itr++)
  //   {        
  //     if(FLAG_noise)
  //     {
  //       static std::normal_distribution<double> gaussian_noise_(noise_mean_, noise_std_);
  //       pose_vec[neighbor_id_[itr]].position.x = pose_vec[neighbor_id_[itr]].position.x + gaussian_noise_(generator);
  //       pose_vec[neighbor_id_[itr]].position.y = pose_vec[neighbor_id_[itr]].position.y + gaussian_noise_(generator);
  //     }
      
  //     current_neighbor_pose_vec_.push_back(pose_vec[neighbor_id_[itr]].position );
  //   }
  // }


  /** 
   * @brief Get positions of neighboring robots in gazebo (global frame)
   * */
  void Explore::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    std::vector<std::string> name = msg->name;
    std::vector<geometry_msgs::Pose> pose_vec = msg->pose;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
    double world_x, world_y;
    std::vector<frontier_exploration::Frontier> frontiers_copy;
    
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    if(duration.count() > measurement_interval__) // publish every 10 seconds
    {
      timestep__+=1;
      wsr_exploration::QuadmapViz msg;
      
      for(int itr=0; itr<name.size(); itr++)
      {
        if(IsMatch(name[itr]))
        {
          wsr_exploration::RelativeEstimate neighboring_robot;   
          quadmap::Robot new_robot_track;

          auto search_val = robot_information__.find(name[itr].c_str());
          if( search_val == robot_information__.end())
          {
            new_robot_track.robot_id = itr;
            robot_information__.insert({name[itr].c_str(), new_robot_track});
            // ROS_DEBUG("NEW: Name, robot_tau, robot_id: %s, %d, %d", name[itr].c_str(), robot_information__[name[itr].c_str()].robot_tau, robot_information__[name[itr].c_str()].robot_id);
          }
          
          //TODO: The positions need to be in map coordinates
          double cmOrigin_X = costmap2d->getOriginX();
          double cmOrigin_Y = costmap2d->getOriginY();
          ROS_INFO("**** Origin_X, Origin_Y: %f, %f **** ", cmOrigin_X, cmOrigin_Y);
          costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
          unsigned int sizeX = costmap2d->getSizeInCellsX();
          unsigned int sizeY = costmap2d->getSizeInCellsY();
          ROS_INFO("**** getSizeInCellsX, getSizeInCellsY: %d, %d **** ", sizeX, sizeY);
          quadmap::Node position_node(mx__, my__, robot_information__[name[itr].c_str()].robot_tau, robot_information__[name[itr].c_str()].robot_id,timestep__);   

          if(name[itr]!=robot_name_)
          {
            // static std::normal_distribution<float> gaussian_noise_(noise_mean_, noise_std_); //Noise is in world coordinates
            // noise_x_ = gaussian_noise_(generator);
            // noise_y_ = gaussian_noise_(generator);
            // pose_vec[itr].position.x = pose_vec[itr].position.x + gaussian_noise_(generator);
            // pose_vec[itr].position.y = pose_vec[itr].position.y + gaussian_noise_(generator);
            // costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
            // position_node.add_position_noise(mx__, my__);            
            // std::vector<double> cov_array{pow(noise_std_,2.0), pow(noise_std_,2.0)}; //This is the covariance for the noise in world coordinates.
            std::vector<double> cov_array{0.1, 0, 0, 0.1};
            
            //Set omega to 1
            position_node.updateOmega(cov_array[0], cov_array[3]); //With true positions, omega should be 1

            // costmap2d->mapToWorld(position_node.true_mx, position_node.true_my, world_x, world_y);          
            // neighboring_robot.true_position.x = world_x;
            // neighboring_robot.true_position.y = world_y;
            
            // costmap2d->mapToWorld(position_node.est_mx, position_node.est_my, world_x, world_y);          
            // neighboring_robot.estimated_position.x = world_x;
            // neighboring_robot.estimated_position.y = world_y;
    
                //Keep track of the latest position esimates for using in beta parameter
            geometry_msgs::Point temp;
            temp.x = pose_vec[itr].position.x;
            temp.y = pose_vec[itr].position.y;
            current_rel_positions__.push_back(temp);


            neighboring_robot.true_map_position.x = position_node.true_mx;
            neighboring_robot.true_map_position.y = position_node.true_my;        
            neighboring_robot.estimated_map_position.x = position_node.est_mx;
            neighboring_robot.estimated_map_position.y = position_node.est_my;

            neighboring_robot.covariance_meter_sq = cov_array;
            neighboring_robot.status = 1;
            neighboring_robot.robot_id = position_node.getRobotID();
            msg.other_robots.push_back(neighboring_robot);
          }
          else
          {
            // costmap2d->mapToWorld(position_node.true_mx, position_node.true_my, world_x, world_y);  
            // msg.own_position.x = world_x;
            // msg.own_position.y = world_y;
            msg.own_position.x = position_node.true_mx;
            msg.own_position.y = position_node.true_my;
            msg.robot_id = position_node.getRobotID();
            robot_id_ = msg.robot_id; //Not that the robot id will not change during an instance of simulation
          }
          
          robot_information__[name[itr]].node_information.push(position_node);
          base_quadmap_.insert_till_end(position_node); 
          base_quadmap_.update_quadmap_ID(itr);
        }
      }

      //Get frontiers centroids.
      frontiers_copy = frontier_temp__;
      // if(frontier_temp__.size()>0) 
      for(auto frontier_val : frontiers_copy)
      {
        // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
        wsr_exploration::FrontierInfo fc_point;
        costmap2d->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
        fc_point.centroid.x = fmx__;
        fc_point.centroid.y = fmy__;
        fc_point.size=frontier_val.size;
        fc_point.information_gain=frontier_val.information_gain;
        fc_point.centroid_distance=frontier_val.centroid_distance;
        fc_point.utility=frontier_val.cost;
        fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
        msg.frontiers.push_back(fc_point);
      }

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);

      start_val = std::chrono::high_resolution_clock::now();
    }
    
    stop_val = std::chrono::high_resolution_clock::now();

  }


void get_noisy_range_and_bearing_from_groundtruth(geometry_msgs::Pose& robot_i_positions,
                                                                geometry_msgs::Pose& robot_j_positions,
                                                                std::vector<double>& output)
{ 
  float diff_x = robot_j_positions.position.x - robot_i_positions.position.x ;
  float diff_y = robot_j_positions.position.y - robot_i_positions.position.y;
  
  
  float noisy_range = sqrt(pow((diff_x),2.0) + pow((diff_y),2.0)); // meters
  float noisy_bearing = atan2(diff_y, diff_x); // radians
  ROS_INFO("True range (meters), bearing (degrees) = %f, %f", noisy_range, noisy_bearing*180/3.14);

  noisy_range = noisy_range +  range_measurement_gaussian_noise_(generator); // meters
  noisy_bearing = noisy_bearing +  bearing_measurement_gaussian_noise_(generator); // radians
  ROS_INFO("Noisy range (meters), bearing(degrees): = %f, %f", noisy_range, noisy_bearing*180/3.14);
  
  output.push_back(noisy_range);
  output.push_back(noisy_bearing);
  ROS_DEBUG("output range (meters), bearing(degrees): = %f, %f", output[0], output[1]*180/3.14);
}


void Explore::modelStateCallbackFilter(const gazebo_msgs::ModelStates::ConstPtr& input_msg)
{
    std::vector<std::string> name = input_msg->name;
    std::vector<geometry_msgs::Pose> pose_vec = input_msg->pose;
    geometry_msgs::Pose robot_i_positions;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
    double world_x, world_y;
    std::vector<frontier_exploration::Frontier> frontiers_copy;
    std::vector<double> cov_array{0,0,0,0};
    
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    if(duration.count() > measurement_interval__) // publish every 10 seconds
    {
      timestep__+=1;
      wsr_exploration::QuadmapViz msg;
      current_rel_positions__.clear();
      
      //First find the current position of the robot i
      for(int itr=0; itr<name.size(); itr++)
      {
        if(name[itr]==robot_name_)
        {
          robot_i_positions = pose_vec[itr];
          robot_id_ = itr; //Not that the robot id will not change during an instance of simulation
          costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
          msg.own_position.x = mx__;
          msg.own_position.y = my__;
          msg.robot_id = itr;
          ROS_INFO("Own position of robot(ID) = %u, %u, %d", mx__,my__, msg.robot_id);
          
          //Add own's position to enable estimating of quadmap fill and terminating.
          quadmap::Node position_node(mx__, my__, my_tau__, robot_id_,timestep__);
          base_quadmap_.insert_till_end(position_node); 
          break;
        }

      }
      
      for(int itr=0; itr<name.size(); itr++)
      {
        if(IsMatch(name[itr]))
        {
          wsr_exploration::RelativeEstimate neighboring_robot;             
          double est_x_j=0,est_y_j=0, one_shot_position_x, one_shot_position_y;
        
          if(name[itr]!=robot_name_) //Only do this for robot j in the neighborhood of robot i
          {
            //Get true positions of the neighboring robot and generate range and bearing 
            measurement_output__.clear();
            get_noisy_range_and_bearing_from_groundtruth(robot_i_positions, pose_vec[itr], measurement_output__);
            ROS_INFO("Got estimates");
            ROS_INFO("Noisy range (meters), bearing(degrees): = %f, %f", measurement_output__[0], measurement_output__[1]*180/3.14);


            auto search_val = robot_information__.find(name[itr].c_str());
            if( search_val == robot_information__.end())
            {
              ROS_INFO("Creating Robot j track");
              quadmap::Robot new_robot_track;
              new_robot_track.robot_id = itr;
              robot_information__.insert({name[itr].c_str(), new_robot_track});

              double first_est_x = robot_i_positions.position.x + measurement_output__[0]*cos(measurement_output__[1]);
              double first_est_y = robot_i_positions.position.y + measurement_output__[0]*sin(measurement_output__[1]);
              
              ROS_INFO("True position = %f, %f", pose_vec[itr].position.x, pose_vec[itr].position.y);
              ROS_INFO("First estimate = %f, %f", first_est_x, first_est_y);
              
              
              VectorXd robot_j_first_estimate(4) ;
              ROS_INFO("Created Kalman filter object");
              robot_j_first_estimate << first_est_x, first_est_y, 0.2, 0.2; // x,y,vx,vy - constant velocity model
              ROS_INFO("Initializaing EKF Track");
              wsr_state_estimation::ExtendedKalmanFilter new_robot_state_estimation_track (robot_j_first_estimate, measurement_interval__); //Run prediction every second
              ekf_robot_track__.insert({name[itr].c_str(),new_robot_state_estimation_track});
              est_x_j = robot_j_first_estimate[0];
              est_y_j = robot_j_first_estimate[1];
              ROS_INFO("First estimate = %f, %f", est_x_j, est_y_j);

              ekf_robot_track__[name[itr].c_str()].predict(); //Prediction comes from model

                      
              // ROS_INFO("Created Particle filter object");
              // VectorXd robot_j_first_estimate(4) ;
              // robot_j_first_estimate << first_est_mx, first_est_my, 0.1, 0.1; // x,y,vx,vy - constant velocity model
              // wsr_state_estimation::ParticleFilter new_robot_state_estimation_track (robot_j_first_estimate, measurement_interval__, 0.5,0.01, 0.2,0.2);
              // pf_robot_track__.insert({name[itr].c_str(),new_robot_state_estimation_track});
              // est_mx_j = robot_j_first_estimate[0];
              // est_my_j = robot_j_first_estimate[1];
              // ROS_INFO("First estimate = %f, %f", est_mx_j, est_my_j);
            }
            else if(search_val->second.robot_tau == 1) 
            {
                            
              //Perform state_estimation of robot j
              ROS_INFO("Found track for functional robot %s with tau: %d", search_val->first.c_str(), search_val->second.robot_tau);
              VectorXd z(2);
              //Predict for next timestep
              ekf_robot_track__[name[itr].c_str()].predict();
              
              z << measurement_output__[0] , measurement_output__[1];
              ekf_robot_track__[name[itr].c_str()].update(z,robot_i_positions); //Correct the prediction based on the new measurement
                
              est_x_j = ekf_robot_track__[name[itr].c_str()].x[0];
              est_y_j = ekf_robot_track__[name[itr].c_str()].x[1];

              //Get covariance
              cov_array[0] = ekf_robot_track__[name[itr].c_str()].P(0,0); //cov_x
              cov_array[1] = ekf_robot_track__[name[itr].c_str()].P(0,1); //cov_xy
              cov_array[2] = ekf_robot_track__[name[itr].c_str()].P(1,0); //cov_yx
              cov_array[3] = ekf_robot_track__[name[itr].c_str()].P(1,1); //cov_y
              

              // pf_robot_track__[name[itr].c_str()].predict();
              // VectorXd z(2);
              // z << measurement_output__[0] , measurement_output__[1];
              // pf_robot_track__[name[itr].c_str()].updateWeights(z,robot_i_positions);
              // VectorXd est_j(4); 
              // est_j =  pf_robot_track__[name[itr].c_str()].getEstimate();
              // est_mx_j = est_j(0);
              // est_my_j = est_j(1);
              // MatrixXd cov = pf_robot_track__[name[itr].c_str()].computeCovariance(est_j);
              // cov_array[0] = cov(0,0); //cov_x
              // cov_array[1] = cov(1,1); //cov_y

              ROS_INFO("True position = %f, %f", pose_vec[itr].position.x, pose_vec[itr].position.y);
              ROS_INFO("Predicted estimate = %f, %f", est_x_j, est_y_j);

            }
            
            //Only do next steps for functional robot
            search_val = robot_information__.find(name[itr].c_str());
            if(search_val->second.robot_tau == 1)
            { 
              //Keep track of the latest position esimates for using in beta weight of the infomration gain.
              geometry_msgs::Point temp;
              temp.x = est_x_j;
              temp.y = est_y_j;
              current_rel_positions__.push_back(temp);

              unsigned int sizeX = costmap2d->getSizeInCellsX();
              unsigned int sizeY = costmap2d->getSizeInCellsY();
              ROS_INFO("**** getSizeInCellsX, getSizeInCellsY: %d, %d **** ", sizeX, sizeY);
              
              //Initialize node with true relative position of the other robot
              costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
              quadmap::Node position_node(mx__, my__, robot_information__[name[itr].c_str()].robot_tau, robot_information__[name[itr].c_str()].robot_id,timestep__);

              //Add estimated position
              costmap2d->worldToMap(est_x_j, est_y_j, mx__, my__);
              position_node.add_position_noise(mx__, my__);            
              position_node.updateOmega(cov_array[0], cov_array[3]); //Update the term based on the covariance
              robot_information__[name[itr]].node_information.push(position_node);
              base_quadmap_.insert_till_end(position_node); 
              
                        
              neighboring_robot.true_map_position.x = position_node.true_mx;
              neighboring_robot.true_map_position.y = position_node.true_my; 
              neighboring_robot.estimated_map_position.x = position_node.est_mx;
              neighboring_robot.estimated_map_position.y = position_node.est_my;
              // base_quadmap_.update_quadmap_ID(itr);


              one_shot_position_x = robot_i_positions.position.x + measurement_output__[0]*cos(measurement_output__[1]);
              one_shot_position_y = robot_i_positions.position.y + measurement_output__[0]*sin(measurement_output__[1]);
              costmap2d->worldToMap(one_shot_position_x, one_shot_position_y, mx__, my__);
              neighboring_robot.one_shot_map_position.x = mx__;
              neighboring_robot.one_shot_map_position.y = my__;

              //Publisher message
              ROS_INFO("Adding other neighbor info");       
              neighboring_robot.true_range_bearing = measurement_output__;
              neighboring_robot.est_range_bearing = measurement_output__;
              neighboring_robot.filter_predicted_range_bearing = ekf_robot_track__[name[itr].c_str()].range_bearing__;
              neighboring_robot.filter_residual_error_range_bearing = ekf_robot_track__[name[itr].c_str()].residual_error__;
              neighboring_robot.error_meters = sqrt(pow((pose_vec[itr].position.x-est_x_j),2) + pow((pose_vec[itr].position.y-est_y_j),2));

              neighboring_robot.covariance_meter_sq = cov_array;
              neighboring_robot.status = 1;
              neighboring_robot.robot_id = position_node.getRobotID();
              msg.other_robots.push_back(neighboring_robot);
              ROS_INFO("Added neighbor info");
            }
          }
        }
      }

      //Get frontiers centroids.
      frontiers_copy = frontier_temp__;
      // if(frontier_temp__.size()>0) 
      for(auto frontier_val : frontiers_copy)
      {
        // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
        wsr_exploration::FrontierInfo fc_point;
        costmap2d->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
        fc_point.centroid.x = fmx__;
        fc_point.centroid.y = fmy__;
        fc_point.size=frontier_val.size;
        fc_point.information_gain=frontier_val.information_gain;
        fc_point.centroid_distance=frontier_val.centroid_distance;
        fc_point.utility=frontier_val.cost;
        fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
        msg.frontiers.push_back(fc_point);
      }

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);

      start_val = std::chrono::high_resolution_clock::now();
    }
    // else if(duration.count() > 1)
    // {
    //   //Just run prediction
    //   for(int itr=0; itr<name.size(); itr++)
    //   {
    //     auto search_val = robot_information__.find(name[itr].c_str());
    //     if( search_val != robot_information__.end())
    //     {
    //       ekf_robot_track__[name[itr].c_str()].predict();
    //     }
    //   }
    // }

    stop_val = std::chrono::high_resolution_clock::now();

  }

/**
 * For vicon hardware experiments with two robots.
 * Robot ids are number starting 1
*/
void Explore::ViconCombinedStateCallbackFilter(const geometry_msgs::PoseArray::ConstPtr& input_msg)
{
    // std::vector<std::string> name = input_msg->name;
    std::vector<geometry_msgs::Pose> pose_vec = input_msg->poses;
    geometry_msgs::Pose robot_i_positions, robot_i_vicon_position;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap(); 
    double world_x, world_y;
    std::vector<frontier_exploration::Frontier> frontiers_copy;
    std::vector<double> cov_array{0,0,0,0};
    
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    if(duration.count() > measurement_interval__) // publish every 10 seconds
    {
      timestep__+=1;
      wsr_exploration::QuadmapViz msg;
      current_rel_positions__.clear();
      
      //First find the current position of the robot i
      for(int itr=0; itr<pose_vec.size(); itr++)
      {
        if(itr==robot_id_-1)
        {
          ROS_INFO("itr: %d", itr);
          ROS_INFO("robot_id_: %d", robot_id_);
          
          //For hardware implementation using global frame, we need to start at 0,0.
          // robot_i_positions = pose_vec[itr]; 
          robot_i_vicon_position = pose_vec[itr]; 
          
          //Get position estimate from SLAM
          robot_i_positions = costmap_client_.getRobotPose();
          costmap2d->worldToMap(robot_i_positions.position.x, robot_i_positions.position.y, mx__, my__);
          msg.own_position.x = mx__;
          msg.own_position.y = my__;
          msg.robot_id = robot_id_;
          ROS_INFO("Own position of robot(ID) in world = %f, %f, %d", robot_i_vicon_position.position.x,robot_i_vicon_position.position.y, msg.robot_id);
          ROS_INFO("Own position of robot(ID) in map = %d, %d, %d", mx__,my__, msg.robot_id);
          

          //Add own's position to enable estimating of quadmap fill and terminating.
          quadmap::Node position_node(mx__, my__, my_tau__, robot_id_,timestep__);
          base_quadmap_.insert_till_end(position_node); 
          break;
        }

      }
      
      for(int itr=0; itr<pose_vec.size(); itr++)
      {
        // if(IsMatch(name[itr]))
        // {
          wsr_exploration::RelativeEstimate neighboring_robot;             
          double est_x_j=0,est_y_j=0, one_shot_position_x, one_shot_position_y;
        
          if(itr!=robot_id_-1)
          {
            //Use the vicon mocap positions of the neighboring robot to generate range and bearing measurements
            measurement_output__.clear();
            get_noisy_range_and_bearing_from_groundtruth(robot_i_vicon_position, pose_vec[itr], measurement_output__);
            ROS_INFO("Got estimates");
            ROS_INFO("Noisy range (meters), bearing(degrees): = %f, %f", measurement_output__[0], measurement_output__[1]*180/3.14);

            //Estimate the relative position of the neighboring robot
            auto search_val = robot_information__.find(name_vicon_hardware[itr].c_str());
            if( search_val == robot_information__.end())
            {
              ROS_INFO("Creating Robot j track");
              quadmap::Robot new_robot_track;
              new_robot_track.robot_id = itr+1;
              robot_information__.insert({name_vicon_hardware[itr].c_str(), new_robot_track});

              double first_est_x = robot_i_positions.position.x + measurement_output__[0]*cos(measurement_output__[1]);
              double first_est_y = robot_i_positions.position.y + measurement_output__[0]*sin(measurement_output__[1]);
              
              ROS_INFO("True position = %f, %f", pose_vec[itr].position.x, pose_vec[itr].position.y);
              ROS_INFO("First estimate = %f, %f", first_est_x, first_est_y);
              
              
              VectorXd robot_j_first_estimate(4) ;
              ROS_INFO("Created Kalman filter object");
              robot_j_first_estimate << first_est_x, first_est_y, ekf_velocity_x, ekf_velocity_y; // x,y,vx,vy - constant velocity model
              ROS_INFO("Initializaing EKF Track");
              wsr_state_estimation::ExtendedKalmanFilter new_robot_state_estimation_track (robot_j_first_estimate, measurement_interval__); //Run prediction every second
              ekf_robot_track__.insert({name_vicon_hardware[itr].c_str(),new_robot_state_estimation_track});
              est_x_j = robot_j_first_estimate[0];
              est_y_j = robot_j_first_estimate[1];
              ROS_INFO("First estimate = %f, %f", est_x_j, est_y_j);

              ekf_robot_track__[name_vicon_hardware[itr].c_str()].predict(); //Prediction comes from model
             
            }
            else
            {                            
              //Perform state_estimation of robot j
              ROS_INFO("Found Robot j track");
              VectorXd z(2);
              //Predict for next timestep
              ekf_robot_track__[name_vicon_hardware[itr].c_str()].predict();
              
              z << measurement_output__[0] , measurement_output__[1];
              ekf_robot_track__[name_vicon_hardware[itr].c_str()].update(z,robot_i_positions); //Correct the prediction based on the new measurement
                
              est_x_j = ekf_robot_track__[name_vicon_hardware[itr].c_str()].x[0];
              est_y_j = ekf_robot_track__[name_vicon_hardware[itr].c_str()].x[1];

              //Get covariance
              cov_array[0] = ekf_robot_track__[name_vicon_hardware[itr].c_str()].P(0,0); //cov_x
              cov_array[1] = ekf_robot_track__[name_vicon_hardware[itr].c_str()].P(0,1); //cov_xy
              cov_array[2] = ekf_robot_track__[name_vicon_hardware[itr].c_str()].P(1,0); //cov_yx
              cov_array[3] = ekf_robot_track__[name_vicon_hardware[itr].c_str()].P(1,1); //cov_y

              ROS_INFO("True position = %f, %f", pose_vec[itr].position.x, pose_vec[itr].position.y);
              ROS_INFO("Predicted estimate = %f, %f", est_x_j, est_y_j);
            }
            
            //Keep track of the latest position esimates for using in beta parameter of the information gain
            geometry_msgs::Point temp;
            temp.x = est_x_j;
            temp.y = est_y_j;
            current_rel_positions__.push_back(temp);

            unsigned int sizeX = costmap2d->getSizeInCellsX();
            unsigned int sizeY = costmap2d->getSizeInCellsY();
            ROS_INFO("**** getSizeInCellsX, getSizeInCellsY: %d, %d **** ", sizeX, sizeY);
            
            //Initialize node with true relative position of the other robot
            costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
            quadmap::Node position_node(mx__, my__, robot_information__[name_vicon_hardware[itr].c_str()].robot_tau, robot_information__[name_vicon_hardware[itr].c_str()].robot_id,timestep__);

            //Add estimated position
            costmap2d->worldToMap(est_x_j, est_y_j, mx__, my__);
            position_node.add_position_noise(mx__, my__);            
            
            //Only insert in quadmap if within these bounds
            
            
            if(mx__ > x_env_map_max_limit__ || my__ > y_env_map_max_limit__  || mx__ < x_env_map_min_limit__ || my__ < y_env_map_min_limit__) 
            {
              ROS_INFO("Predicted estimate (map) beyond boundary= %d, %d", mx__, my__);
            }
            else
            {
              ROS_INFO("Predicted estimate (map)= %d, %d", mx__, my__);

              position_node.updateOmega(cov_array[0], cov_array[3]); //Update the term based on the covariance
              robot_information__[name_vicon_hardware[itr]].node_information.push(position_node);
              base_quadmap_.insert_till_end(position_node); 
              
              neighboring_robot.true_map_position.x = position_node.true_mx;
              neighboring_robot.true_map_position.y = position_node.true_my; 
              neighboring_robot.estimated_map_position.x = position_node.est_mx;
              neighboring_robot.estimated_map_position.y = position_node.est_my;

              one_shot_position_x = robot_i_positions.position.x + measurement_output__[0]*cos(measurement_output__[1]);
              one_shot_position_y = robot_i_positions.position.y + measurement_output__[0]*sin(measurement_output__[1]);
              costmap2d->worldToMap(one_shot_position_x, one_shot_position_y, mx__, my__);
              neighboring_robot.one_shot_map_position.x = mx__;
              neighboring_robot.one_shot_map_position.y = my__;

              //Publisher message
              ROS_INFO("Adding other neighbor info");       
              neighboring_robot.true_range_bearing = measurement_output__;
              neighboring_robot.est_range_bearing = measurement_output__;
              neighboring_robot.filter_predicted_range_bearing = ekf_robot_track__[name_vicon_hardware[itr].c_str()].range_bearing__;
              neighboring_robot.filter_residual_error_range_bearing = ekf_robot_track__[name_vicon_hardware[itr].c_str()].residual_error__;
              neighboring_robot.error_meters = sqrt(pow((pose_vec[itr].position.x-est_x_j),2) + pow((pose_vec[itr].position.y-est_y_j),2));

              neighboring_robot.covariance_meter_sq = cov_array;
              neighboring_robot.status = 1;
              neighboring_robot.robot_id = position_node.getRobotID();
              msg.other_robots.push_back(neighboring_robot);
              ROS_INFO("Added neighbor info");
            }

          }
        // }
      }

      //Get frontiers centroids.
      frontiers_copy = frontier_temp__;
      // if(frontier_temp__.size()>0) 
      for(auto frontier_val : frontiers_copy)
      {
        // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
        wsr_exploration::FrontierInfo fc_point;
        costmap2d->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
        fc_point.centroid.x = fmx__;
        fc_point.centroid.y = fmy__;
        fc_point.size=frontier_val.size;
        fc_point.information_gain=frontier_val.information_gain;
        fc_point.centroid_distance=frontier_val.centroid_distance;
        fc_point.utility=frontier_val.cost;
        fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
        msg.frontiers.push_back(fc_point);
      }

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);

      start_val = std::chrono::high_resolution_clock::now();
    }
    stop_val = std::chrono::high_resolution_clock::now();
  }


  void Explore::AllOnboardSensingCallbackFilter(const explore_lite::RangeBearing::ConstPtr& input_msg)
  {
      ROS_INFO("=== Entered AllOnboardSensingCallbackFilter ===");

      std::vector<frontier_exploration::Frontier> frontiers_copy;
      wsr_exploration::QuadmapViz msg;
      costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
      std::vector<double> covariance_array = {0, 0, 0, 0};
      wsr_exploration::RelativeEstimate neighboring_robot;             
      double est_x_j=0,est_y_j=0, one_shot_position_x, one_shot_position_y;
      
      timestep__++;
      current_rel_positions__.clear();


      // Get current own pose of the robot. This is mostly for debugging and can later be removed if deemed unnecessary.
      auto current_pose = costmap_client_.getRobotPose();
      tf::Quaternion current_quaternion(
          current_pose.orientation.x,
          current_pose.orientation.y,
          current_pose.orientation.z,
          current_pose.orientation.w
      );
      double current_heading_deg = wrap0to360(quaternionToYaw(current_quaternion) * 180.0 / M_PI);
      std::vector<double> own_current_pose_vec = {
          current_pose.position.x, current_pose.position.y, current_heading_deg
      };
      ROS_INFO("Current pose (x, y, heading): %.2f, %.2f, %.2f",
              current_pose.position.x, current_pose.position.y, current_heading_deg);

      /* Retrieve pose history to find pose closest to the time when the first CSI sample was take.
      * We do this beacuse there is a 7 second gap between get raw data at some position and generating the measurement 
      * by which time the robot i would have moved/rotated.
      */
      own_pose_mutex.lock();
      std::vector<std::pair<double,geometry_msgs::Pose>> own_pose_history = {own_pose_deque_.begin(), own_pose_deque_.end()};
      own_pose_mutex.unlock();
      auto [matched_timestamp, closes_robot_pose_at_csi_measurement] = findClosestPoseToFirstSample(input_msg->csi_timestamp, own_pose_history);
      tf::Quaternion measurement_quaternion(
          closest_robot_pose_at_csi_measurement.orientation.x,
          closest_robot_pose_at_csi_measurement.orientation.y,
          closest_robot_pose_at_csi_measurement.orientation.z,
          closest_robot_pose_at_csi_measurement.orientation.w
      );
      own_orientation_deg__ = wrap0to360(quaternionToYaw(measurement_quaternion) * 180.0 / M_PI);
      current_angle__ = input_msg->bearing_measurements[0];
      if(__FLAG_first_measurement)
      {
        _previous_range_measurement = input_msg->range_measurements[0];
        __FLAG_first_measurement = false;
      }

      /*// === This block of code is deprecated; the functionality should be handled by the PDAF filter ===. 
      //Find the closest angle to previous angles among top peaks
      if(__FLAG_first_measurement)
      {
        previous_angle__ = wrap0to360(input_msg->bearing_measurements[0]);
        _previous_range_measurement = input_msg->range_measurements[0];
        current_angle__ = previous_angle__;
        __FLAG_first_measurement = false;
      }
      else
      {
        //Pick the closest peak in AOA top peaks by comparing with previous AOA angle
        min_diff__ = 1000;
        for(int angle_vals = 0; angle_vals<input_msg->bearing_measurements.size(); angle_vals++)
        {
          diff__ = abs(previous_angle__ - wrap0to360(input_msg->bearing_measurements[angle_vals]));
          if(diff__ < min_diff__)
          {
          min_diff__ = diff__;
          current_angle__ = wrap0to360(input_msg->bearing_measurements[angle_vals]);
          }
        }
        previous_angle__ = current_angle__;
      }
      */

      bearing_angle_radians__ = warptoPi(
          wrap0to360(own_orientation_deg__ + current_angle__) * M_PI / 180.0
      );
      
      //Workaround for issues in range measurements
      __range_to_use = input_msg->range_measurements[0];
      __bearing_to_use = bearing_angle_radians__;
      
      if(input_msg->range_measurements[0] == 0.0 || abs(_previous_range_measurement-input_msg->range_measurements[0]) > 2.0)
      {
      __range_to_use = _previous_range_measurement;
      }
      else
      {
        _previous_range_measurement = __range_to_use;
      }
      
      ROS_INFO("Measurement pose (x, y, heading): %.2f, %.2f, %.2f",
              closest_robot_pose_at_csi_measurement.position.x,
              closest_robot_pose_at_csi_measurement.position.y,
              own_orientation_deg__);
      ROS_INFO("Range: %.2f, Raw AOA: %.2f, Adjusted AOA: %.2f",
              __range_to_use,
              current_angle__,
              __bearing_to_use * 180.0 / M_PI);
      
      std::vector<double> own_measurement_pose_vec = {
          closest_robot_pose_at_csi_measurement.position.x,
          closest_robot_pose_at_csi_measurement.position.y,
          own_orientation_deg__
      };
      
      //=========== Add own position estimate from SLAM ==============
      costmap->worldToMap(closest_robot_pose_at_csi_measurement.position.x, closest_robot_pose_at_csi_measurement.position.y, mx__, my__);
      msg.own_position.x = mx__;
      msg.own_position.y = my__;
      msg.robot_id = robot_id_;
      ROS_INFO("Own position (world): %.2f, %.2f (ID: %d)",
              closest_robot_pose_at_csi_measurement.position.x,
              closest_robot_pose_at_csi_measurement.position.y,
              msg.robot_id);
      ROS_INFO("Own position (map): %d, %d", mx__, my__);
      
      //Add own's position to enable estimating of quadmap fill and terminating.
      quadmap::Node my_position_node(mx__, my__, my_tau__, robot_id_,timestep__);
      base_quadmap_.insert_till_end(my_position_node);
      

      //=========== Estimate neighboring robot position ==============
      //Estimate the relative position of the neighboring robot
      std::string other_robot_name = name_vicon_hardware[other_robot_id__ - 1];
      auto robot_it = robot_information__.find(other_robot_name);
      double est_x_j = 0.0, est_y_j = 0.0;
      wsr_exploration::RelativeEstimate neighbor;

      if( robot_it == robot_information__.end())
      {
          ROS_INFO("Creating track for Robot ID %d", other_robot_id__);

          double initial_x = closest_robot_pose_at_csi_measurement.position.x + __range_to_use * std::cos(__bearing_to_use);
          double initial_y = closest_robot_pose_at_csi_measurement.position.y + __range_to_use * std::sin(__bearing_to_use);

          VectorXd init_state(4);
          init_state << initial_x, initial_y, ekf_velocity_x, ekf_velocity_y;

          wsr_state_estimation::ExtendedKalmanFilter new_ekf(init_state, measurement_interval__);
          ekf_robot_track__.insert({other_robot_name, new_ekf});

          quadmap::Robot new_robot;
          new_robot.robot_id = other_robot_id__;
          robot_information__[other_robot_name] = new_robot;

          est_x_j = initial_x;
          est_y_j = initial_y;
          ROS_INFO("First estimate = %f, %f", est_x_j, est_y_j);

          ekf_robot_track__[other_robot_name].predict();
          VectorXd z(2); z << __range_to_use, __bearing_to_use;
          ekf_robot_track__[other_robot_name].update(z, closest_robot_pose_at_csi_measurement);

          prev_neighboring_position.position.x = est_x_j;
          prev_neighboring_position.position.y = est_y_j;
        
      } else {                            
        //Perform state_estimation of robot j
        ROS_INFO("Found Robot (robot id : %d) track", other_robot_id__);
        VectorXd z(2); z << __range_to_use, __bearing_to_use;
        ekf_robot_track__[other_robot_name].predict();
        ekf_robot_track__[other_robot_name].update(z, closest_robot_pose_at_csi_measurement); //TODO
        est_x_j = ekf_robot_track__[other_robot_name].x[0];
        est_y_j = ekf_robot_track__[other_robot_name].x[1];
        const auto& P = ekf_robot_track__[other_robot_name].P;
        covariance_array = { P(0, 0), P(0, 1), P(1, 0), P(1, 1) };
        ROS_INFO("Predicted estimate = %f, %f", est_x_j, est_y_j);
      }
      
      //Keep track of the latest position esimates for using in beta parameter of the information gain
      geometry_msgs::Point estimated_j_position;
      estimated_j_position.x = est_x_j;
      estimated_j_position.y = est_y_j;
      current_rel_positions__.push_back(estimated_j_position);

      //Initialize node with estimated position of the other robot
      costmap->worldToMap(est_x_j, est_y_j, mx__, my__);   
      unsigned int sizeX = costmap->getSizeInCellsX();
      unsigned int sizeY = costmap->getSizeInCellsY();
      ROS_INFO("**** getSizeInCellsX, getSizeInCellsY: %d, %d **** ", sizeX, sizeY);
      bool out_of_bounds = (
          mx__ > x_env_map_max_limit__ || my__ > y_env_map_max_limit__ ||
          mx__ < x_env_map_min_limit__ || my__ < y_env_map_min_limit__
      );

      if(out_of_bounds) 
      {
          ROS_INFO("Estimate out of bounds: (%d, %d)", mx__, my__);
          neighbor.true_map_position.x = prev_neighboring_position.position.x;
          neighbor.true_map_position.y = prev_neighboring_position.position.y;
          neighbor.estimated_map_position.x = prev_neighboring_position.position.x;
          neighbor.estimated_map_position.y = prev_neighboring_position.position.x;
      }
      else
      {
          ROS_INFO("Estimate in map: (%d, %d)", mx__, my__);

          auto& robot_data = robot_information__[other_robot_name];
          quadmap::Node j_position_node(mx__, my__, robot_data.robot_tau, robot_data.robot_id, timestep__);
          other_node.updateOmega(covariance_array[0], covariance_array[3]);
          robot_data.node_information.push(j_position_node);
          base_quadmap_.insert_till_end(j_position_node);

          neighbor.true_map_position.x = j_position_node.true_mx;
          neighbor.true_map_position.y = j_position_node.true_my;
          neighbor.estimated_map_position.x = j_position_node.est_mx;
          neighbor.estimated_map_position.y = j_position_node.est_my;
      }

      // Final message population
      neighbor.one_shot_map_position.x = mx__;
      neighbor.one_shot_map_position.y = my__;
      neighbor.est_range_bearing = { __range_to_use, current_angle__, __bearing_to_use * 180.0 / M_PI };
      neighbor.first_csi_measurement_timestamp = input_msg->csi_timestamp;
      neighbor.nearest_timestamp_for_pose = matched_timestamp;
      neighbor.covariance_meter_sq = covariance_array;
      neighbor.status = 1;
      neighbor.robot_id = other_robot_id__;
      neighbor.filter_predicted_range_bearing = ekf_robot_track__[other_robot_name].range_bearing__;
      neighbor.filter_residual_error_range_bearing = ekf_robot_track__[other_robot_name].residual_error__;

      msg.own_current_pose = own_current_pose_vec;
      msg.own_measurement_pose = own_measurement_pose_vec;
      msg.other_robots.push_back(neighbor);

      //Get frontiers centroids. //Commeted out for Debugging
      // frontiers_copy = frontier_temp__;
      // // if(frontier_temp__.size()>0) 
      // for(auto frontier_val : frontiers_copy)
      // {
      //   // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
      //   wsr_exploration::FrontierInfo fc_point;
      //   costmap->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
      //   fc_point.centroid.x = fmx__;
      //   fc_point.centroid.y = fmy__;
      //   fc_point.size=frontier_val.size;
      //   fc_point.information_gain=frontier_val.information_gain;
      //   fc_point.centroid_distance=frontier_val.centroid_distance;
      //   fc_point.utility=frontier_val.cost;
      //   fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
      //   msg.frontiers.push_back(fc_point);
      // }

      // === Final publish ===
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);
    }

/**
 * For vicon hardware experiments with two robots.
 * Robot ids are number starting 1
*/
void Explore::ViconCombinedStateCallbackTruePositionBaseline(const geometry_msgs::PoseArray::ConstPtr& input_msg)
{
    // std::vector<std::string> name = input_msg->name;
    std::vector<geometry_msgs::Pose> pose_vec = input_msg->poses;
    geometry_msgs::Pose robot_i_positions, robot_i_vicon_position;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap(); 
    double world_x, world_y;
    std::vector<frontier_exploration::Frontier> frontiers_copy;
    std::vector<double> cov_array{0,0,0,0};
    
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    if(duration.count() > measurement_interval__) // publish every 10 seconds
    {
      timestep__+=1;
      wsr_exploration::QuadmapViz msg;
      current_rel_positions__.clear();
      
      //First find the current position of the robot i
      for(int itr=0; itr<pose_vec.size(); itr++)
      {
        if(itr==robot_id_-1)
        {
          ROS_INFO("itr: %d", itr);
          ROS_INFO("robot_id_: %d", robot_id_);
          
          //For hardware implementation using global frame, we need to start at 0,0.
          //For this initialize the (0,0) origin of vicon reference frame close to the robot's starting point 
          robot_i_positions = pose_vec[itr]; 
          costmap2d->worldToMap(robot_i_positions.position.x, robot_i_positions.position.y, mx__, my__);
          msg.own_position.x = mx__;
          msg.own_position.y = my__;
          msg.robot_id = robot_id_;
          ROS_INFO("Own position of robot(ID) in world = %f, %f, %d", robot_i_vicon_position.position.x,robot_i_vicon_position.position.y, msg.robot_id);
          ROS_INFO("Own position of robot(ID) in map = %d, %d, %d", mx__,my__, msg.robot_id);


          //Add own's position to enable estimating of quadmap fill and terminating.
          quadmap::Node position_node(mx__, my__, my_tau__, robot_id_,timestep__);
          base_quadmap_.insert_till_end(position_node); 
          break;
        }
      }
      
      for(int itr=0; itr<pose_vec.size(); itr++)
      {
          wsr_exploration::RelativeEstimate neighboring_robot;             
          double est_x_j=0,est_y_j=0, one_shot_position_x, one_shot_position_y;
        
          if(itr!=robot_id_-1)
          {
            //Use the vicon mocap positions of the neighboring robot to generate range and bearing measurements
            measurement_output__.clear();

            //Initialize node with true relative position of the other robot
            costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
            quadmap::Node position_node(mx__, my__, robot_information__[name_vicon_hardware[itr].c_str()].robot_tau, robot_information__[name_vicon_hardware[itr].c_str()].robot_id,timestep__);
            
            if(mx__ > x_env_map_max_limit__ || my__ > y_env_map_max_limit__  || mx__ < x_env_map_min_limit__ || my__ < y_env_map_min_limit__) 
            {
              ROS_INFO("Predicted estimate (map) beyond boundary= %d, %d", mx__, my__);
            }
            else
            {
              ROS_INFO("Predicted estimate (map)= %d, %d", mx__, my__);
              neighboring_robot.true_map_position.x = position_node.true_mx;
              neighboring_robot.true_map_position.y = position_node.true_my; 
              neighboring_robot.estimated_map_position.x = position_node.est_mx;
              neighboring_robot.estimated_map_position.y = position_node.est_my;

              ROS_INFO("Added neighbor info for visualization only");
              //Publisher message      
              neighboring_robot.true_range_bearing = measurement_output__;
              neighboring_robot.est_range_bearing = measurement_output__;
              neighboring_robot.filter_predicted_range_bearing = ekf_robot_track__[name_vicon_hardware[itr].c_str()].range_bearing__;
              neighboring_robot.filter_residual_error_range_bearing = ekf_robot_track__[name_vicon_hardware[itr].c_str()].residual_error__;
              neighboring_robot.error_meters = sqrt(pow((pose_vec[itr].position.x-est_x_j),2) + pow((pose_vec[itr].position.y-est_y_j),2));

              neighboring_robot.covariance_meter_sq = cov_array;
              neighboring_robot.status = 1;
              neighboring_robot.robot_id = position_node.getRobotID();
              msg.other_robots.push_back(neighboring_robot);
             
            }
          }
      }

      //Get frontiers centroids.
      frontiers_copy = frontier_temp__;
      // if(frontier_temp__.size()>0) 
      for(auto frontier_val : frontiers_copy)
      {
        // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
        wsr_exploration::FrontierInfo fc_point;
        costmap2d->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
        fc_point.centroid.x = fmx__;
        fc_point.centroid.y = fmy__;
        fc_point.size=frontier_val.size;
        fc_point.information_gain=frontier_val.information_gain;
        fc_point.centroid_distance=frontier_val.centroid_distance;
        fc_point.utility=frontier_val.cost;
        fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
        msg.frontiers.push_back(fc_point);
      }

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);

      start_val = std::chrono::high_resolution_clock::now();
    }
    stop_val = std::chrono::high_resolution_clock::now();
  }


void Explore::modelStateCallbackTruePositionForBaseline(const gazebo_msgs::ModelStates::ConstPtr& input_msg)
{
    std::vector<std::string> name = input_msg->name;
    std::vector<geometry_msgs::Pose> pose_vec = input_msg->pose;
    geometry_msgs::Pose robot_i_positions;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
    double world_x, world_y;
    std::vector<frontier_exploration::Frontier> frontiers_copy;
    std::vector<double> cov_array{0,0};
    
    duration = std::chrono::duration_cast<std::chrono::seconds>(stop_val - start_val);
    if(duration.count() > measurement_interval__) // publish every 10 seconds
    {
      timestep__+=1;
      wsr_exploration::QuadmapViz msg;
      
      //First find the current position of the robot i
      for(int itr=0; itr<name.size(); itr++)
      {
        if(name[itr]==robot_name_)
        {
          robot_i_positions = pose_vec[itr];
          robot_id_ = itr; //Not that the robot id will not change during an instance of simulation
          costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
          msg.own_position.x = mx__;
          msg.own_position.y = my__;
          msg.robot_id = itr;
          ROS_INFO("Own position of robot(ID) = %u, %u, %d", mx__,my__, msg.robot_id);
          break;
        }
      }
      
      for(int itr=0; itr<name.size(); itr++)
      {
        if(IsMatch(name[itr]))
        {        
          if(name[itr]!=robot_name_) //Only do this for robot j in the neighborhood of robot i
          {
            wsr_exploration::RelativeEstimate neighboring_robot;
            unsigned int sizeX = costmap2d->getSizeInCellsX();
            unsigned int sizeY = costmap2d->getSizeInCellsY();
            ROS_INFO("**** getSizeInCellsX, getSizeInCellsY: %d, %d **** ", sizeX, sizeY);
            
            //Add true position
            costmap2d->worldToMap(pose_vec[itr].position.x, pose_vec[itr].position.y, mx__, my__);
            quadmap::Node position_node(mx__, my__, robot_information__[name[itr].c_str()].robot_tau, robot_information__[name[itr].c_str()].robot_id,timestep__);

            //Publisher message
            ROS_INFO("Adding neighbor info");
            neighboring_robot.true_map_position.x = position_node.true_mx;
            neighboring_robot.true_map_position.y = position_node.true_my;        
            neighboring_robot.estimated_map_position.x = position_node.est_mx;
            neighboring_robot.estimated_map_position.y = position_node.est_my;

            neighboring_robot.covariance_meter_sq = cov_array;
            neighboring_robot.status = 1;
            neighboring_robot.robot_id = position_node.getRobotID();
            msg.other_robots.push_back(neighboring_robot);
            ROS_INFO("Added neighbor info");
          }
        }
      }

      //Get frontiers centroids.
      frontiers_copy = frontier_temp__;
      // if(frontier_temp__.size()>0) 
      for(auto frontier_val : frontiers_copy)
      {
        // frontier_exploration::Frontier frontier_val = frontier_temp__[0];
        wsr_exploration::FrontierInfo fc_point;
        costmap2d->worldToMap(frontier_val.centroid.x, frontier_val.centroid.y, fmx__, fmy__);
        fc_point.centroid.x = fmx__;
        fc_point.centroid.y = fmy__;
        fc_point.size=frontier_val.size;
        fc_point.information_gain=frontier_val.information_gain;
        fc_point.centroid_distance=frontier_val.centroid_distance;
        fc_point.utility=frontier_val.cost;
        fc_point.neighboring_robot_position_count=frontier_val.neighbors_count;
        msg.frontiers.push_back(fc_point);
      }

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(frame__++);
      quadmapPub_.publish(msg);

      start_val = std::chrono::high_resolution_clock::now();
    }
    
    stop_val = std::chrono::high_resolution_clock::now();

  }


  /** 
   * @brief Get positions of neighboring robots in for hardware experiments in motion capture lab
   * */
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
        current_neighbor_pose_vec_.clear();
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
          current_neighbor_pose_vec_.push_back(temp);
          
          for (int itr=0; itr<current_neighbor_pose_vec_.size(); itr++)
          {   
            ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, current_neighbor_pose_vec_[itr].x, current_neighbor_pose_vec_[itr].y);
          }
        }
      }
      FLAG_GET_POS = false;
    }
  }


  /** 
   * @brief Callback for setting a failed robot tau status 
   * */
  void Explore::setFailedRobotStatus(const std_msgs::String::ConstPtr& msg)
  {
    if(msg->data == robot_name_) 
    {
      exploration_done_ = true;
    }
    else
    {
      auto search_val = robot_information__.find(msg->data);
      search_val->second.robot_tau = 0;
      ROS_INFO("Setting neighboring robot %s tau to zero", search_val->first.c_str());
    }
  }


  /** 
   * @brief Callback to set flag for stopping own exploration
   * */
  void Explore::explorationStatusCB(const std_msgs::Bool::ConstPtr& msg)
  {
    exploration_done_ = msg->data;
  }


 /**
  *@brief Robot collects own pose when CSI data is being collected
  * */
  void Explore::CollectOwnPoseCB(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
      //Get filename of the csi data file
      ROS_INFO(" ========= Deque size : %u ======== ", own_pose_deque_.size());
      std::time_t current_epoch_time;
      std::string orig_output = exec(servo_output_file_reader_command_.c_str());
      std::string new_output = orig_output;

      while(new_output == orig_output){
        auto current_pose = costmap_client_.getRobotPose();
        const auto now = std::chrono::system_clock::now();
        current_epoch_time = std::chrono::system_clock::to_time_t(now); 
        if(own_pose_deque_.size() > 200) own_pose_deque_.pop_front();
        own_pose_deque_.push_back(std::make_pair(current_epoch_time, current_pose));
	      ros::Duration(0.1).sleep();
        new_output = exec(command.c_str());
      }

      for(auto& elem:  own_pose_deque_) {
	      ROS_INFO("%f", elem.first);
      }
    }
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
    private_nh_.param("robot_name", robot_name_, std::string("tb3_1"));
    private_nh_.param("robot_id", robot_id_, 0);
    private_nh_.param("neighbor_name", neighbor_name_, std::string("tb3_"));
    private_nh_.param("potential_scale", potential_scale_, 1e-3);
    private_nh_.param("orientation_scale", orientation_scale_, 0.0);
    private_nh_.param("gain_scale", gain_scale_, 1.0);
    private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
    private_nh_.param("sensor_range", sensor_range_, 1.0); 
    private_nh_.param("use_WSR", FLAG_WSR_, true);
    private_nh_.param("noise_WSR", FLAG_noise, false);
    private_nh_.param("WSR_noise_mean", noise_mean_, 0.0);
    private_nh_.param("WSR_noise_std", noise_std_, 1.0); 
    private_nh_.param("WSR_utility_alpha_parameter", utility_alpha_parameter_, 1.0); 
    private_nh_.param("WSR_utility_beta_parameter", utility_beta_parameter_, 1.0); 
    private_nh_.param("Quadmap_width", Quadmap_width_, 20.0); 
    private_nh_.param("Quadmap_height", Quadmap_height, 20.0);
    private_nh_.param("ekf_predict_interval", measurement_interval__, 1.0);
    private_nh_.param("quadmap_fill_percentage", fill_percentage_threshold__, 90.0); 
    private_nh_.param("map_resolution", map_resolution__, 0.15);
    private_nh_.param("diff_between_termination_thresholds", diff_between_termination_thresholds__, 5); 
    private_nh_.param("use_sim", FLAG_SIM_, false);
    private_nh_.param("robot_speed", robot_speed_, 0.15);  // Used to compute progress timeout and force reevaluation of frontiers
    
 
    //Subscribers
    // modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
    
    if(FLAG_WSR_)
    {
      if(FLAG_SIM_)
      {
      //For gazebo simulation
      modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallbackFilter, this);
      // modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallback, this);
      }
      else
      {
        //For all onboard sensing
        modelStateSub_ =  private_nh_.subscribe<explore_lite::RangeBearing>("/"+robot_name_+"/range_bearing_estimates", 10, &Explore::AllOnboardSensingCallbackFilter, this);
        saveRobotPose_ =  private_nh_.subscribe<std_msgs::Bool>("/"+robot_name_+"/wsr_antenna_motor/start_motion", 10, &Explore::CollectOwnPoseCB, this);

        //For vicon hardware experiments.
        // modelStateSub_ = private_nh_.subscribe<geometry_msgs::PoseArray> ("/vicon_state_topic", 10, &Explore::ViconCombinedStateCallbackFilter, this);
      
      }
    }
    else
    {
      if(FLAG_SIM_)
      {
        //For gazebo simulation
        modelStateSub_ = private_nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 10, &Explore::modelStateCallbackTruePositionForBaseline, this);
      }
      else
      {
        //For vicon hardware experiments.
        modelStateSub_ = private_nh_.subscribe<geometry_msgs::PoseArray> ("/vicon_state_topic", 10, &Explore::ViconCombinedStateCallbackTruePositionBaseline, this);
      }
      
    }

    move_bas_path_client__ = private_nh_.serviceClient<nav_msgs::GetPlan>("/"+robot_name_+"/move_base_node/make_plan");
    optitrackSub_ = private_nh_.subscribe<natnet_pkg::PoseArrayID> ("/optitrack_pose", 10, &Explore::optitrackMocapCB, this);
    exploration_ = private_nh_.subscribe<std_msgs::Bool> ("/true_exploration_status", 10, &Explore::explorationStatusCB, this);
    setFailedRobotTau_ = private_nh_.subscribe<std_msgs::String> ("/set_failed_neighboring_robot", 10, &Explore::setFailedRobotStatus, this);
    exploration_eval_stop_ = private_nh_.advertise<std_msgs::Bool> ("/"+robot_name_+"/stop_evaluation", 10);
    quadmapPub_ =  private_nh_.advertise<wsr_exploration::QuadmapViz>("node_list", 10);
    struct passwd *pw = getpwuid(getuid());
    homedir_ = pw->pw_dir;
    servo_output_file_reader_command_ = "stat"+homedir_+"/catkin_ws/src/react-m_explore/explore/data/motorjoint_displacement_final.csv | grep Change | awk ' {print $3} '";  
    __dim_object_name = "mailbox_blue_clone"; //Used in simulation

    ROS_INFO("Sensor range = %f", sensor_range_);
    ROS_INFO("min_frontier_size = %f", min_frontier_size);  
    
    //Since we insert in map coordinates
    float width = Quadmap_width_/map_resolution__;
    float height = Quadmap_height/map_resolution__;

    //TODO : Check this - map coordinates, map_resolution,
    
    auto domain = quadmap::Rect(float(width)/2, float(height)/2, float(width), float(height));
    base_quadmap_ = quadmap::QuadMap(domain, sensor_range_, map_resolution__);
    cell_count__ = base_quadmap_.total_cells;
    
    /**
     * This change has been made for running hardware expperiments in the flight lab
    */
    if(!FLAG_SIM_)
    {
      cell_count__ = 16;
      ekf_velocity_x = 0.1;
      ekf_velocity_y = 0.1;
      baseline_1_frontier_selection_threshold__ = 80; //Since our hardware environment is small and not many forntiers are generated
      other_robot_id__ = robot_id_ == 1 ? 2:1;

      x_env_map_max_limit__ = 40; 
      y_env_map_max_limit__ = 40;
      x_env_map_min_limit__ = 4;
      y_env_map_min_limit__ = 4;
    }
    //*****************************************************************

    if (visualize_) {
      marker_array_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
    }

    search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                  potential_scale_, gain_scale_,
                                                  min_frontier_size, sensor_range_,
                                                  utility_alpha_parameter_, utility_beta_parameter_);
    
    ROS_INFO("Waiting to connect to move_base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");

    last_progress_ = ros::Time::now();
    exploring_timer_ =
        relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                [this](const ros::TimerEvent&) { makePlan();});
  }

  /** 
   * @brief Destructor
   * */
  Explore::~Explore()
  {
    stop();
  }

  /** 
   * @brief Add markers for visualization of frontiers in rviz
   * */
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
   * @brief Exploration Planner
   * */
  void Explore::makePlan()
  {
    // find frontiers
    auto pose = costmap_client_.getRobotPose();

    if(!__Flag_set_home)
    {
      __home_position.x = pose.position.x;
      __home_position.y = pose.position.y;
      __Flag_set_home = true;
    }
    // ROS_INFO("Neighbors count = %d", int(current_neighbor_pose_vec_.size()));
    
    // for (int itr=0; itr<current_neighbor_pose_vec_.size(); itr++)
    // {
    //   ROS_DEBUG("neighbor: %d pos: (%f, %f )", itr, current_neighbor_pose_vec_[itr].x, current_neighbor_pose_vec_[itr].y);
    // }

    unsigned fmx, fmy, mx__, my__;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

    // get frontiers sorted according to cost
    frontiers__.clear();
    std::vector<frontier_exploration::Frontier> final_sorted_frontiers;
    frontiers__ = search_.searchFrontiers(pose.position,
                                          x_env_map_max_limit__, 
                                          y_env_map_max_limit__,
                                          x_env_map_min_limit__,
                                          y_env_map_min_limit__);

    
    ROS_DEBUG("found %lu frontiers", frontiers__.size());


    // for(int i=0; i<frontiers__.size(); i++)
    // {
    //   frontier_exploration::Frontier frontier = frontiers__[i];
    //   costmap2d->worldToMap(frontier.centroid.x, frontier.centroid.y, fmx, fmy);
    //   unsigned int clear, frontier_pos  = costmap2d->getIndex(fmx,fmy);
      
    //   ROS_INFO("***************************************************");
    //   ROS_INFO("Frontier centroid Map pos: %d, %d", fmx, fmy);
      
    //   //Get relative positions around a frontier centroid by searching the quadmap and update the vector neighboring_robots_positions
    //   quadmap::Node center(fmx, fmy, robot_id_, robot_id_); //Value of the 3rd parameter is meaningless here for the query
    //   std::vector<quadmap::Node> neighboring_robots_positions;
    //   ROS_INFO("Quadmap ID : %d", base_quadmap_.get_quadmap_ID());
    //   auto op_val = base_quadmap_.query_radius(center,2*sensor_range_,neighboring_robots_positions);
    //   ROS_INFO("[Explore.cpp] query result %d", op_val);
    //   for(auto val : neighboring_robots_positions)
    //   {
    //     ROS_INFO("Node position %f, %f", val.true_mx, val.true_my);
    //   }
    //   ROS_INFO("[Explore.cpp] Query Check: Neighboring robot positions around the frontier = %ld", neighboring_robots_positions.size());
    // }
    // final_sorted_frontiers = frontiers;
    if(!FLAG_WSR_) ROS_INFO("!!!!!!!!!! FLAG_WSR is disabled !!!!!!!!!");

    
    // Reevaulate frontiers every progress_timeout_ seconds
    // std::cout << last_progress_ << std::endl;
    std::cout << ros::Time::now() - last_progress_ << std::endl;
    std::cout << progress_timeout_ << std::endl;
    if (ros::Time::now() - last_progress_ > progress_timeout_) 
    {
      move_base_client_.cancelAllGoals();
      last_progress_ = ros::Time::now();
      ROS_INFO("******* REVAULATING ALL FRONTIERS ******************");
    }


    // Reevaulate all frontiers once 50% progress has been made to the frontier to understand if its still worthwhile to 
    //go to that frontier.
    ros::Duration half_duration(progress_timeout_.toSec()*0.5);
    if (ros::Time::now() - last_progress_ > half_duration) 
    {
      move_base_client_.cancelAllGoals();
      ROS_INFO("******* REACHED halfway to goal ---  REVAULATING ALL FRONTIERS ******************");
    }

    
    //Need to update the frontier centroid distance based on the path length, should be in world coordinates
    actionlib::SimpleClientGoalState current_state  = move_base_client_.getState();
    if (current_state != actionlib::SimpleClientGoalState::ACTIVE)
    { 
      
      last_progress_ = ros::Time::now();
      std::cout << last_progress_ << std::endl;
      
      start__.pose = pose;
      for (size_t i = 0; i < frontiers__.size(); ++i) 
      {
        ROS_INFO("frontier %zd centroid distance before: %f meters", i, frontiers__[i].centroid_distance);
        
        goal__.pose.position.x = frontiers__[i].centroid.x;
        goal__.pose.position.y = frontiers__[i].centroid.y;
        goal__.pose.orientation.w = 0.0;

        if(GetPlanPath(start__, goal__, tolerance__, frontier_centroid_path__))
        {
          ROS_INFO("Path received with %ld poses", frontier_centroid_path__.poses.size());
          frontiers__[i].centroid_distance = calculatePathLength(frontier_centroid_path__);
        }
        else
        {
          //No reachable path, assign high cost
          frontiers__[i].centroid_distance = 1000;
        }
        ROS_INFO("frontier %zd centroid distance after: %f meters", i, frontiers__[i].centroid_distance);
        ROS_INFO("*****************************************************************************");
      }   
    
      final_sorted_frontiers = search_.getMaxUtilityFrontiers(frontiers__, base_quadmap_, 
                                                              robot_id_,FLAG_WSR_,__FLAG_can_stop_now__,
                                                              current_rel_positions__,
                                                              x_env_map_max_limit__, 
                                                              y_env_map_max_limit__,
                                                              x_env_map_min_limit__,
                                                              y_env_map_min_limit__); 
      frontier_temp__ = final_sorted_frontiers;

      
      ROS_INFO("===============Sorted frontiers===================");
      for (size_t i = 0; i < final_sorted_frontiers.size(); ++i) 
      {
        ROS_INFO("frontier %zd cost: %f", i, final_sorted_frontiers[i].cost);
        ROS_INFO("frontier %zd position: (%f, %f )", i, final_sorted_frontiers[i].centroid.x, final_sorted_frontiers[i].centroid.y);
      }

      //TODO: Update this to store utility without using the relative positions
      end_exploration = std::chrono::high_resolution_clock::now();
      std::chrono::seconds elapsed_time__ = std::chrono::duration_cast<std::chrono::seconds>(end_exploration - start_exploration);
      writeToFile(final_sorted_frontiers,fn,elapsed_time__); 
      
      
      // Stop if no more new frontiers exist or the stop flag is set.
      if (final_sorted_frontiers.empty() || exploration_done_) 
      {
        stop();
        end_exploration = std::chrono::high_resolution_clock::now();
        std::chrono::seconds elapsed_time__ = std::chrono::duration_cast<std::chrono::seconds>(end_exploration - start_exploration);
        writeToFile(final_sorted_frontiers,fn,elapsed_time__); 
        return;
      }

      // publish frontiers as visualization markers
      if (visualize_) 
      {
        visualizeFrontiers(final_sorted_frontiers);
      }

      
      //Randomly choose a frontier
      // std::random_device rd; // obtain a random number from hardware
      // std::mt19937 gen(rd()); // seed the generator
      // std::uniform_int_distribution<> distr(0, int(final_sorted_frontiers.size())-1); // define the range
      // int rval = distr(gen); // generate numbers
      
      //frontier_exploration::Frontier frontier;
      //if(final_sorted_frontiers.size() > 0) frontier = final_sorted_frontiers[rval];
      //else 
      //{
      //    stop();
      //    return;
      //}
      
      // time out if we are not making any progress
      //geometry_msgs::Point target_position = frontier.centroid;
      //bool same_goal = prev_goal_ == target_position;
      //prev_goal_ = target_position;
      //if (!same_goal || prev_distance_ > frontier.min_distance) 
      //{
      //  last_progress_ = ros::Time::now(); // we have different goal or we made some progress
      //  prev_distance_ = frontier.min_distance;
      //}
      //if (ros::Time::now() - last_progress_ > progress_timeout_) // black list if we've made no progress for a long time
      //{
      //  frontier_blacklist_.push_back(target_position);
      //  ROS_DEBUG("Adding current goal to black list");
      //  makePlan();
      //  return;
      //}


      // find non blacklisted frontier
      auto frontier = std::find_if_not(final_sorted_frontiers.begin(), final_sorted_frontiers.end(),
                          [this](const frontier_exploration::Frontier& f) {
                            return goalOnBlacklist(f.centroid);
                          });

      
      // //FR-1-B Baseline1-b : // Randomly choose a frontier from top 2
      // //FR-1-B Noisy-c : // Randomly choose a frontier from top 2 - Don't use
      if(!FLAG_WSR_)
      {
        //Only use for random motion baseline with close proximity initialization of robot positions.
        if(final_sorted_frontiers.size()>1)
        {
          std::random_device rd; // obtain a random number from hardware
          std::mt19937 gen(rd()); // seed the generator
          std::uniform_int_distribution<> distr(0, 99); // define the range
          int rval = distr(gen); // generate numbers

          //Choose the first frontier with x% probability and second one with 100-x%
          //Starting with 60% and then over time only the top frontier will be chosen.
          //Just need to ensure that the robots spread out more even after close initialization
          if (baseline_1_frontier_selection_threshold__ >= 97) baseline_1_frontier_selection_threshold__ = 97;
          
          if(rval <= baseline_1_frontier_selection_threshold__) 
            std::advance(frontier, 0);     
          else 
            std::advance(frontier, 1); 
          
          baseline_1_frontier_selection_threshold__ +=2;
        }
      }
      
      //Evaluate if its still worthwhile to go to that frontier midway
      //0.15 is the robot speed. 
      //Multiply by 0.90 to get the time to reach greater than 3/4th way to the frontier. Division by 3 is for hardware experiments sinec our distances are small
      try
      {
        // progress_timeout_ = ros::Duration(frontier->centroid_distance/(robot_speed_) * 0.90); //For simulation 
        progress_timeout_ = ros::Duration(frontier->centroid_distance/(robot_speed_/3)); //For hardware
      }
      catch(...)
      {
        progress_timeout_ = ros::Duration(3); 
      }
      
      
      // progress_start_time_ = ros::Time::now();
      if (frontier == final_sorted_frontiers.end()) 
      {
        //TODO add navigation to home position.
        stop();
        return;
      }
      
      
      geometry_msgs::Point target_position = frontier->centroid;
      
      /* === This is deprecated code from original repo, we do not use it anymore=====
      //timeout if we are not making any progress
      // geometry_msgs::Point target_position = frontier->view_point_to_navigate_to; @BUG -some weird waypoints.
      // bool same_goal = prev_goal_ == target_position;
      // prev_goal_ = target_position;
      // if (!same_goal || prev_distance_ > frontier->min_distance) 
      // {
      //   last_progress_ = ros::Time::now(); // we have different goal or we made some progress
      //   prev_distance_ = frontier->min_distance;
      // }
      
      // if (ros::Time::now() - last_progress_ > progress_timeout_) // black list if we've made no progress for a long time
      // {
      //   frontier_blacklist_.push_back(target_position);
      //   ROS_DEBUG("Adding current goal to black list");
      //   makePlan();
      //   return;
      // }

      // if (same_goal) 
      // {
      //   return;     // we don't need to do anything if we still pursuing the same goal
      // }
      // send goal to move_base if we have something new to pursue
      ========================================================================================*/

      /*  
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose.position = target_position;
      goal.target_pose.pose.orientation.w = 1.;
      goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
      goal.target_pose.header.stamp = ros::Time::now();
      move_base_client_.sendGoal(goal, [this, target_position]
                      ( const actionlib::SimpleClientGoalState& status,
                        const move_base_msgs::MoveBaseResultConstPtr& result) 
                       {
                        reachedGoal(status, result, target_position);
                     });

      */

    }

    //Get estimated fill percentage of the quadmap
    if(FLAG_WSR_)
    {
      filled_cell_count__ = 0;
      base_quadmap_.query_filled(filled_cell_count__);
      ROS_INFO("******* Total cells = %f", cell_count__);
      ROS_INFO("******* Filled cells = %f", filled_cell_count__);
      ROS_INFO("******* Estimated map fill percentage = %f", 100*filled_cell_count__/cell_count__);

      if(100*filled_cell_count__/cell_count__ >= fill_percentage_threshold__-diff_between_termination_thresholds__)
      { 
        __FLAG_can_stop_now__ = true;
        ROS_INFO("******* Exploration Termination condition satisfied - Soft Threshold ******************");

        if(!__FLAG_publish_once)
        {
          // std_msgs::Bool msg_val;
          // msg_val.data=true;
          // exploration_eval_stop_.publish(msg_val);
          __FLAG_publish_once = true;
          move_base_client_.cancelAllGoals(); // Immediate revaluate frontiers before proceeding
        }

      }

      if(100*filled_cell_count__/cell_count__ >= fill_percentage_threshold__)
      { 
        ROS_INFO("******* Hard Termination stop ******************");
        final_sorted_frontiers.clear();
        stop();
      }
    }

  
  }


  /** 
   * @brief Get the path from planner from current position to the frontier centroid
   * */
  bool Explore::GetPlanPath(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, float tolerance,
                nav_msgs::Path& plan) 
  {
    
    move_bas_path_client__.waitForExistence(); // Optional: Wait for the service to exist.

    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = tolerance;

    if (move_bas_path_client__.call(srv)) 
    {
      plan = srv.response.plan;
      return true;
    } 
    else 
    {
      ROS_ERROR("Failed to call service.");
      return false;
    }
  }


  /** 
   * @brief Blacklist frontiers that are not reachable
   * */
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
   * @brief Find the actual path length instead of the euclidean distance.
   * */
  double Explore::calculatePathLength(const nav_msgs::Path& path) 
  {
    double total_length = 0.0;

    for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto& pose1 = path.poses[i - 1].pose.position;
        const auto& pose2 = path.poses[i].pose.position;

        double dx = pose1.x - pose2.x;
        double dy = pose1.y - pose2.y;
        total_length += sqrt(dx*dx + dy*dy);
    }

    return total_length;
  }
  
  
  
  /** 
   * @brief Action of finding a new goal when the current goal is reached
   * */
  void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                            const move_base_msgs::MoveBaseResultConstPtr&,
                            const geometry_msgs::Point& frontier_goal)
  {
    ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());

    if (status == actionlib::SimpleClientGoalState::ABORTED) 
    {
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

  /** 
   * @brief Start exploration timer
   * */
  void Explore::start()
  {
    exploring_timer_.start();
  }

  /** 
   * @brief End exploration
   * */
  void Explore::stop()
  {
    move_base_client_.cancelAllGoals();
    exploring_timer_.stop();
    
    std_msgs::Bool msg_val;
    msg_val.data=true;

    for(int ii=0; ii<3; ii++)
      exploration_eval_stop_.publish(msg_val); //This is to also trigger stopping of collection of merged map for evaluation
    
    sleep(3); //make sure that the exploration evaluation stops

    exploration_completed_ = true;
    ROS_INFO("Exploration stopped.");
  }


  /** 
   * Save exploration stats to file
   * */
  void Explore::writeToFile(std::vector<frontier_exploration::Frontier>& wsr_frontiers,
                            std::string& fn,
                            std::chrono::seconds& elapsed_time__)
  {

    float dur = elapsed_time__.count();
    for(int i=0; i<wsr_frontiers.size(); i++)
    {
      std::vector<float> temp{float(wsr_frontiers[i].pos_id), float(wsr_frontiers[i].information_gain), 
                              float(wsr_frontiers[i].centroid_distance), float(wsr_frontiers[i].neighbors_count), float(i+1), dur};
      wsr_frontiers_stats_.push_back(temp);
    }

    // IF exploration ends, store all data into a file
    if(exploration_completed_)
    {
      ROS_INFO("Saving exploration termination stats to file.");
      std::cout.precision(10);
      const auto p1 = std::chrono::system_clock::now();
      std::string ts = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count());
      std::string fn1 = fn+"wsr_frontiers_stats_"+robot_name_+"_"+ts+".csv";
      std::ofstream myfile_def (fn1);
      std::vector<double> temp;
      std::vector<std::string> details {"pos_id", "info_gain", "centroid_distance" , "j_relative_position_count", "frontier_index", "Elapsed time(sec)"}; //index 1 means the top most frontier at each iteration which will then be selected

      if (myfile_def.is_open())
      {
          for(int j=0; j< details.size(); j++)
          {
              myfile_def << std::fixed << details[j] << ",";
          }
          myfile_def << "\n";


          for(size_t i = 0; i < wsr_frontiers_stats_.size(); i++)
          {
              for(int j=0; j< wsr_frontiers_stats_[i].size(); j++)
              {
                  myfile_def << std::fixed << wsr_frontiers_stats_[i][j] << ",";
              }
              myfile_def << "\n";
          }
          
      }
      myfile_def.close();
      
      std_msgs::Bool msg_val;
      msg_val.data=true;

      for(int ii=0; ii<3; ii++)
        exploration_eval_stop_.publish(msg_val); //This is to also trigger stopping of collection of merged map for evaluation    
      
      exit(0);
    }
  }

}  // namespace explore


/** 
 * @brief Main function
 * */
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
