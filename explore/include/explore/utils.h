#ifndef WSR_EXPLORE_UTILS_H
#define WSR_EXPLORE_UTILS_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <regex>
#include <iterator>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <random>
#include <pwd.h>
#include <time.h>
#include <chrono>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/quadmap.h>
#include <explore/state_estimation_filter.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include <natnet_pkg/PoseArrayID.h>
#include <wsr_exploration/QuadmapViz.h>
#include <wsr_exploration/RelativeEstimate.h>
#include <wsr_exploration/FrontierInfo.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include<explore_lite/LocalMeasurement.h>
#include<explore_lite/RangeBearing.h>
#include <ros/ros.h>
#include "tf/tf.h"

        
inline double wrapToPi(double angle) {
    // Normalize to [0, 2*pi)
    angle = fmod(angle, 2 * M_PI);

    // Adjust to [-pi, pi]
    if (angle >= M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) { // This case handles negative angles that wrap past -pi
        angle += 2 * M_PI;
    }
    return angle;
}

inline double wrap0to360(double val) 
{
    val = fmod(val, 360);

    if (val < 0)
        val += 360;

    return val;
}

inline double wrapto180(double val)
{
    val = fmod(val+180.0, 360.0);
    if(val<0) val+= 360.0;
    return (val - 180.0);
}

inline double diff_360(double a, double b) 
{
    double tmp = a-b;

    if(tmp > 180)
        tmp -=360;
    else if (tmp < -180)
        tmp += 360;

    return tmp;
}


inline double quaternionToYaw(const tf::Quaternion& q) 
{
    double yaw = 0.0;


    bool validateQuaternion = (q.getW() != 0 || 
                                q.getX() != 0 || 
                                q.getY() != 0 || 
                                q.getZ() != 0);

    if (validateQuaternion) {
        tf::Matrix3x3 m(q);

        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    }

    return yaw;
}


inline std::pair<double, geometry_msgs::Pose> findClosestPoseToFirstSample(double csi_first_timestamp, std::vector<std::pair<double,geometry_msgs::Pose>>& own_pose_history_vector)
{
    size_t i = 0;
    //Since the timestamps are sorted, we just need to find a first timestamp > csi timestamp and use the pose corresponding to it.
    while(own_pose_history_vector[i].first < csi_first_timestamp && i < own_pose_history_vector.size()) i++;
    std::cout.precision(15);
    std::cout << "[INFO] **** The closest timestamp is : " << own_pose_history_vector[i].first << std::endl;

    return own_pose_history_vector[i];
}


inline std::pair<double, std::vector<geometry_msgs::Pose>> findClosestPoseToFirstSample(double csi_first_timestamp, std::vector<std::pair<double,std::vector<geometry_msgs::Pose>>>& true_pose_history_vector)
{
    size_t i = 0;
    //Since the timestamps are sorted, we just need to find a first timestamp > csi timestamp and use the pose corresponding to it.
    while(true_pose_history_vector[i].first < csi_first_timestamp && i < true_pose_history_vector.size()) i++;
    std::cout.precision(15);
    std::cout << "[INFO] **** The closest timestamp is : " << true_pose_history_vector[i].first << std::endl;

    return true_pose_history_vector[i];
}



//Generate measurements from groundtruth mocap positions
inline std::pair<double, double> get_range_and_bearing_from_groundtruth(geometry_msgs::Pose& robot_i_positions,
                                                                        geometry_msgs::Pose& neighbor_robot_j_positions)
{ 

    float diff_x = neighbor_robot_j_positions.position.x - robot_i_positions.position.x ;
    float diff_y = neighbor_robot_j_positions.position.y - robot_i_positions.position.y;
    float range = sqrt(pow((diff_x),2.0) + pow((diff_y),2.0)); // meters
    float bearing = atan2(diff_y, diff_x); // radians
    ROS_INFO("True range: %f meters, bearing: %f degrees", range, bearing*180/3.14);
    return std::make_pair(range, bearing);
}


//Generate measurements from groundtruth mocap positions
inline void add_noise_to_groundtruth_measurements(std::pair<double, double>& measurements, 
                                                  float range_noise=0.1, 
                                                  float bearing_noise=0.1)
{ 

    std::default_random_engine generator;
    // static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.3); //range noise 0  mean and 30cm stddev in meters 
    // static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.3); //bearing noise  0 mean and 17 deg stddev in radians

    //Used for flight lab vicon hardware experiments - somehow has issues with tb3_2
    static std::normal_distribution<float> range_measurement_gaussian_noise_(0, range_noise); //range noise 0  mean and 10cm stddev in meters 
    static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, bearing_noise); //bearing noise  0 mean and 5 deg stddev in radians

    // static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.1); //range noise mean and stddev in meters 
    // static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg

    // static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.05); //range noise mean and stddev in meters 
    // static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg

    // static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.01); //range noise mean and stddev in meters 
    // static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.03); //bearing noise mean and stddev in radians 2 deg

    //Used mostly in sim
    // static std::normal_distribution<float> range_measurement_gaussian_noise_(0, 0.2); //range noise mean and stddev in meters 20cm
    // static std::normal_distribution<float> bearing_measurement_gaussian_noise_(0, 0.17); //bearing noise mean and stddev in radians 10 deg

    measurements.first = measurements.first +  range_measurement_gaussian_noise_(generator); // meters
    measurements.second = measurements.second +  bearing_measurement_gaussian_noise_(generator); // radians
	ROS_INFO("Noisy range: %f meters, bearing: %f degrees", measurements.first, measurements.second*180/3.14);
}



#endif
