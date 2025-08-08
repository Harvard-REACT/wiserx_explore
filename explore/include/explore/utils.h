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



#endif
