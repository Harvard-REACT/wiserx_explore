
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
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "tf/tf.h"


bool got_robot_1_data = false, got_robot_2_data = false;
geometry_msgs::Pose robot_1_pose;
geometry_msgs::Pose robot_2_pose;

/** 
 * @brief Get robot 1's pose
 * */
void vicon_robot_1_cb(const geometry_msgs::PoseStamped::ConstPtr& input_msg)
{
    if(!got_robot_1_data)
    {     
        robot_1_pose = input_msg->pose;
        got_robot_1_data = true;        
    }
}


/** 
 * @brief Get robot 2's pose
 * */
void vicon_robot_2_cb(const geometry_msgs::PoseStamped::ConstPtr& input_msg)
{
    if(!got_robot_2_data)
    {     
        robot_2_pose = input_msg->pose;  
        got_robot_2_data = true;
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "groundtruth_state_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("/robots_groundtruth_state", 1000);
  ros::Subscriber sub1 = n.subscribe("/vrpn_client_node/REACT_TB3_1/pose", 10, vicon_robot_1_cb);
  ros::Subscriber sub2 = n.subscribe("/vrpn_client_node/REACT_TB3_2/pose", 10, vicon_robot_2_cb);

  ros::Rate loop_rate(2);
  geometry_msgs::PoseArray msg;
  
  /* Publish the info together in a singel pose array topic*/
  while (ros::ok())
  {
    msg.poses.clear();
    ros::spinOnce();
    ROS_INFO("Pubishing data");
    msg.poses.push_back(robot_1_pose);
    msg.poses.push_back(robot_2_pose);
    pub.publish(msg);
    got_robot_1_data = false;
    got_robot_2_data = false;
    loop_rate.sleep();
  }
  return 0;
}

