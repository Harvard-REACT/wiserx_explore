
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
  ros::init(argc, argv, "vicon_state_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("/vicon_state_topic", 1000);
  ros::Subscriber sub1 = n.subscribe("/vrpn_client_node/WSR_tb3_1/pose", 10, vicon_robot_1_cb);
  ros::Subscriber sub2 = n.subscribe("/vrpn_client_node/WSR_tb3_2/pose", 10, vicon_robot_2_cb);

  ros::Rate loop_rate(3);
  geometry_msgs::PoseArray msg;
  ros::spinOnce();


  /* Publish the info together in a singel pose array topic*/
  while (ros::ok())
  {
    msg.poses.clear();
    // if(got_robot_1_data == false && got_robot_2_data == false)
    if(got_robot_1_data)
    {
        ROS_INFO("Pubishing data");
        msg.poses.push_back(robot_1_pose);
        msg.poses.push_back(robot_2_pose);
        pub.publish(msg);
        got_robot_1_data = false;
        got_robot_2_data = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
   
  }
  return 0;
}

