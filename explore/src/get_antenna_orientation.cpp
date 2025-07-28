#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <natnet_pkg/PoseArrayID.h>
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <chrono>
#include <signal.h>
#include <pwd.h>
#include <time.h>
#include <fstream>
#include <cmath>

bool running = true;
bool get_true_orientation=false;
bool first_sample=true;
bool __Flag_start_motion=false;
bool __Flag_collect_ori=false;
nav_msgs::Odometry groundTruthPose;
double orientation_true;
double prev_ts;
double curr_ts;
double diff_ts=0;
double joint_angle=0.0;
std::vector<std::vector<double>> __robot_ori_gt;
std::vector<std::vector<double>> __robot_ori_joint;
std::vector<double> temp_ori_gt;
std::vector<double> temp_ori_joint;


bool validateQuaternion(const tf::Quaternion& quat) {
    return (quat.getW() != 0 || quat.getX() != 0 || quat.getY() != 0 || quat.getZ() != 0);
}


double quaternionToYaw(const tf::Quaternion& q) {
    double yaw = 0.0;

    if (validateQuaternion(q)) {
        tf::Matrix3x3 m(q);

        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    }

    return yaw;
}


void positionCallbackMocap(const natnet_pkg::PoseArrayID::ConstPtr& msg) 
{ 
    
    if(__Flag_collect_ori)
    {
        for (int i=0; i<msg->poses.size(); i++) 
        {

            if(msg->poses[i].ID == 56)
            {
                groundTruthPose.pose.pose.position = msg->poses[i].position;
                groundTruthPose.pose.pose.orientation = msg->poses[i].orientation;
        
                tf::Quaternion q(
                        groundTruthPose.pose.pose.orientation.x,
                        groundTruthPose.pose.pose.orientation.y,
                        groundTruthPose.pose.pose.orientation.z,
                        groundTruthPose.pose.pose.orientation.w
                );

                orientation_true = quaternionToYaw(q);

                temp_ori_gt.clear();
                double nsec_timestamp = msg->header.stamp.sec*1e9 + msg->header.stamp.nsec - msg->latency;
                temp_ori_gt.push_back(nsec_timestamp);
                temp_ori_gt.push_back(0);
                temp_ori_gt.push_back(orientation_true);

                //Keep capturing poses and clear each time after AOA calculation.
                __robot_ori_gt.push_back(temp_ori_gt);


            }
        }
    }
}


void positionCallbackVicon(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
    );

    orientation_true = quaternionToYaw(q);
    //joint_angle = orientation_true;

    if(__Flag_collect_ori)
    {
        temp_ori_joint.clear();
        //wifi_data_packet.ts = wifi_data_packet.tv_sec + wifi_data_packet.tv_usec*pow(10,-6);
        temp_ori_joint.push_back(msg->header.stamp.sec);
        temp_ori_joint.push_back(msg->header.stamp.nsec);
        temp_ori_joint.push_back(orientation_true);
        __robot_ori_joint.push_back(temp_ori_joint);
    }
}


void CallJointState(const sensor_msgs::JointState::ConstPtr& msg) 
{ 
    joint_angle = msg->position[0];
    
    
    if(__Flag_collect_ori)
    {
        temp_ori_joint.clear();
        //wifi_data_packet.ts = wifi_data_packet.tv_sec + wifi_data_packet.tv_usec*pow(10,-6);
        temp_ori_joint.push_back(msg->header.stamp.sec);
        temp_ori_joint.push_back(msg->header.stamp.nsec);
        temp_ori_joint.push_back(msg->position[0]);
        __robot_ori_joint.push_back(temp_ori_joint);
    }
    
}

void CallStartMotion(const std_msgs::Bool::ConstPtr& msg) 
{ 
    __Flag_start_motion = msg->data;
}


void CTRL_C(int sig) 
{
    running = false;    
}


void writeTrajToFile(std::vector<std::vector<double>>& ori_vector, 
                     std::string fn)
{
    std::cout.precision(10);
    std::ofstream myfile (fn);
    std::vector<double> temp;

    std::cout << "Trajectory size " << ori_vector.size() << std::endl;
    if (myfile.is_open())
    {
        for(size_t i = 0; i < ori_vector.size(); i++)
        {
            for(int j=0; j< ori_vector[i].size(); j++)
            {
                myfile << std::fixed << ori_vector[i][j] << ",";
            }
            myfile << "\n";
        }
        
    }
    myfile.close();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "antenna_rotor", 1); //Option to make the node name anonymous
    ros::NodeHandle nh("~");
    std::string robot_name;
    std::string mocap_rigid_body_name;
    nh.param("robot_name", robot_name, std::string("tb3"));
    nh.param("mocap_rigid_body_name", mocap_rigid_body_name, std::string("WSR_tb3_1_antenna_1"));
    struct passwd *pw = getpwuid(getuid());
    std::string homedir = pw->pw_dir;
    double z_vel = 0.0;
    nh.param("servo_velocity", z_vel, 1.0);
    float iteration_duration = 0;
    bool flip = false;
    bool first = true;
    bool __Flag_started_CSI = false;
    ros::Rate rate(100);    
    geometry_msgs::Twist rotate_right, rotate_left;
    rotate_left.angular.z = -z_vel;
    rotate_right.angular.z = z_vel;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/"+robot_name+"/wsr_antenna_motor/cmd_vel", 10);
    ros::Subscriber get_ori = nh.subscribe("/optitrack_pose", 10, positionCallbackMocap);
    ros::Subscriber joint_ori = nh.subscribe("/"+robot_name+"/wsr_antenna_motor/joint_states", 10, CallJointState);
    ros::Subscriber start_rotation = nh.subscribe("/"+robot_name+"/wsr_antenna_motor/start_motion", 10, CallStartMotion);
    //ros::Subscriber joint_ori_vicon = nh.subscribe("/vrpn_client_node/"+mocap_rigid_body_name+"/pose", 10, positionCallbackVicon);

    int i = 0;
    int cmd_status = 0;
    double joint_threshold  = 2.85;
    signal(SIGINT, CTRL_C);
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    std::string csi_start_local_cmd = homedir+"/catkin_ws/src/react-m_explore/explore/scripts/start_csi.sh rx &";
    std::string csi_stop_cmd = homedir+"/catkin_ws/src/react-m_explore/explore/scripts/stop_csi.sh rx";
    std::string csi_backup_cmd = homedir+"/catkin_ws/src/react-m_explore/explore/scripts/backup_csi_local.sh rx ";

    ROS_INFO("Initialized antenna rotation.");

    auto iteration_start = std::chrono::high_resolution_clock::now();
    auto starttime = std::chrono::high_resolution_clock::now();
    auto endtime = std::chrono::high_resolution_clock::now();
    //auto exp_duration = std::chrono::duration_cast<std::chrono::seconds>(endtime - starttime);
    float exp_duration = 0.0;

    while(running)
    {
        ros::spinOnce();
        if(__Flag_start_motion)
        {
            if(!__Flag_started_CSI)
            {
                cmd_status = system(csi_start_local_cmd.c_str());
                if (cmd_status < 0)
                {
                    std::cout << "Error: " << strerror(errno) << '\n';
                    __Flag_start_motion = false;
                    break;
                }
                else
                {
                    __Flag_started_CSI = true;
                    __Flag_collect_ori = true;
                    sleep(1);
                }
                
                starttime = std::chrono::high_resolution_clock::now();
                ROS_INFO("Starting CSI Data Collection");
            }

            if(joint_angle > 3.12 || joint_angle < -3.12)
            {
                ROS_INFO("======= Limit exceeding. Force stop =====");
                for(i=0;i<60;i++)
                {
                    vel_pub.publish(geometry_msgs::Twist());
                    ros::spinOnce();
                    rate.sleep();
                }
                exit(1);
            }

            endtime = std::chrono::high_resolution_clock::now();
            exp_duration = std::chrono::duration<float, std::milli>(endtime - starttime).count() * 0.001;
	        //exp_duration = std::chrono::duration_cast<std::chrono::seconds>(endtime - starttime);

            // if(std::abs(joint_angle - joint_threshold) <=0.025 || exp_duration > 0.25)
            if(std::abs(joint_angle - joint_threshold) <=0.025)
            {                
                //Stop before changing direction or next iteration
                for(i=0;i<60;i++)
                {
                    vel_pub.publish(geometry_msgs::Twist());
                    ros::spinOnce();
                    rate.sleep();
                }

                flip = !flip;
                if(first)
                {
                    first = false;
                }
                
                if(flip)
                    joint_threshold = -2.85; // -120 deg, true, rotate right
                else
                    joint_threshold = 2.85; //2.75,  120 deg,false, rotate left
                
		ROS_INFO("Stopping CSI data collection");       
                ROS_INFO("======= Saving antenna orientation data =====");
                
                //Stop before changing direction or next iteration
                ros::spinOnce();
                __Flag_start_motion = false;
		        __Flag_started_CSI = false;
                __Flag_collect_ori=false;

                time (&rawtime);
                timeinfo = localtime(&rawtime);
                strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H%M%S",timeinfo);
                std::string time_str(buffer);

                //std::string joint_ori_gt = homedir+"/catkin_ws/src/react-m_explore/explore/data/gt_displacement_"+time_str+".csv";
                //std::cout << "Mocap Orientation file " << std::endl;
                //writeTrajToFile(__robot_ori_gt, joint_ori_gt);
                
                std::string joint_ori_fn = homedir+"/catkin_ws/src/react-m_explore/explore/data/motorjoint_displacement.csv";
                std::cout << "Motor Joint Orientation File" << std::endl;
                writeTrajToFile(__robot_ori_joint, joint_ori_fn);

                __robot_ori_gt.clear();
                __robot_ori_joint.clear();
		
                 system(csi_stop_cmd.c_str());
                 system(csi_backup_cmd.c_str());
 
		 ROS_INFO("======= Completed =====");

            }
            else
            {
                if(flip)
                    vel_pub.publish(rotate_right);
                else
                    vel_pub.publish(rotate_left);
            }
        }

        rate.sleep();
    }

    vel_pub.publish(geometry_msgs::Twist());
    ROS_INFO("Exiting.");
    ros::shutdown();
    return 0;
}
