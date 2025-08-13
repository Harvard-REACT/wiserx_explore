#include<explore/explore.h>

auto start_aoa_check = std::chrono::high_resolution_clock::now();
auto end_aoa_check = std::chrono::high_resolution_clock::now();
auto duration_aoa_check = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check - start_aoa_check);
std::string aoa_fn = "";
std::string aoa_profile_name = "";
std::ifstream fin, aoa_profile;
std::vector<double> all_range_data, sampled_range_data, top_aoa_peaks;
bool Flag_get_data_ = false, first_itr=true;
time_t last_time, current_time_val;
ros::Publisher range_bearing_publisher;
double profile_variance = 0;
std::vector<double> profile_array;
std::string tb3_name = "";
std::string onboard_name = "" ;
bool FLAG_publish_aoa_profile=false; 
bool Flag_sensor_obs=false;
int own_id=-1, other_robot_id = -1;
std::deque<std::pair<double, std::vector<geometry_msgs::Pose>>> pose_deque_vec;
std::mutex true_pose_mutex;
double measurement_interval = 0;

void TruePoseCB(const geometry_msgs::PoseArray::ConstPtr& msg){
    //ROS_INFO(" ========= Deque size : %d ======== ", int(pose_deque_vec.size()));
    std::time_t current_epoch_time;
    const auto now = std::chrono::system_clock::now();
    current_epoch_time = std::chrono::system_clock::to_time_t(now); 
    if(pose_deque_vec.size() > 50) pose_deque_vec.pop_front();
    pose_deque_vec.push_back(std::make_pair(current_epoch_time, msg->poses));
    
    duration_aoa_check = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check - start_aoa_check);
    if(!Flag_sensor_obs && duration_aoa_check.count() > measurement_interval){
        ROS_INFO("Publishing true mocap positions");
	    wiserx_explore_lite::LocalMeasurement rb_msg;
	    wiserx_explore_lite::RangeBearing rb_raw;
	    auto latest_true_measurement = pose_deque_vec.back();
        current_time_val = latest_true_measurement.first;
        std::vector<geometry_msgs::Pose> closest_true_pose = latest_true_measurement.second;
        rb_raw.true_pose = closest_true_pose[other_robot_id-1];
        rb_msg.own_true_pose = closest_true_pose[own_id-1];
	    rb_raw.robot_id = other_robot_id;
        rb_raw.bearing_profile_variance = profile_variance;
        rb_raw.csi_timestamp = current_time_val;
        rb_msg.header.stamp = ros::Time::now();
        rb_msg.other_robots_rb.push_back(rb_raw);
        range_bearing_publisher.publish(rb_msg);
	    start_aoa_check = std::chrono::high_resolution_clock::now();
    }
    end_aoa_check = std::chrono::high_resolution_clock::now();
}

void range_bearing_CB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    all_range_data.push_back(msg->data[0]); 
    duration_aoa_check = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check - start_aoa_check);
    if(duration_aoa_check.count() > measurement_interval) //Get a measurement estimate every x seconds 
    {  
        ROS_INFO("Checking for measurements");
	    wiserx_explore_lite::LocalMeasurement rb_msg;

        //Get UWB range value 
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> distr(0, int(all_range_data.size())); // define the range
        
        int points_to_sample = std::min(2,int(all_range_data.size())/2);

        for(int n=0; n<points_to_sample; n++) //Get 5 random samples from UWB node
        {
            sampled_range_data.push_back(all_range_data[distr(gen)]);// randomly sample uwb range value
        }

        all_range_data.clear();

        //Get WiFi AOA value
        fin.open(aoa_fn);
        if(fin.is_open()) 
        {
            fin.seekg(-2,std::ios_base::end);                // go to one spot before the EOF

            bool keepLooping = true;
            while(keepLooping) {
                char ch;
                fin.get(ch);                            // Get current byte's data

                if((int)fin.tellg() <= 1) {             // If the data was at or before the 0th byte
                    fin.seekg(0);                       // The first line is the last line
                    keepLooping = false;                // So stop there
                }
                else if(ch == '\n') {                   // If the data was a newline
                    keepLooping = false;                // Stop at the current position.
                }
                else {                                  // If the data was neither a newline nor at the 0 byte
                    fin.seekg(-2,std::ios_base::cur);        // Move to the front of that data, then to the front of the data before it
                }
            }

            std::string lastLine; 
            std::vector <std::string> tokens;           
            getline(fin,lastLine);                      // Read the current line
            fin.close();
            
            // stringstream class check1
            std::stringstream check1(lastLine);
            std::string intermediate;
            
            // Tokenizing w.r.t. space ' '
            while(getline(check1, intermediate, ','))
            {
                tokens.push_back(intermediate);
            }

            if(first_itr)
            {
                last_time = strtoul( tokens[0].c_str(), NULL, 0 );
                first_itr = false;
            }

            current_time_val = strtoul( tokens[0].c_str(), NULL, 0 );
            if( current_time_val > last_time-2)
            {
                //<timstamp, txid, profile_variance, topN phi angles>
                ROS_INFO("Reading from aoa file");
                profile_variance = stod(tokens[2]);
                for(int i = 3; i < tokens.size(); i++) //The first two values are timestamp and TX ID
                    top_aoa_peaks.push_back(stod(tokens[i]));
                
                if(FLAG_publish_aoa_profile){
            
                    //Use this to publish the aoa profile also
		             ROS_INFO("Reading AOA profile data");
                    aoa_profile.open(aoa_profile_name); 
                    if(aoa_profile.is_open()){
                        std::string line, val;                  /* string for line & value */
                    //       /* vector of vector<int>  */

                        while (std::getline (aoa_profile, line)) 
                        {        /* read each line */
                            std::stringstream s (line);         /* stringstream line */
                            while (getline (s, val, ','))       /* get each value (',' delimited) */
                                profile_array.push_back (std::stod (val));                /* add row vector to array */
                        }
                        ROS_INFO("Elements: %d", int(profile_array.size()));
                  }
                  aoa_profile.close();

                }

                                
                last_time = current_time_val;
            }
        }
        
        //Publish the range and bearing AOA
        ROS_INFO("Ranged sample size: %lu", sampled_range_data.size());
        ROS_INFO("Top AOA peak size: %lu", top_aoa_peaks.size());
        if(sampled_range_data.size() > 0 && top_aoa_peaks.size()>0)
        {
            wiserx_explore_lite::RangeBearing rb_raw;
            for(int i = 0; i<sampled_range_data.size();i++)
            {
                    ROS_INFO("Range: %f", sampled_range_data[i]);
                    rb_raw.range_measurements.push_back(sampled_range_data[i]);
            }

            for(int i = 0; i<top_aoa_peaks.size();i++){
                    ROS_INFO("AOA: %f", top_aoa_peaks[i]);
                    rb_raw.bearing_measurements.push_back(top_aoa_peaks[i]);
            }
                
            ROS_INFO(" ========= Deque size : %d ======== ", int(pose_deque_vec.size()));
            //This ensures that mocap measurements, if available, are always used with real sensor data to validate.
	        if(int(pose_deque_vec.size()) > 0){
                    true_pose_mutex.lock();
                    std::vector<std::pair<double, std::vector<geometry_msgs::Pose>>> true_pose_history = {pose_deque_vec.begin(), pose_deque_vec.end()};
                    true_pose_mutex.unlock();
                    std::pair<double, std::vector<geometry_msgs::Pose>> closest_vals  = findClosestPoseToFirstSample(current_time_val, true_pose_history);
                    std::vector<geometry_msgs::Pose> closest_true_pose = closest_vals.second;
                    rb_raw.true_pose = closest_true_pose[other_robot_id-1];
                    rb_msg.own_true_pose = closest_true_pose[own_id-1];
            }
            
            rb_raw.robot_id = other_robot_id;
	        rb_raw.bearing_profile_variance = profile_variance;
            rb_raw.aoa_profile = profile_array;
            rb_raw.csi_timestamp = current_time_val;
            rb_msg.header.stamp = ros::Time::now();
            rb_msg.other_robots_rb.push_back(rb_raw);
            range_bearing_publisher.publish(rb_msg);
            
            sampled_range_data.clear();
            top_aoa_peaks.clear();
            profile_array.clear();
        }
        else
        {
            ROS_INFO("No data");
        }

        start_aoa_check = std::chrono::high_resolution_clock::now();
    }
    end_aoa_check = std::chrono::high_resolution_clock::now();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "range_bearing_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    nh.param("robot_name", tb3_name, std::string("tb3_1"));
    nh.param("profile_file_path", aoa_profile_name, std::string("/catkin_ws/src/react-m_explore/explore/data/debug/tx2_aoa_profile__0.csv"));
    nh.param("peaks_file_path", aoa_fn, std::string("/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv"));
    nh.param("pub_aoa_profile", FLAG_publish_aoa_profile, false);
    nh.param("own_robot_id", own_id,-1);
    nh.param("use_real_sensor", Flag_sensor_obs, true);
    nh.param("measurement_interval", measurement_interval,4.0);
    other_robot_id = own_id == 1 ? 2:1;

    ROS_INFO("AOA file: %s", aoa_profile_name.c_str());
    ROS_INFO("Peaks val file: %s", aoa_fn.c_str());
    ROS_INFO("use_real_sensor: %d", Flag_sensor_obs);
    ROS_INFO("measurement_interval %f", measurement_interval);

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                        ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
  
    ros::NodeHandle n;
    ros::Subscriber neighbor_distance_ = n.subscribe<std_msgs::Float64MultiArray> ("distance_multi", 10, range_bearing_CB);
    ros::Subscriber true_positions_ = n.subscribe<geometry_msgs::PoseArray> ("/robots_groundtruth_state", 10, TruePoseCB);
    range_bearing_publisher =  n.advertise<wiserx_explore_lite::LocalMeasurement>("range_bearing_estimates", 10);    
    ros::spin();

    return 0;
}
