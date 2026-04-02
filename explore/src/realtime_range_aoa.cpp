#include <explore/explore.h>
#include <explore/custom_logger.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <deque>
#include <mutex>
#include <random>
#include <map>

class RealtimeRangeAOA
{
public:
  RealtimeRangeAOA() : nh_("~")
  {
    // Parameter initialization
    nh_.param("robot_name", tb3_name_, std::string("tb3_1"));
    nh_.param("profile_file_path", aoa_profile_dir_, std::string("/catkin_ws/src/react-m_explore/explore/data/debug/tx2_aoa_profile__0.csv"));
    nh_.param("peaks_file_path", aoa_fn_, std::string("/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv"));
    nh_.param("pub_aoa_profile", flag_publish_aoa_profile_, false);
    nh_.param("own_robot_id", own_id_, -1);
    nh_.param("use_real_sensor", flag_sensor_obs_, true);
    nh_.param("gt_measurement_interval", measurement_interval_, 4.0);
    nh_.param("visualize_aoa_timer", visualize_aoa_timer_, 180.0);

    // Logic for other robot ID (assuming 2 robots for now, but safer to param)
    other_robot_id_ = (own_id_ == 1) ? 2 : 1;

    CUSTOM_LOG_INFO("AOA file: %s", aoa_profile_dir_.c_str());
    CUSTOM_LOG_INFO("Peaks val file: %s", aoa_fn_.c_str());
    CUSTOM_LOG_INFO("use_real_sensor: %d", flag_sensor_obs_);
    CUSTOM_LOG_INFO("gt_measurement_interval %f", measurement_interval_);

    // Publishers and Subscribers
    neighbor_distance_sub_ = n_.subscribe<std_msgs::Float64MultiArray>("distance_multi", 10, &RealtimeRangeAOA::rangeBearingCB, this);
    true_positions_sub_ = n_.subscribe<geometry_msgs::PoseArray>("/robots_groundtruth_state", 10, &RealtimeRangeAOA::truePoseCB, this);
    range_bearing_pub_ = n_.advertise<wiserx_explore_lite::LocalMeasurement>("range_bearing_estimates", 10);

    start_timer = std::chrono::high_resolution_clock::now();
    start_aoa_check_ = std::chrono::high_resolution_clock::now();
    end_aoa_check_ = std::chrono::high_resolution_clock::now();
  }

  void run()
  {
    ros::spin();
  }

private:
  ros::NodeHandle n_;
  ros::NodeHandle nh_;
  ros::Subscriber neighbor_distance_sub_;
  ros::Subscriber true_positions_sub_;
  ros::Publisher range_bearing_pub_;

  std::string tb3_name_;
  std::string aoa_profile_dir_;
  std::string aoa_fn_;
  bool flag_publish_aoa_profile_;
  int own_id_;
  int other_robot_id_;
  bool flag_sensor_obs_;
  double measurement_interval_;
  double visualize_aoa_timer_; // seconds

  std::vector<std::vector<double>> all_robot_range_data_;
  std::deque<std::pair<double, std::vector<geometry_msgs::Pose>>> pose_deque_vec_;
  
  std::chrono::high_resolution_clock::time_point start_aoa_check_;
  std::chrono::high_resolution_clock::time_point end_aoa_check_;
  std::chrono::high_resolution_clock::time_point start_timer;
  
  time_t last_file_mod_time_ = 0;
  double last_csi_time_ = 0.0;
  bool first_itr_ = true;
  std::streampos last_file_pos_ = 0;

  struct AoaData {
      double timestamp;
      double variance;
      std::vector<double> peaks;
  };

  // Helper to get file modification time efficiently
  time_t getFileModTime(const std::string& filename)
  {
    struct stat result;
    if (stat(filename.c_str(), &result) == 0)
    {
      return result.st_mtime;
    }
    return 0;
  }

  /**
   * TODO: Fix when there are more that two robots and using mocap message
   */
  void truePoseCB(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    std::time_t current_epoch_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    
    if (pose_deque_vec_.size() > 50)
    {
      pose_deque_vec_.pop_front();
    }
    pose_deque_vec_.push_back(std::make_pair(current_epoch_time, msg->poses));

    end_aoa_check_ = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check_ - start_aoa_check_);

    // If not using real sensors, publish ground truth periodically
    if (!flag_sensor_obs_ && duration.count() > measurement_interval_)
    {
      CUSTOM_LOG_INFO("Publishing true mocap positions (Sim/Baseline)");
      wiserx_explore_lite::LocalMeasurement rb_msg;
      wiserx_explore_lite::RangeBearing rb_raw;

      if (pose_deque_vec_.empty()) return;

      auto latest_true_measurement = pose_deque_vec_.back();
      double current_time_val = latest_true_measurement.first;
      std::vector<geometry_msgs::Pose> closest_true_pose = latest_true_measurement.second;

      if (other_robot_id_ - 1 < closest_true_pose.size() && own_id_ - 1 < closest_true_pose.size())
      {
        rb_raw.true_pose = closest_true_pose[other_robot_id_ - 1];
        rb_msg.own_true_pose = closest_true_pose[own_id_ - 1];
        rb_raw.robot_id = other_robot_id_;
        rb_raw.bearing_profile_variance = 0.0; // No profile in GT mode
        rb_raw.csi_timestamp = current_time_val;
        rb_msg.header.stamp = ros::Time::now();
        rb_msg.other_robots_rb.push_back(rb_raw);
        range_bearing_pub_.publish(rb_msg);
      }
      
      start_aoa_check_ = std::chrono::high_resolution_clock::now();
    }
  }

  void rangeBearingCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if (msg->data.empty()) return;
    
    all_robot_range_data_.push_back(msg->data);

    // Check file modification time using 
    time_t current_mod_time = getFileModTime(aoa_fn_);

    if (current_mod_time != last_file_mod_time_ && current_mod_time != 0)
    {
      last_file_mod_time_ = current_mod_time;
      CUSTOM_LOG_INFO("Checking for measurements (File modified)");

      std::map<int, AoaData> parsed_aoa_data;

      // Read AOA Data
      std::ifstream fin(aoa_fn_);
      if (fin.is_open())
      {
        if (first_itr_)
        {
          fin.seekg(0, std::ios::end);
          last_file_pos_ = fin.tellg();
          first_itr_ = false;
          fin.close();
          return;
        }

        fin.seekg(last_file_pos_);
        std::string line;
        while (std::getline(fin, line))
        {
          if (line.empty()) continue;

          std::stringstream ss(line);
          std::string segment;
          std::vector<std::string> tokens;
          while (std::getline(ss, segment, ','))
          {
            tokens.push_back(segment);
          }

          if (tokens.size() >= 3)
          {
            try
            {
              double ts = std::stod(tokens[0]);
              std::string rid_str = tokens[1];
              double var = std::stod(tokens[2]);
              std::vector<double> peaks;
              for (size_t i = 3; i < tokens.size(); i++)
              {
                peaks.push_back(std::stod(tokens[i]));
              }

              int rid = -1;
              if (rid_str.size() > 2 && rid_str.substr(0, 2) == "tx")
              {
                rid = std::stoi(rid_str.substr(2));
              }

              if (rid != -1)
              {
                parsed_aoa_data[rid] = {ts, var, peaks};
              }
            }
            catch (const std::exception& e)
            {
              CUSTOM_LOG_WARN("Error parsing AOA file line: %s", e.what());
            }
          }
          else
          {
            CUSTOM_LOG_INFO("Insufficient token size detected");
          }
        }
        fin.clear();
        last_file_pos_ = fin.tellg();
        fin.close();
      }

      if (parsed_aoa_data.empty()) {
        CUSTOM_LOG_INFO("No Measurements parsed from aoa_val file");
        return;
      }

      wiserx_explore_lite::LocalMeasurement rb_msg;
      
      for (auto const& item : parsed_aoa_data)
      {
        int rid = item.first;
        AoaData data = item.second;

        wiserx_explore_lite::RangeBearing rb_raw;
        rb_raw.robot_id = rid;
        rb_raw.bearing_profile_variance = data.variance;
        rb_raw.bearing_measurements = data.peaks;
        rb_raw.csi_timestamp = data.timestamp;

        // Sample UWB Data
        std::vector<double> sampled_range_data;
        int range_idx = rid - 1;

        if (!all_robot_range_data_.empty())
        {
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_int_distribution<> distr(0, int(all_robot_range_data_.size()) - 1);

          int points_to_sample = std::min(4, int(all_robot_range_data_.size()));
          for (int n = 0; n < points_to_sample; n++)
          {
            int sample_idx = distr(gen);
            if (range_idx >= 0 && range_idx < all_robot_range_data_[sample_idx].size())
            {
              sampled_range_data.push_back(all_robot_range_data_[sample_idx][range_idx]);
            }
          }
        }
        rb_raw.range_measurements = sampled_range_data;
        auto time_right_now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(time_right_now - start_timer);
        
        if(flag_publish_aoa_profile_ && duration.count() <= visualize_aoa_timer_){
            //Use this to publish the aoa profile also
            CUSTOM_LOG_INFO("Reading AOA profile data");
            std::string aoa_profile_name = aoa_profile_dir_ + "tx" + std::to_string(rid) + "_aoa_profile__0.csv";
            std::ifstream aoa_profile(aoa_profile_name);
            std::vector<double> profile_array;
            if(aoa_profile.is_open()){
                std::string line, val;                  /* string for line & value */
                while (std::getline (aoa_profile, line)) 
                {        /* read each line */
                    std::stringstream s (line);         /* stringstream line */
                    while (getline (s, val, ','))       /* get each value (',' delimited) */
                    {
                        try { profile_array.push_back (std::stod (val)); } catch(...) {}               /* add row vector to array */
                    }
                }
                CUSTOM_LOG_INFO("Elements: %d", int(profile_array.size()));
                aoa_profile.close();
            }
            rb_raw.aoa_profile = profile_array;
        }

        // Match with Ground Truth
        if (!pose_deque_vec_.empty())
        {
          std::vector<std::pair<double, std::vector<geometry_msgs::Pose>>> true_pose_history(pose_deque_vec_.begin(), pose_deque_vec_.end());
          
          auto closest_vals = findClosestPoseToFirstSample(data.timestamp, true_pose_history);
          std::vector<geometry_msgs::Pose> closest_true_pose = closest_vals.second;

          if (rid - 1 < closest_true_pose.size() && own_id_ - 1 < closest_true_pose.size())
          {
            rb_raw.true_pose = closest_true_pose[rid - 1];
            rb_msg.own_true_pose = closest_true_pose[own_id_ - 1];
          }
        }
        rb_msg.other_robots_rb.push_back(rb_raw);
        CUSTOM_LOG_INFO("Got data for Neighboring robot with robot id: %d", rid);
      }

      if (!rb_msg.other_robots_rb.empty())
      {
        rb_msg.header.stamp = ros::Time::now();
        range_bearing_pub_.publish(rb_msg);
      }
      
      all_robot_range_data_.clear();
      start_aoa_check_ = std::chrono::high_resolution_clock::now();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_bearing_publisher", ros::init_options::AnonymousName);
  
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  RealtimeRangeAOA node;
  node.run();

  return 0;
}
