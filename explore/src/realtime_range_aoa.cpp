#include<explore/explore.h>
#include<explore_lite/RangeBearing.h>

auto start_aoa_check = std::chrono::high_resolution_clock::now();
auto end_aoa_check = std::chrono::high_resolution_clock::now();
auto duration_aoa_check = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check - start_aoa_check);
std::string aoa_fn = "/home/react-ws-1/catkin_ws/src/wsr_exploration/data/aoa_val.csv";
std::ifstream fin;
std::vector<double> all_range_data, sampled_range_data, top_aoa_peaks;
bool Flag_get_data_ = false, first_itr=true;
time_t last_time, current_time_val;
ros::Publisher range_bearing_publisher;


void range_bearing_CB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    
    all_range_data.push_back(msg->data[0]);
    
    duration_aoa_check = std::chrono::duration_cast<std::chrono::seconds>(end_aoa_check - start_aoa_check);
    if(duration_aoa_check.count() > 2) //Get a position estimate every 5 seconds 
    {  
        
        sampled_range_data.clear();
        top_aoa_peaks.clear();
        
        //Get UWB range value 
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> distr(0, int(all_range_data.size())); // define the range
        
        int points_to_sample = std::min(10,int(all_range_data.size())/2);

        for(int n=0; n<points_to_sample; n++) //Get 20 random samples from UWB node
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
            if( current_time_val > last_time )
            {
                for(int i = 2; i < tokens.size(); i++) //The first two values are timestamp and TX ID
                    top_aoa_peaks.push_back(stod(tokens[i]));
                
                last_time = current_time_val;
            }
        }
        
        //Publish the range and bearing AOA
        if(sampled_range_data.size() > 0 && top_aoa_peaks.size()>0)
        {
            explore_lite::RangeBearing rbmsg;
            for(int i = 0; i<sampled_range_data.size();i++)
            {
                ROS_INFO("Range: %f", sampled_range_data[i]);
                rbmsg.range_measurements.push_back(sampled_range_data[i]);
            }

            for(int i = 0; i<top_aoa_peaks.size();i++)
            {
                ROS_INFO("AOA: %f", top_aoa_peaks[i]);
                rbmsg.brearing_measurements.push_back(top_aoa_peaks[i]);
            }
            range_bearing_publisher.publish(rbmsg);

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
  ros::init(argc, argv, "range_bearing_publisher");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  ros::NodeHandle n;
  ros::Subscriber neighbor_distance_ = n.subscribe<std_msgs::Float64MultiArray> ("/tb3_2/distance_multi", 10, range_bearing_CB);
  range_bearing_publisher =  n.advertise<explore_lite::RangeBearing>("/tb3_2/range_bearing_estimates", 10);
  
  ros::spin();

  return 0;
}