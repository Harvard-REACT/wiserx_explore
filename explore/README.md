First time:
Update the udev rules:
sudo vim /etc/udev/rules.d/99-usb-serial.rules

KERNEL=="ttyUSB0", ATTRS{idVendor}=="10c4", SYMLINK+="sensor_lidar"
KERNEL=="ttyUSB1", ATTRS{idVendor}=="0403", SYMLINK+="sensor_servo"
KERNEL=="ttyACM1", ATTRS{idVendor}=="1366", SYMLINK+="sensor_uwb", MODE="0777"

========================= Collecting Range and AOA=============================
sudo ./WSR-WifiDriver/setup.sh 57 108 HT20
echo "abc123" | sudo -S ${HOME}/WSR-Toolbox-linux-80211n-csitool-supplementary/injection/random_packets_two_antenna 10000000000 59 1 17000

roslaunch wiserx_explore_lite dynamixel_controller_wsr.launch robot_name:=tb3_1 usb_port:=/dev/sensor_servo servo_vel:=0.6

cd ~/WSR_Project/WSR-Toolbox-cpp/wsr_build
./test_wsr_two_antenna_rotor joint

==== Optional =====
cd ~/catkin_ws/src/react-m_explore/explore/data
tail -f aoa_val.csv
===================

python2 ~/catkin_ws/src/uwb_ros_publisher/scripts/uwb_pub_multi.py -s 1 -n /tb3_1/

flight_lroslaunch wiserx_explore_lite realtime_range_aoa.launch publish_aoa_profile:=true robot_id:=1 aoa_profile_file_path:="/home/explorer-1/catkin_ws/src/react-m_explore/explore/data/tx2_aoa_profile__0.csv" aoa_peaks_file_path:="/home/explorer-1/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv" use_real_sensor:=true measurement_interval:=4.0 robot_name:=tb3_1

Note: 
set use_real_sensor:=false to use only mocap. No need to run UWB sensor node. But this will still need the use of ./alternate_motion.sh (and servo should be rotating) to start own pose data collection from the SLAM in the explore.cpp.
===============================================================================


========================= Running Exploration==================================

roslaunch wiserx_explore_lite hw_single_turtlebot3_bringup.launch robot_name:=tb3_1 port:=/dev/sensor_lidar

roslaunch wiserx_explore_lite hw_single_turtlebot3_gmapping.launch robot_name:=tb3_1 xmin:=-3.0 ymin:=-5.75 xmax:=14 ymax:=13

roslaunch wiserx_explore_lite hw_single_turtlebot3_move_base_and_explore.launch multi_robot_name:=tb3_1 multi_robot_id:=1 enable_wsr:=true use_real_sensor:=true open_rviz:=false debug_mocap_pose:=false

Note:
enable_wsr : this decides the type of algorithm to be used (e.g., WiSER-X vs the baselines)
user_real_sensor: determines whether to use actual sensor vs mocap positions to generate measurements.
===============================================================================


TO KILL log_to_file: 
echo "abc123" | sudo -S killall log_to_file

View profile list: 
cd ~/catkin_ws/src/explore_lite/data 
tail -f aoa_val.csv
