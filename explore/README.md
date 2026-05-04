# WiSER-X Robot Setup & Exploration Guide

---

## TB3_1 Setup

### First-Time: Update udev Rules

```bash
sudo vim /etc/udev/rules.d/99-usb-serial.rules
```

Add the following rules:

```
KERNEL=="ttyUSB0", ATTRS{idVendor}=="10c4", SYMLINK+="sensor_lidar", MODE="0777"
KERNEL=="ttyUSB1", ATTRS{idVendor}=="0403", SYMLINK+="sensor_servo", MODE="0777"
KERNEL=="ttyACM1", ATTRS{idVendor}=="1366", SYMLINK+="sensor_uwb", MODE="0777"
```

Then reload:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

### Collecting Range and AOA

```bash
sudo ./WSR-WifiDriver/setup.sh 57 108 HT20

echo "abc123" | sudo -S ${HOME}/WSR-Toolbox-linux-80211n-csitool-supplementary/injection/random_packets_two_antenna 10000000000 59 1 25000

roslaunch wiserx_explore_lite dynamixel_controller_wsr.launch \
  robot_name:=tb3_1 usb_port:=/dev/sensor_servo servo_vel:=1.2

cd ~/WSR_Project/WSR-Toolbox-cpp/wsr_build
./test_wsr_two_antenna_rotor joint
```

#### Optional

```bash
cd ~/catkin_ws/src/react-m_explore/explore/data
tail -f aoa_val.csv

python2 ~/catkin_ws/src/react-m_explore/explore/scripts/uwb_publisher_multi.py \
  -n /tb3_1 -id 1 --indices 0,2,5

roslaunch wiserx_explore_lite realtime_range_aoa.launch \
  publish_aoa_profile:=true \
  robot_id:=1 \
  aoa_profile_file_path:="/home/explorer-1/catkin_ws/src/react-m_explore/explore/data/" \
  aoa_peaks_file_path:="/home/explorer-1/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv" \
  use_real_sensor:=true \
  gt_measurement_interval:=4.0 \
  robot_name:=tb3_1
```

> **Note:** Set `use_real_sensor:=false` to use only mocap. No need to run UWB sensor node.

---

### Running Exploration

```bash
roslaunch wiserx_explore_lite hw_single_turtlebot3_bringup.launch \
  robot_name:=tb3_1 port:=/dev/sensor_lidar

# Extra buffer to avoid auto-expansion of the map as the robot's pose changes
roslaunch wiserx_explore_lite hw_single_turtlebot3_gmapping.launch \
  robot_name:=tb3_1 xmin:=-4.0 ymin:=-7.5 xmax:=13.5 ymax:=14.5

roslaunch wiserx_explore_lite hw_single_turtlebot3_move_base_and_explore.launch \
  multi_robot_name:=tb3_1 \
  multi_robot_id:=1 \
  open_rviz:=false \
  debug_mocap_pose:=false \
  xmin:=-2.5 ymin:=-6.0 xmax:=7.0 ymax:=1 \
  map_resolution:=0.15 \
  sensor_range:=2.5 \
  robot_initial_map_pose_x:=0 \
  robot_initial_map_pose_y:=0 \
  robot_speed:=0.05 \
  use_real_sensor:=true \
  enable_wsr:=true \
  measurement_interval:=3.5
```

> **Note:**
> - `enable_wsr` — selects the algorithm (e.g., WiSER-X vs. baselines)
> - `use_real_sensor` — uses actual sensor vs. mocap for relative position measurements

#### To Kill `log_to_file`

```bash
echo "abc123" | sudo -S killall log_to_file
```

#### View AOA Profile List

```bash
cd ~/catkin_ws/src/explore_lite/data
tail -f aoa_val.csv
```

---

## TB3_2 Setup

### First-Time: Update udev Rules

```bash
sudo vim /etc/udev/rules.d/99-usb-serial.rules
```

Add the following rules:

```
KERNEL=="ttyUSB0", ATTRS{idVendor}=="0403", SYMLINK+="sensor_servo"
KERNEL=="ttyUSB1", ATTRS{idVendor}=="10c4", SYMLINK+="sensor_lidar"
KERNEL=="ttyACM1", ATTRS{idVendor}=="05e3", SYMLINK+="sensor_uwb", MODE="0777"
```

---

### Collecting Range and AOA

```bash
sudo ./WSR-WifiDriver/setup.sh 59 108 HT20

roslaunch wiserx_explore_lite dynamixel_controller_wsr.launch \
  robot_name:=tb3_2 usb_port:=/dev/sensor_servo servo_vel:=1.2

cd ~/WSR_Project/WSR-Toolbox-cpp/wsr_build
./test_wsr_two_antenna_rotor joint
```

#### Optional

```bash
cd ~/catkin_ws/src/react-m_explore/explore/data
tail -f aoa_val.csv

python2 ~/catkin_ws/src/react-m_explore/explore/scripts/uwb_publisher_multi.py \
  -n /tb3_2 -id 2 --indices 0,2,5

roslaunch wiserx_explore_lite realtime_range_aoa.launch \
  publish_aoa_profile:=true \
  robot_id:=2 \
  aoa_profile_file_path:="/home/explorer3/catkin_ws/src/react-m_explore/explore/data/" \
  aoa_peaks_file_path:="/home/explorer3/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv" \
  gt_measurement_interval:=4.5 \
  robot_name:=tb3_2 \
  use_real_sensor:=true
```

> **Note:** Set `use_real_sensor:=false` to use only mocap. No need to run UWB sensor node.
> Add a buffer of `2 * sensor_range` to the gmapping boundary conditions to prevent auto-expansion.

---

### Running Exploration

```bash
roslaunch wiserx_explore_lite hw_single_turtlebot3_bringup.launch \
  robot_name:=tb3_2 port:=/dev/sensor_lidar

# Extra buffer to avoid auto-expansion of the map as the robot's pose changes.
# Add buffer of 1x sensor_range; explore uses its own boundary separately from the map.
roslaunch wiserx_explore_lite hw_single_turtlebot3_gmapping.launch \
  robot_name:=tb3_2 xmin:=-3.0 ymin:=-7.0 xmax:=11.0 ymax:=5.0

roslaunch wiserx_explore_lite hw_single_turtlebot3_move_base_and_explore.launch \
  multi_robot_name:=tb3_2 \
  multi_robot_id:=2 \
  open_rviz:=false \
  debug_mocap_pose:=false \
  xmin:=-1.5 ymin:=-5.0 xmax:=8.0 ymax:=2.0 \
  map_resolution:=0.15 \
  sensor_range:=2.5 \
  robot_initial_map_pose_x:=0 \
  robot_initial_map_pose_y:=0 \
  robot_speed:=0.05 \
  use_real_sensor:=true \
  enable_wsr:=true \
  measurement_interval:=3.5
```

#### To Kill Servo

```bash
echo "abc123" | sudo -S killall log_to_file
```

---

## TB3_3 Setup

### First-Time: Update udev Rules

```bash
sudo vim /etc/udev/rules.d/99-usb-serial.rules
```

Add the following rules:

```
KERNEL=="ttyUSB0", ATTRS{idVendor}=="0403", SYMLINK+="sensor_servo"
KERNEL=="ttyUSB1", ATTRS{idVendor}=="10c4", SYMLINK+="sensor_lidar"
KERNEL=="ttyACM1", ATTRS{idVendor}=="05e3", SYMLINK+="sensor_uwb", MODE="0777"
```

---

### Collecting Range and AOA

```bash
sudo ./WSR-WifiDriver/setup.sh 59 108 HT20

roslaunch wiserx_explore_lite dynamixel_controller_wsr.launch \
  robot_name:=tb3_3 usb_port:=/dev/sensor_servo servo_vel:=1.2

cd ~/WSR_Project/WSR-Toolbox-cpp/wsr_build
./test_wsr_two_antenna_rotor joint
```

#### Optional

```bash
cd ~/catkin_ws/src/react-m_explore/explore/data
tail -f aoa_val.csv

python2 ~/catkin_ws/src/react-m_explore/explore/scripts/uwb_publisher_multi.py \
  -n /tb3_3 -id 3 --indices 0,2,5

roslaunch wiserx_explore_lite realtime_range_aoa.launch \
  publish_aoa_profile:=true \
  robot_id:=3 \
  aoa_profile_file_path:="/home/explorer-4/catkin_ws/src/react-m_explore/explore/data/" \
  aoa_peaks_file_path:="/home/explorer-4/catkin_ws/src/react-m_explore/explore/data/aoa_val.csv" \
  gt_measurement_interval:=6.0 \
  robot_name:=tb3_3 \
  use_real_sensor:=true
```

> **Note:** Set `use_real_sensor:=false` to use only mocap. No need to run UWB sensor node.
> Add a buffer of `2 * sensor_range` to the gmapping boundary conditions to prevent auto-expansion.

---

### Running Exploration

```bash
roslaunch wiserx_explore_lite hw_single_turtlebot3_bringup.launch \
  robot_name:=tb3_2 port:=/dev/sensor_lidar

# Extra buffer to avoid auto-expansion of the map as the robot's pose changes.
# Add buffer of 1x sensor_range; explore uses its own boundary separately from the map.
roslaunch wiserx_explore_lite hw_single_turtlebot3_gmapping.launch \
  robot_name:=tb3_2 xmin:=-3.0 ymin:=-7.0 xmax:=11.0 ymax:=5.0

roslaunch wiserx_explore_lite hw_single_turtlebot3_move_base_and_explore.launch \
  multi_robot_name:=tb3_2 \
  multi_robot_id:=2 \
  open_rviz:=false \
  debug_mocap_pose:=false \
  xmin:=-1.5 ymin:=-5.0 xmax:=8.0 ymax:=2.0 \
  map_resolution:=0.15 \
  sensor_range:=2.5 \
  robot_initial_map_pose_x:=0 \
  robot_initial_map_pose_y:=0 \
  robot_speed:=0.05 \
  use_real_sensor:=true \
  enable_wsr:=true \
  measurement_interval:=3.5
```

#### To Kill Servo

```bash
echo "abc123" | sudo -S killall log_to_file
```

---

## Workstation Setup

### Terminal 1

```bash
roscore
```

### Terminal 2

```bash
cd ~/catkin_ws/src/m-explore/explore/scripts
./alternate_motion.sh
```

### Start Groundtruth Data Collection and Visualization

```bash
roslaunch wiserx_explore_lite supporting_nodes_sensor_viz.launch \
  groundtruth_access:=true \
  mocap_server_ip:=192.168.1.8 \
  viz_aoa_profiles:=true

roslaunch wiserx_explore_lite supporting_nodes_quadmap_viz.launch

roslaunch wiserx_explore_lite hw_map_merger.launch

roslaunch wiserx_explore_lite hw_map_eval.launch
```

### Visualize Data

```bash
python3.8 ~/WSR_Project/WSR-Toolbox-cpp/scripts/viz_channel_data.py --file $1/$csi_phase_fn
python3.8 ~/WSR_Project/WSR-Toolbox-cpp/scripts/viz_traj.py --file $1/$traj_pkt_fn
```

### Record ROSbag

```bash
rosbag record \
  /tb3_1/range_bearing_estimates \
  /tb3_2/range_bearing_estimates \
  /tb3_1/explore/node_list \
  /tb3_2/explore/node_list
```
