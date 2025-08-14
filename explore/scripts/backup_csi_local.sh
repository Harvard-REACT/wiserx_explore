#!/bin/bash

#$1 local bot rx

dir_path="${HOME}/catkin_ws/src/react-m_explore/explore/data"

echo 'Copying file to back up folder'
timestamp=$(date "+%Y-%m-%d_%H%M%S")
cp $dir_path/csi_$1.dat $dir_path/backup_data/csi_backup_data/csi_$1_$timestamp.dat &
cp $dir_path/motorjoint_displacement.csv $dir_path/backup_data/motor_joint_backup_data/motorjoint_displacement_${timestamp}.csv
#cp $dir_path/t265_rx_trajectory.csv $dir_path/motor_joint_backup_data/t265_rx_trajectory_${timestamp}.csv;
#cp $dir_path/odom_rx_trajectory.csv $dir_path/motor_joint_backup_data/odom_rx_trajectory_${timestamp}.csv;

#echo 'Copying file to the online-data folder'
#online_data="${HOME}/WSR_Project/WSR-Toolbox-cpp/data/online_data/"
#rm -rf $online_data/*
#cp $dir_path/csi_backup_data/csi_$1_$timestamp.dat  $online_data/csi_$1_$timestamp.dat
#cp $dir_path/rx_trajectory.csv $online_data/rx_trajectory_${timestamp}.csv
#cp $dir_path/t265_rx_trajectory.csv $online_data/t265_rx_trajectory_${timestamp}.csv;
#cp $dir_path/odom_rx_trajectory.csv $online_data/odom_rx_trajectory_${timestamp}.csv;
~                                                                                             
