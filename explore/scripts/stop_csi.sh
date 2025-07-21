#!/bin/bash
echo "abc123" | sudo -S killall log_to_file
#echo "abc123" | sudo -S killall random_packets_two_antenna

rm -rf $dir_path/csi_rx.dat
rm -rf  $dir_path/*.csv

echo "abc123" | sudo -S cp ${HOME}/catkin_ws/src/react-m_explore/explore/data/temp_csi_$1.dat ${HOME}/catkin_ws/src/react-m_explore/explore/data/csi_$1.dat
echo "abc123" | sudo -S cp ${HOME}/catkin_ws/src/react-m_explore/explore/data/motorjoint_displacement.csv ${HOME}/catkin_ws/src/react-m_explore/explore/data/motorjoint_displacement_final.csv

