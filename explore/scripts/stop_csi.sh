#!/bin/bash
echo "abc123" | sudo -S killall log_to_file
echo "abc123" | sudo -S killall random_packets_two_antenna

rm -rf $dir_path/csi_rx.dat
rm -rf  $dir_path/*.csv

echo "abc123" | sudo -S cp ${HOME}/catkin_ws/src/wsr_exploration/data/temp_csi_$1.dat ${HOME}/catkin_ws/src/wsr_exploration/data/csi_$1.dat
