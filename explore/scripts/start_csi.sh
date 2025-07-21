#!/bin/bash

#$1 = local BotID

dir_path="${HOME}/catkin_ws/src/wsr_exploration/data"

echo "abc123" | sudo -S ${HOME}/WSR-Toolbox-linux-80211n-csitool-supplementary/netlink/log_to_file ${HOME}/catkin_ws/src/wsr_exploration/data/temp_csi_$1.dat &

sleep 0.05

#echo "abc123" | sudo -S ${HOME}/linux-80211n-csitool-supplementary/injection_bak/random_packets 10000000 57 1 5000 & #Only send forward packets
echo "abc123" | sudo -S ${HOME}/WSR-Toolbox-linux-80211n-csitool-supplementary/injection/random_packets_two_antenna 100000000 57 1 5000

#echo "abc123" | sudo -S ${HOME}/linux-80211n-csitool-supplementary/injection_multiple/random_packets 1000000 29 1 7500 3 & #dynamic random_packets

