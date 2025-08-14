#!/bin/bash

#$1 = local BotID

echo "abc123" | sudo -S ${HOME}/linux-80211n-csitool-supplementary/netlink/log_to_file ${HOME}/catkin_ws/src/wsr_ros/data/temp_csi_$1.dat &


