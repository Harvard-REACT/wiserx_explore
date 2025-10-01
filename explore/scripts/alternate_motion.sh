#!/bin/bash

itr=0
while [ $itr -lt 2 ]
do	
#echo "Pub for tb3_1"
rostopic pub --once /tb3_1/wsr_antenna_motor/start_motion std_msgs/Bool "data: true" &

sleep 1.5

echo "Pub for tb3_2"
rostopic pub --once /tb3_2/wsr_antenna_motor/start_motion std_msgs/Bool "data: true" &

#Do not comment this out as the rostopic is running in background and will eat up all memory. 
sleep 1.5

done
