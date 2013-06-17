#!/bin/bash

rosparam set /use_sim_time false

#roslaunch hrpsys_gazebo screenpoint_resize_view.launch COMPRESS_TYPE:=theora &
roslaunch hrpsys_gazebo screenpoint_resize_view.launch &
PID=$!

sleep 15
rosparam set /use_sim_time true

trap "kill -SIGINT $PID; wait $PID" 1 2 3 15

wait $PID

