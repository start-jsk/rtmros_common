#!/bin/bash

rosparam set /use_sim_time false

roslaunch hrpsys_gazebo screenpoint_resize_view.launch &

sleep 2
rosparam set /use_sim_time true
