#!/bin/bash

RATE=0.25

if [ "$1" != "" ]; then
    RATE=$1
fi

echo "RATE = ${RATE}"

rosparam set /use_sim_time false

#roslaunch hrpsys_gazebo screenpoint_resize_view.launch COMPRESS_TYPE:=theora &
roslaunch hrpsys_gazebo screenpoint_resize_view.launch RESIZE_RATE:=${RATE}&
PID=$!

rosrun image_view image_view image:=/head_snap/image_rect _image_transport:=compressed &
PID="$PID $!"
rosrun image_view image_view image:=/lhand_snap/image_rect _image_transport:=compressed &
PID="$PID $!"
rosrun image_view image_view image:=/rhand_snap/image_rect _image_transport:=compressed &
PID="$PID $!"

sleep 15
rosparam set /use_sim_time true

trap "kill -SIGINT $PID; wait" 1 2 3 15

wait

