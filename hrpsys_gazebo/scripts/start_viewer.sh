#!/bin/bash

RATE=0.25

if [ "$1" != "" ]; then
    RATE=$1
fi

echo "RATE = ${RATE}"

rosparam set /use_sim_time false

# [usage] when you call `rosservice call /head_snap/snapshot`, image will show!!

#roslaunch hrpsys_gazebo screenpoint_resize_view.launch COMPRESS_TYPE:=theora &
roslaunch hrpsys_gazebo screenpoint_resize_view.launch RESIZE_RATE:=${RATE} &
PID=$!

#rosrun image_view image_view image:=/head_snap/image_rect _image_transport:=compressed &
roslaunch hrpsys_gazebo screenpoint_resize_view.launch CAMERA:=/head_snap IMAGE_TYPE:=image_rect RESIZE_RATE:=${RATE} &

PID="$PID $!"
rosrun image_view image_view image:=/lhand_snap/image_rect _image_transport:=compressed &
PID="$PID $!"
rosrun image_view image_view image:=/rhand_snap/image_rect _image_transport:=compressed &
PID="$PID $!"

sleep 15
rosparam set /use_sim_time true

trap "kill -SIGINT $PID; wait" 1 2 3 15

wait

