#!/bin/bash

echo "`rospack find hironx_ros_bridge`/gui/.robothost"
cat `rospack find hironx_ros_bridge`/gui/.robothost
rosrun hrpsys hrpsyspy `rospack find hironx_ros_bridge`/gui/gui.py