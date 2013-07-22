#!/bin/bash -ex
#
# This script updates following programs:
#  drcsim
#  gazebo
#  hrpsys
#  hrpsys_ros_bridge
#  hrpsys_gazebo
#   - also add sensors to dae file
#


source ~/ros/${ROS_DISTRO}/setup.sh

export ROS_PACKAGE_PATH_ORG=$ROS_PACKAGE_PATH
source /usr/share/drcsim/setup.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH_ORG:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=`echo $(echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep -v "opt/ros"; echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep "opt/ros") | sed -e "s/ /:/g"`

sudo apt-get update
sudo apt-get install drcsim gazebo

cd `rospack find roseus`/..
svn up

cd `rospack find hrpsys`/..
svn up

cd hrpsys/build/hrpsys-base-source/
svn up
cd `rospack find hrpsys`
rm -rf installed
make

cd `rospack find hrpsys_ros_bridge`
make

cd `rospack find hrpsys_gazebo`
rosmake

if [ `grep gyro ./models/atlas.dae | wc -l` -eq 0 ]; then
	mv ./models/atlas.dae ./models/atlas.dae.bak
	./scripts/add_sensor_to_collada.py ./models/atlas.dae.bak > ./models/atlas.dae
	make
fi
