#!/bin/bash

wget 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosbuild?format=raw' -O /tmp/jsk.rosbuild.$$
. /tmp/jsk.rosbuild.$$ $1 -e

function install-hrpsys-atlas {
    install-pkg 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosinstall?format=raw' http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall
    # install drcsim
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'
    wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install gazebo drcsim
    # merge ROS_PACKAGE_PATh
    ROS_PACKAGE_PATH_ORG=$ROS_PACKAGE_PATH
    source /usr/share/drcsim/setup.sh
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH_ORG:$ROS_PACKAGE_PATH
    export ROS_PACKAGE_PATH=`echo $(echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep -v "opt/ros"; echo $ROS_PACKAGE_PATH | sed -e "s/:/\n/g" | awk '!($0 in A) && A[$0] = 1' | grep "opt/ros") | sed -e "s/ /:/g"`
}

function compile-hrpsys-atlas {
    compile-pkg hrpsys_gazebo
}

set -x

setup-ros
apt-get-ros-package
install-hrpsys-atlas
compile-hrpsys-atlas


