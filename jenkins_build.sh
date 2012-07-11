#!/bin/bash

# setup workspace and buildspace
if [ "$WORKSPACE" == "" ]; then # if not jenkins
    export WORKSPACE=$HOME
fi
. $WORKSPACE/ros/electric/jsk-ros-pkg/jsk.rosbuild -e $@

function install-rtm-ros-robotics {
    install-pkg http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/jsk.rosinstall http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstal
}
function compile-rtm-ros-robotics {
    compile-pkg euscollada rtmros_common openhrp3 hrpsys_ros_bridge mrobot_ros_bridge RS003 openrtm_ros_bridge
}
function test-jsk-ros-pkig {
    test-pkg openrtm_ros_bridge openhrp3 hrpsys hrpsys_ros_bridge
e_pr2
}

setup-ros
apt-get-ros-pkg
install-rtm-ros-robotics
compile-rtm-ros-robotics

