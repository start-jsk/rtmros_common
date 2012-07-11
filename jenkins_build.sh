#!/bin/bash

wget https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/jsk.rosbuild -O /tmp/jsk.rosbuild
. /tmp/jsk.rosbuild -e $@

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
apt-get-ros-package
install-rtm-ros-robotics
compile-rtm-ros-robotics

