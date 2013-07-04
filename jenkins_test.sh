#!/bin/bash

function error {
    rosrun rosunit clean_junit_xml.py
    echo "source $ROS_WORKSPACE/setup.bash"
    exit 1
}
rm -fr $ROS_HOME/test_results/
trap error ERR

wget 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosbuild?format=raw' -O /tmp/jsk.rosbuild.$$
/tmp/jsk.rosbuild.$$ $1 -e

install-pkg 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosinstall?format=raw' http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall

compile-pkg hrpsys_tutorials hrpsys_ros_bridge_tutorials hironx_ros_bridge

test-pkg hrpsys_tutorials hrpsys_ros_bridge_tutorials hironx_ros_bridge

#
(which rosrun && rosrun rosunit clean_junit_xml.py; echo "done")     # check error

