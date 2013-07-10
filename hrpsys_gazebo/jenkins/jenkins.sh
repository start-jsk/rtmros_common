#!/bin/bash

wget 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosbuild?format=raw' -O /tmp/jsk.rosbuild.$$
. /tmp/jsk.rosbuild.$$ $1 -e

function install-hrpsys-atlas {
    install-pkg 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosinstall?format=raw' http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall
}

function compile-hrpsys-atlas {
    compile-pkg hrpsys_gazebo
}

set -x

setup-ros
apt-get-ros-package
install-hrpsys-atlas
compile-hrpsys-atlas


