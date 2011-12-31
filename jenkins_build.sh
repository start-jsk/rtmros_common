#!/bin/bash

trap 'exit 1' ERR

export LANG=C

DISTRIBUTION=${@-"electric"}

# setup workspace and buildspace
if [ "$WORKSPACE" == "" ]; then # if not jenkins
    export WORKSPACE=$HOME
fi
if [ "$ROS_INSTALLDIR" == "" ]; then # if not jenkins
    export ROS_INSTALLDIR=$WORKSPACE/ros/$DISTRIBUTION
fi

# setup ros packages
wget https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/jsk.rosbuild -O /tmp/jsk.rosbuild
bash /tmp/jsk.rosbuild --show-install | sh

# rosinstall
/usr/local/bin/rosinstall --rosdep-yes --continue-on-error  --delete-changed-uris $ROS_INSTALLDIR /opt/ros/$DISTRIBUTION  http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall || rosinstall $ROS_INSTALLDIR

# copy jenkins source
if [ "$JENKINS_HOME" != "" ]; then #if jenkins
    rm -fr $ROS_INSTALLDIR/rtm-ros-robotics/rtmros_common/
    cp -r $WORKSPACE/rtm-ros-robotics/rtmros_common $ROS_INSTALLDIR/rtm-ros-robotics/rtmros_common
    # set ROS_HOME under workspace so that we can check from web interface
    echo "export ROS_HOME=$WORKSPACE/.ros" >> $ROS_INSTALLDIR/setup.sh
fi

# source
. $ROS_INSTALLDIR/setup.sh
rospack profile
# set environment
export ROS_PARALLEL_JOBS=-j4
unset SVN_REVISION ## this conflicts with mk/svn_checkout.mk
# rosmake
ROSMAKE='rosmake --status-rate=0 --rosdep-install --rosdep-yes'
(cd `rospack find euslisp`; svn up; svn up)
$ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE  euscollada
$ROSMAKE rtmros_common
$ROSMAKE openhrp3
$ROSMAKE hrpsys_ros_bridge
$ROSMAKE mrobot_ros_bridge
$ROSMAKE RS003
