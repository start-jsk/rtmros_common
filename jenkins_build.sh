#!/bin/bash

trap 'exit 1' ERR

export LANG=C
LAST_STABLE_NUMBER=`grep '^  <number' $HOME/jobs/agentsystem/lastSuccessful/build.xml | sed 's/[^0-9]//g'`
# remove old build
echo "last stable number ... $LAST_STABLE_NUMBER"
find $WORKSPACE -maxdepth 1 -name "rtm-ros-robotics-*" -a ! -name "rtm-ros-robotics-$LAST_STABLE_NUMBER" -a -ctime +3 -exec echo "remove " {} \; -exec rm -fr {} \;
# rosinstall
rosinstall --continue-on-error $WORKSPACE/rtm-ros-robotics-$BUILD_NUMBER /opt/ros/electric http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall || rosinstall $WORKSPACE/rtm-ros-robotics-$BUILD_NUMBER
# copy jenkins source
rm -fr rtm-ros-robotics-$BUILD_NUMBER/rtm-ros-robotics/rtmros_common/
cp -r rtm-ros-robotics/rtmros_common rtm-ros-robotics-$BUILD_NUMBER/rtm-ros-robotics/rtmros_common
# set ROS_HOME under workspace so that we can check from web interface
echo "export ROS_HOME=$WORKSPACE/.ros" >> rtm-ros-robotics-$BUILD_NUMBER/setup.sh
# source
. rtm-ros-robotics-$BUILD_NUMBER/setup.sh
rospack profile
# set environment
export ROS_PARALLEL_JOBS=-j4
unset SVN_REVISION ## this conflicts with mk/svn_checkout.mk
# rosmake
ROSMAKE='rosmake --status-rate=0 --rosdep-install --rosdep-yes'
(cd `rospack find euslisp`; svn up; svn up)
$ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE  euscollada
$ROSMAKE rtmros_common || $ROSMAKE rtmros_common
$ROSMAKE openhrp3 || $ROSMAKE openhrp3
$ROSMAKE hrpsys_ros_bridge
$ROSMAKE mrobot_ros_bridge
$ROSMAKE RS003
