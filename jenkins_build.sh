LAST_STABLE_NUMBER=`grep number $HOME/jobs/agentsystem/lastSuccessful/build.xml | sed 's/[^0-9]//g'`
# remove old build
find $WORKSPACE -maxdepth 1 -name "rtm-ros-robotics-*" -a ! -name "rtm-ros-robotics-$LAST_STABLE_NUMBER" -print -exec rm -fr {} \;
# rosinstall
rosinstall --continue-on-error $WORKSPACE/rtm-ros-robotics-$BUILD_NUMBER /opt/ros/diamondback http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall || rosinstall $WORKSPACE/rtm-ros-robotics-$BUILD_NUMBER
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
$ROSMAKE openhrp3
$ROSMAKE hrpsys_ros_bridge
$ROSMAKE RS003
