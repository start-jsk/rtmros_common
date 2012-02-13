#!/bin/bash

usage_exit () {
    echo "Usage : $0 [-t|--target TARGET_PACKAGE] [--no-install] ROS_DISTRIBUTION" 1>&2
    exit 1
}
trap usage_exit ERR

export LANG=C

# command line parse
GETOPT=`getopt -o ht: -l help,no-install,target: -- "$@"` ; [ $? != 0 ] && usage_exit

TARGET=""
INSTALL=1
eval set -- "$GETOPT"
while [ -n "$1" ] ; do
    case $1 in
        -t|--target) TARGET=$2; shift 2;;
        -h|--help) usage_exit ;;
        --no-install) INSTALL=0; shift ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage_exit;;
    esac
done

DISTRIBUTION=${@-"electric"}
echo "Running $0 with {distribution:$DISTRIBUTION, target:$TARGET}"

# setup workspace and buildspace
if [ "$WORKSPACE" == "" ]; then # if not jenkins
    export WORKSPACE=$HOME
fi
if [ "$ROS_INSTALLDIR" == "" ]; then
    export ROS_INSTALLDIR=$WORKSPACE/ros/$DISTRIBUTION
fi

if [ $INSTALL == 1 ]; then

    # setup ros packages
    wget https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/jsk.rosbuild -O /tmp/jsk.rosbuild
    bash /tmp/jsk.rosbuild --show-install | sh

    # rosinstall
    /usr/local/bin/rosinstall --rosdep-yes --continue-on-error  --delete-changed-uris $ROS_INSTALLDIR /opt/ros/$DISTRIBUTION  http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall || rosinstall $ROS_INSTALLDIR

    # copy jenkins source
    if [ "$JENKINS_HOME" != "" ]; then #if jenkins, use jenkins SVN_REVISION to sync with jenkins SCM plugin
	(cd $ROS_INSTALLDIR/rtm-ros-robotics/rtmros_common/; svn up -r $SVN_REVISION)
	# set ROS_HOME under workspace so that we can check from web interface
	echo "export ROS_HOME=$WORKSPACE/.ros" >> $ROS_INSTALLDIR/setup.sh
    fi
fi

# source
. $ROS_INSTALLDIR/setup.sh
rospack profile
# set environment
export ROS_PARALLEL_JOBS=-j4
unset SVN_REVISION ## this conflicts with mk/svn_checkout.mk
# rosmake

ROSMAKE='rosmake --status-rate=0 --rosdep-install --rosdep-yes --profile -V -r'
(cd `rospack find euslisp`; svn up; svn up)
if [ ! $TARGET ]; then
    $ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE euscollada || $ROSMAKE  euscollada
    $ROSMAKE rtmros_common
    $ROSMAKE openhrp3
    $ROSMAKE hrpsys_ros_bridge
    $ROSMAKE mrobot_ros_bridge
    $ROSMAKE RS003
else
    $ROSMAKE $TARGET
fi

exit 0

