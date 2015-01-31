#!/bin/bash

CORBA_HOSTNAME=$1
CORBA_HOSTNAME=${CORBA_HOSTNAME:="hiro014"}
CORBA_HOST_PORT=$2
CORBA_HOST_PORT=${CORBA_HOST_PORT:="15005"}

FILENAME_LOG_COMMANDS=/tmp/rtmros_diagnosisinfo_commands_`date +"%Y%m%d-%H%M%S"`.log
FILENAME_LOG_ROS=/tmp/rtmros_diagnosisinfo_ros_`date +"%Y%m%d-%H%M%S"`.tgz
FILENAME_LOG_ALL=/tmp/rtmros_diagnosisinfo_all_`date +"%Y%m%d-%H%M%S"`.tgz

env |grep ROS | tee -a ${FILENAME_LOG_COMMANDS}
ifconfig | tee -a ${FILENAME_LOG_COMMANDS}
rosrun rtshell rtls ${CORBA_HOSTNAME}:${CORBA_HOST_PORT}/ 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
# Get packages' version. Ref. http://askubuntu.com/a/347563/24203
dpkg -l | grep '^ii' | grep ros- | awk '{print $2 "\t" $3}' 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}

tar -C ~/.ros/log -cvzf ${FILENAME_LOG_ROS} `cd ~/.ros/log; ls -d * | head -1`

tar cfvz ${FILENAME_LOG_ALL} ${FILENAME_LOG_COMMANDS} ${FILENAME_LOG_ROS}

echo "=== All diagnostic info recorded into a tarball: ${FILENAME_LOG_ALL}"
echo "=== Ask at rtm-ros-robotics@googlegroups.com"
