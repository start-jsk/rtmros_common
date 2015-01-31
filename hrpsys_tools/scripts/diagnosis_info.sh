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
rosrun rtshell rtls ${CORBA_HOSTNAME}:${CORBA_HOST_PORT}/ | tee -a ${FILENAME_LOG_COMMANDS}
dpkg -p ros-hydro-hironx-ros-bridge | grep Ver  | tee -a ${FILENAME_LOG_COMMANDS}
dpkg -p ros-hydro-rtmros-common | grep Ver  | tee -a ${FILENAME_LOG_COMMANDS}
dpkg -p ros-hydro-hrpsys | grep Ver  | tee -a ${FILENAME_LOG_COMMANDS}
dpkg -p ros-hydro-openhrp3 | grep Ver  | tee -a ${FILENAME_LOG_COMMANDS}
dpkg -p ros-hydro-openrtm-aist-core | grep Ver  | tee -a ${FILENAME_LOG_COMMANDS}

tar -C ~/.ros/log -cvzf ${FILENAME_LOG_ROS} `cd ~/.ros/log; ls -d * | head -1`

tar cfvz ${FILENAME_LOG_ALL} ${FILENAME_LOG_COMMANDS} ${FILENAME_LOG_ROS}

echo "=== All diagnostic info recorded into a tarball: ${FILENAME_LOG_ALL}"
echo "=== Ask at rtm-ros-robotics@googlegroups.com"
