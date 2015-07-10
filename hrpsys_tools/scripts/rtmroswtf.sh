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
## Get packages' version. Ref. http://askubuntu.com/a/347563/24203
## Get rosversion of the same packages
res_dpkg=$(dpkg -l | grep '^ii' | grep ros- | awk '{print $2 " " $3}')
IFS=$'\n'  # http://askubuntu.com/questions/344407/how-to-read-complete-line-in-for-loop-with-spaces
for i in ${res_dpkg}
do 
  printf "%s $i "
  p_underscore=$(echo ${i} | awk '{print $1}' | sed 's/ros-\([a-zA-Z]*\)-//' | tr '-' '_');
  printf "%s ${p_underscore} "
  rospackfind_result=$(rospack -q find ${p_underscore})
  printf "%s ${rospackfind_result} "
  echo "${rospackfind_result}" | grep -o '[^/]*$' | xargs rosversion
done

tar -C ~/.ros/log -cvzf ${FILENAME_LOG_ROS} `cd ~/.ros/log; ls -d * | head -1`

tar cfz ${FILENAME_LOG_ALL} ${FILENAME_LOG_COMMANDS} ${FILENAME_LOG_ROS}

echo "=== All diagnostic info recorded into a tarball: ${FILENAME_LOG_ALL}"
echo "=== Ask at rtm-ros-robotics@googlegroups.com"
