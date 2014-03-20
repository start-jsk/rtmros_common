#!/bin/bash

CATKIN_DIR=/tmp/test_compile_robot_$$
rm -fr ${CATKIN_DIR}
mkdir -p ${CATKIN_DIR}/src/hrpsys_ros_bridge_test
cp `rospack find hrpsys_ros_bridge`/test/test-compile-robot.cmake ${CATKIN_DIR}/src/hrpsys_ros_bridge_test/CMakeLists.txt
cp `rospack find hrpsys_ros_bridge`/test/test-compile-robot.xml   ${CATKIN_DIR}/src/hrpsys_ros_bridge_test/package.xml
# copy hrpsys_tools
ln -sf `rospack find hrpsys_tools` ${CATKIN_DIR}/src/hrpsys_tools
cd ${CATKIN_DIR}
catkin_make
source ${CATKIN_DIR}/devel/setup.bash
#yes | rosrun openrtm_aist rtm-naming
#roslaunch hrpsys_ros_bridge_test samplerobot.launch corbaport:=2809 GUI:=false RUN_RVIZ:=false













