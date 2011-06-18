cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(openhrp3)
rosbuild_find_ros_package(hrpsys)

configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)
configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.conf)
