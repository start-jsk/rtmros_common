cmake_minimum_required(VERSION 2.8.3)
project(rtmbuild_test)

if(NOT USE_ROSBUILD) ## if catkin
  # call catkin depends
  find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp)
else()
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_find_ros_package(rtmbuild)
  include(${rtmbuild_PACKAGE_PATH}/cmake/rtmbuild.cmake)
endif()

# initialize rtmbuild
rtmbuild_init()

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

