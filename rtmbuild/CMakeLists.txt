cmake_minimum_required(VERSION 2.8.3)
project(rtmbuild)

if(NOT USE_ROSBUILD) ## if catkin

  find_package(catkin REQUIRED COMPONENTS rostest)

  find_package(PkgConfig)
  pkg_check_modules(OMNIORB REQUIRED omniORB4)
  if (!OMNIORB_FOUND)
    message(FATAL_ERROR  "OmniORB4 is not found")
  endif()

  catkin_package(
    DEPENDS omniorb
    CATKIN_DEPENDS message_generation std_msgs
    CFG_EXTRAS servicebridge.cmake rtmbuild.cmake
    )

  install(DIRECTORY cmake scripts test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

  add_rostest(test/test-compile-idl.test)

else()
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  rosbuild_add_rostest(test/test-compile-idl.test)

endif()

