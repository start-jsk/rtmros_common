# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openrtm_ros_bridge)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.

find_package(catkin REQUIRED COMPONENTS roscpp openrtm_aist openrtm_tools rtmbuild)
# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})if(NOT USE_ROSBUILD)

# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(PkgConfig)
pkg_check_modules(rtmbuild REQUIRED rtmbuild)
include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

pkg_check_modules(openrtm_aist REQUIRED openrtm_aist)
execute_process(COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/idl)
file(GLOB_RECURSE _idl_files "${openrtm_aist_SOURCE_DIR}/share/openrtm-1.1/example/*.idl")
foreach(_idl_file ${_idl_files})
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_idl_file} ${PROJECT_SOURCE_DIR}/idl/)
endforeach()

#
# CATKIN_MIGRATION: removed during catkin migration
#  rosbuild_init()
rtmbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
#  FILES
  # TODO: List your msg files here
#)
#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)
rtmbuild_genidl()

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rtmbuild_genbridge()
##
## clean
##
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/bin;${CMAKE_SOURCE_DIR}/src;${CMAKE_SOURCE_DIR}/idl;${CMAKE_SOURCE_DIR}/srv;${CMAKE_SOURCE_DIR}/idl_gen;${CMAKE_SOURCE_DIR}/srv_gen;${CMAKE_SOURCE_DIR}/msg_gen;${CMAKE_SOURCE_DIR}/src_gen;${CMAKE_SOURCE_DIR}/msg/SimpleService_MyService.msg")


##
## tests
##

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(samples/test_myservice_rosbridge.launch)

## Generate added messages and services with any dependencies listed here
#generate_messages(
  #TODO DEPENDENCIES geometry_msgs std_msgs
#)
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS #
    CATKIN-DEPENDS # openrtm_tools rtmbuild roscpp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
