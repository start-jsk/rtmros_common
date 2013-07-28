cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(hrpsys_ros_bridge)
include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_download_data(https://github.com/rdiankov/collada_robots/raw/master/kawada-hironx.zae models/kawada-hironx.zae be4b0015914d33a5aaa24ee055bcdbc8)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae
  COMMAND unzip -u ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/kawada-hironx.zae)
compile_collada_model(${PROJECT_SOURCE_DIR}/models/kawada-hironx.dae)
#rosbuild_find_ros_package(collada_robots)
#compile_collada_model(${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae)
##
## tests
##
rosbuild_add_rostest(test/hironx-test.launch)

