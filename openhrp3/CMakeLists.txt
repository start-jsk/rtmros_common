cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Build OpenHRP3
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openhrp3 installed
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenHRP3 failed")
endif(_make_failed)


