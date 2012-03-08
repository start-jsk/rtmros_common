cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(rtmbuild)
include(${rtmbuild_PACKAGE_PATH}/cmake/rtmbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Build hrpsys before rtmbuild_init
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of hrpsys failed")
endif(_make_failed)

rtmbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rtmbuild_genidl()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

##
## tools
##
rtmbuild_add_executable(AbsTransformToPosRpy src/AbsTransformToPosRpy.cpp src/AbsTransformToPosRpyComp.cpp)

rtmbuild_genbridge()

#macro(rtmbridge interface)
#  rtmbuild_add_executable(${interface}Comp src/${interface}.cpp src/${interface}Comp.cpp)
#endmacro(rtmbridge)
#rtmbridge(CameraCaptureServiceROSBridge)
#rtmbridge(DataLoggerServiceROSBridge) # OK, but name space is collide
#rtmbridge(ExecutionProfileServiceROSBridge)
#rtmbridge(ForwardKinematicsServiceROSBridge) # error -> array
#rtmbridge(NullServiceROSBridge)
#rtmbridge(OGMap3DServiceROSBridge)
#rtmbridge(RobotHardwareServiceROSBridge) # error -> multi dimensional seq
#rtmbridge(SequencePlayerServiceROSBridge) # error -> multi dimensional seq
#rtmbridge(StateHolderServiceROSBridge)
#rtmbridge(WavPlayerServiceROSBridge)

##
## download models
##
execute_process(COMMAND rosrun hrpsys hrp4c_model_download.sh RESULT_VARIABLE _download_failed)
if (_download_failed)
  message(STATUS "Download HRP-4C model from http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html")
endif(_download_failed)

##
## tests
##
rosbuild_check_for_display(disp)
if(disp)
  rosbuild_add_rostest(launch/hrp4c.launch)
  #rosbuild_add_rostest(launch/pa10.launch)
  rosbuild_add_rostest(launch/samplerobot.launch)
endif(disp)

##
## documents
##
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)

