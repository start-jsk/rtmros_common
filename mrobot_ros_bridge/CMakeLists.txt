cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(rtmbuild)
include(${rtmbuild_PACKAGE_PATH}/cmake/rtmbuild.cmake)

rtmbuild_init()
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rtmbuild_genidl()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rtmbuild_add_executable(IISMobileRobotControllerComp src/IISMobileRobotControllerComp.cpp src/IISMobileRobotController.cpp)
rtmbuild_add_executable(IISBeegoControllerComp src/IISBeegoControllerComp.cpp src/IISBeegoController.cpp)
rtmbuild_add_executable(MobileRobotROSBridgeComp src/MobileRobotROSBridge.cpp src/MobileRobotROSBridgeComp.cpp)
rtmbuild_add_executable(RangeSensorROSBridgeComp src/RangeSensorROSBridgeComp.cpp src/RangeSensorROSBridge.cpp)

## documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)
# execute_process(COMMAND rosrun rosdoc rosdoc ${PROJECT_NAME})
