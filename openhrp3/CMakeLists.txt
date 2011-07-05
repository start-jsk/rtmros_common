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

# Build OpenHRP3
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openhrp-aist-grx
		RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenHRP3 failed")
endif(_make_failed)

#rosbuild_init()
rtmbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(OPENHRP_PRIVATE_LIBRARIES `rospack find openhrp3`/build/openhrp-aist-grx/lib/libhrpCorbaStubSkel-3.1.a)
rtmbuild_genidl()

#rtmbuild_add_executable(JoystickController share/OpenHRP-3.1/sample/JoystickControl/Controller/JoystickControllerComp.cpp share/OpenHRP-3.1/sample/JoystickControl/Controller/JoystickController.cpp)

