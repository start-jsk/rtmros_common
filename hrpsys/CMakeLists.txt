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

rtmbuild_init()

# Build OpenHRP3
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.hrpsys-base
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of hrpsys failed")
endif(_make_failed)


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

execute_process(COMMAND rosrun hrpsys hrp4c_model_download.sh RESULT_VARIABLE _download_failed)
if (_download_failed)
  message(FATAL_ERROR "Download HRP-4C model from http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html")
endif(_download_failed)
execute_process(COMMAND rosrun openhrp3 export-collada -i ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl -o ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4C.dae)
execute_process(COMMAND rosrun euscollada collada2eus ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4C.dae ${PROJECT_SOURCE_DIR}/launch/hrp4c.yaml ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/hrp4c.l)

