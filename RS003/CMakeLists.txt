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

rtmbuild_add_executable(MotorControlComp src/MotorControl/MotorControl.cpp src/MotorControl/MotorControlComp.cpp src/MotorControl/MotorControlProSVC_impl.cpp)
rtmbuild_add_library(MotorControl src/MotorControl/MotorControl.cpp src/MotorControl/MotorControlProSVC_impl.cpp)
#
rtmbuild_add_executable(CeilingNavigationComp src/CeilingNavigation/CeilingNavigation.cpp src/CeilingNavigation/CeilingNavigationComp.cpp src/CeilingNavigation/CeilingMap.cpp src/CeilingNavigation/BlockMatching.cpp src/CeilingNavigation/ImageData.cpp src/Common/sharedmem.c)
rosbuild_add_compile_flags(CeilingNavigationComp "-I${PROJECT_SOURCE_DIR}/src -D_USE_SHOW_MAP")
#
rtmbuild_add_executable(DispPositionComp src/DispPosition/DispPosition.cpp src/DispPosition/DispPositionComp.cpp)
rosbuild_add_compile_flags(DispPositionComp '-DORG_MAP="${PROJECT_SOURCE_DIR}/share/image/RTCcenterUnifiedMap.jpg"')
#
rtmbuild_add_executable(OdometryComp src/Odometry/Odometry.cpp src/Odometry/OdometryComp.cpp)
#
rtmbuild_add_executable(GamePadComp src/GamePad/GamePad.cpp src/GamePad/GamePadComp.cpp src/GamePad/pad_linux.cpp)
target_link_libraries(GamePadComp SDL)
#
rtmbuild_add_executable(LocalizeCenterComp src/LocalizeCenter/LocalizeCenter.cpp src/LocalizeCenter/LocalizeCenterComp.cpp)
#
rtmbuild_add_executable(PathPlanningComp src/PathPlanning/PathPlanning.cpp src/PathPlanning/PathPlanningComp.cpp src/PathPlanning/planner.cpp)
#compile PathPlanningParam.conf
configure_file(${PROJECT_SOURCE_DIR}/src/PathPlanning/PathPlanningParam.conf.in ${PROJECT_SOURCE_DIR}/src/PathPlanning/PathPlanningParam.conf)
add_custom_target(conf ALL DEPENDS ${PROJECT_SOURCE_DIR}/src/PathPlanning/PathPlanningParam.conf)
#
rtmbuild_add_executable(PositionInputComp src/PositionInput/PositionInput.cpp src/PositionInput/PositionInputComp.cpp)
#
rtmbuild_add_executable(SimRangeUrgComp src/SimRangeUrg/SimRangeUrg.cpp src/SimRangeUrg/SimRangeUrgComp.cpp)
#
rtmbuild_add_executable(SimViewCamComp src/SimViewCam/SimViewCam.cpp src/SimViewCam/SimViewCamComp.cpp)
#
rtmbuild_add_executable(show_map src/show_map/show_map.c src/Common/sharedmem.c)
rosbuild_add_compile_flags(show_map '-DMAP_SHOW="${PROJECT_SOURCE_DIR}/share/image/rotRTCcenterCeiling.JPG"')
#
rtmbuild_add_executable(NavigationComp src/Navigation/Navigation.cpp src/Navigation/NavigationComp.cpp)
#
rtmbuild_add_executable(PathFollowerComp src/PathFollower/PathFollower.cpp src/PathFollower/PathFollowerComp.cpp src/PathFollower/control.cpp)
#
rtmbuild_add_executable(SwitchInputComp src/SwitchInput/SwitchInputRTC.cpp src/SwitchInput/SwitchInputRTCComp.cpp)
#
rtmbuild_add_executable(Tmvel2D2TmvelComp src/Tmvel2D2Tmvel/Tmvel2D2Tmvel.cpp src/Tmvel2D2Tmvel/Tmvel2D2TmvelComp.cpp)

