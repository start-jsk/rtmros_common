macro(download_pr2_controllers_msgs branch)
  execute_process(COMMAND cmake -E remove_directory /tmp/pr2_controllers)
  set(ENV{GIT_SSH} 'ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no $*')
  execute_process(
    COMMAND git clone git://github.com/PR2/pr2_controllers /tmp/pr2_controllers
    OUTPUT_VARIABLE _download_output
    RESULT_VARIABLE _download_failed)
  execute_process(
    COMMAND git checkout ${branch}
    WORKING_DIRECTORY /tmp/pr2_controllers)
  execute_process(
    COMMAND cmake -E copy_directory  /tmp/pr2_controllers/pr2_controllers_msgs ${PROJECT_SOURCE_DIR}/../pr2_controllers_msgs)
  message("download pr2_controllers_msgs files ${_download_output}")
  if (_download_failed)
    message(FATAL_ERROR "Download pr2_controllers_msgs failed : ${_download_failed}")
  endif(_download_failed)
  # rospack profile
  execute_process(COMMAND rospack profile OUTPUT_VARIABLE _profile_output RESULT_VARIABLE _profile_failed)
  message("rospack profile ${_profile_output}")
  if (_profile_failed)
    message(FATAL_ERROR "rospack profile failed : ${_profile_failed}")
  endif(_profile_failed)
endmacro()

if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(rtmbuild)
include(${rtmbuild_PACKAGE_PATH}/cmake/rtmbuild.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/compile_robot_model.cmake)

# if groovy and not found pr2_controllers_msgs
rosbuild_find_ros_package(pr2_controllers_msgs)
if(NOT EXISTS "${pr2_controllers_msgs_PACKAGE_PATH}")
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/../pr2_controllers_msgs")
    download_pr2_controllers_msgs(groovy-devel)
    # rosmake pr2_controllers_msgs
    execute_process(COMMAND rosmake pr2_controllers_msgs OUTPUT_VARIABLE _compile_output RESULT_VARIABLE _compile_failed)
    message("compile pr2_controllers_msgs ${_compile_output}")
    if (_compile_failed)
      message(FATAL_ERROR "compile pr2_controllers_msgs failed : ${_compile_failed}")
    endif(_compile_failed)
  endif()
endif()

## <depend package="pr2_controllers_msgs"/> in manifest.xml is initially commented out
## since rosmake -a seems to check manifest.xml and if some of depend is not found, 
## it does not create dependency graph. To avoid this, we commented out.
## However when make hrpsys_ros_bridge component, it uses pr2_controller_msgs,
## so we need to comment in, in compilation time.
message("sed -i s@'<!--\\s*\\(.*pr2_controllers.*/\\)\\s*-->'@'<\\1>'@g ${PROJECT_SOURCE_DIR}/manifest.xml")
execute_process(
  COMMAND sh -c "grep '<depend package=\"pr2_controllers_msgs\"/>' ${PROJECT_SOURCE_DIR}/manifest.xml || sed -i s@'<!--\\s*\\(.*pr2_controllers.*/\\)\\s*-->'@'<\\1>'@g ${PROJECT_SOURCE_DIR}/manifest.xml"
  )


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(hrpsys)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
file(GLOB _hrpsys_idl_files RELATIVE ${hrpsys_PACKAGE_PATH}/share/hrpsys/idl/ ${hrpsys_PACKAGE_PATH}/share/hrpsys/idl/*.idl)
foreach(_hrpsys_idl_file ${_hrpsys_idl_files})
  if(${hrpsys_PACKAGE_PATH}/share/hrpsys/idl/${_hrpsys_idl_file} IS_NEWER_THAN ${PROJECT_SOURCE_DIR}/idl/${_hrpsys_idl_file})
    execute_process(COMMAND cmake -E copy ${hrpsys_PACKAGE_PATH}/share/hrpsys/idl/${_hrpsys_idl_file} ${PROJECT_SOURCE_DIR}/idl)
  endif()
endforeach()

rtmbuild_init()
#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
rtmbuild_genidl()

rtmbuild_genbridge()


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

##
## hrpsys ros bridge tools
##
rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)
rtmbuild_add_executable(HrpsysJointTrajectoryBridge src/HrpsysJointTrajectoryBridge.cpp src/HrpsysJointTrajectoryBridgeComp.cpp)

##
## test (Copy from catkin.cmake)
##
rosbuild_find_ros_package(openhrp3)
set(ENV{PKG_CONFIG_DIR} ${oepnhrp3_PACKAGE_PATH}/lib/pkgconfig:$ENV{PKG_CONFIG_DIR})
execute_process(COMMAND pkg-config openhrp3.1 --variable=idl_dir
  OUTPUT_VARIABLE _OPENHRP3_IDL_DIR
  RESULT_VARIABLE _OPENHRP3_RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(_OPENHRP3_MODEL_DIR ${_OPENHRP3_IDL_DIR}/../sample/model)
if(NOT _OPENHRP3_RESULT EQUAL 0)
  message(FATAL_ERROR "Fail to run pkg-config ${_OPENHRP3_RESULT}")
endif()
if(NOT EXISTS ${_OPENHRP3_IDL_DIR})
  message(FATAL_ERROR "Path ${_OPENHRP3_IDL_DIR} is not exists")
endif()
if(NOT EXISTS ${_OPENHRP3_MODEL_DIR})
  message(FATAL_ERROR "Path ${_OPENHRP3_MODEL_DIR} is not exists")
endif()

compile_openhrp_model(${_OPENHRP3_MODEL_DIR}/PA10/pa10.main.wrl)
compile_openhrp_model(${_OPENHRP3_MODEL_DIR}/sample1.wrl SampleRobot)
generate_default_launch_eusinterface_files(${_OPENHRP3_MODEL_DIR}/PA10/pa10.main.wrl hrpsys_ros_bridge)
generate_default_launch_eusinterface_files(${_OPENHRP3_MODEL_DIR}/sample1.wrl hrpsys_ros_bridge SampleRobot)
execute_process(COMMAND sed -i s@pa10\(Robot\)0@HRP1\(Robot\)0@ ${PROJECT_SOURCE_DIR}/launch/pa10.launch)
execute_process(COMMAND sed -i s@pa10\(Robot\)0@HRP1\(Robot\)0@ ${PROJECT_SOURCE_DIR}/launch/pa10_startup.launch)
execute_process(COMMAND sed -i s@pa10\(Robot\)0@HRP1\(Robot\)0@ ${PROJECT_SOURCE_DIR}/launch/pa10_ros_bridge.launch)
file(WRITE models/SampleRobot_controller_config.yaml
"controller_configuration:
  - group_name: rarm
    controller_name: /rarm_controller
    joint_list:
      - RARM_SHOULDER_P
      - RARM_SHOULDER_R
      - RARM_SHOULDER_Y
      - RARM_ELBOW
      - RARM_WRIST_Y
      - RARM_WRIST_P
  - group_name: larm
    controller_name: /larm_controller
    joint_list:
      - LARM_SHOULDER_P
      - LARM_SHOULDER_R
      - LARM_SHOULDER_Y
      - LARM_ELBOW
      - LARM_WRIST_Y
      - LARM_WRIST_P
  - group_name: torso
    controller_name: /torso_controller
    joint_list:
      - WAIST_P
      - WAIST_R
      - CHEST
  - group_name: rhand
    controller_name: /rhand_controller
    joint_list:
      - RARM_WRIST_R
  - group_name: lhand
    controller_name: /lhand_controller
    joint_list:
      - LARM_WRIST_R
")


rosbuild_add_rostest(test/test-samplerobot.test)
rosbuild_add_rostest(test/test-pa10.test)
rosbuild_add_rostest(test/test-import-python.test)
