cmake_minimum_required(VERSION 2.4.6)
project(hrpsys_ros_bridge_test)

# for rosbuild
message("USE_ROSBUILD = ${USE_ROSBUILD}")
if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_find_ros_package(hrpsys_ros_bridge)
  include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)
else() # catkin
  find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys_tools)
endif()

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

    find_package(hrpsys_tools)
    message("hrpsys_ros_bridge_SOURCE_DIR ---> ${hrpsys_ros_bridge_SOURCE_DIR}")
    message("     hrpsys_tools_SOURCE_DIR ---> ${hrpsys_tools_SOURCE_DIR}")
    message("         hrpsys_tools_PREFIX ---> ${hrpsys_tools_PREFIX}")

message("[hrpsys_ros_bridge/test] Path ${_OPENHRP3_MODEL_DIR} exists")

# macro(compile_openhrp_model wrlfile) # compile openhrp3 model to collada and project file
compile_openhrp_model(${_OPENHRP3_MODEL_DIR}/sample1.wrl SampleRobot)
generate_default_launch_eusinterface_files("\$(find hrpsys_ros_bridge_test)/models/SampleRobot.dae" hrpsys_ros_bridge SampleRobot)


# macro(compile_collada_model daefile) # compile collada model project file
