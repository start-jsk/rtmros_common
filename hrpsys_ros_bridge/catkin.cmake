# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge)

# call catkin depends
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure ) # pr2_controllers_msgs robot_monitor
catkin_python_setup()
# include rtmbuild
#include(${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
if(EXISTS ${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
  include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
elseif(EXISTS ${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
  include(${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endif()
# include compile_robot_model.cmake
include(${PROJECT_SOURCE_DIR}/cmake/compile_robot_model.cmake)

# copy idl files from hrpsys
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
set(hrpsys_IDL_DIR ${hrpsys_PREFIX}/share/hrpsys/share/hrpsys/idl/)
if(EXISTS ${hrpsys_IDL_DIR})
  file(COPY
    ${hrpsys_IDL_DIR}
    DESTINATION ${PROJECT_SOURCE_DIR}/idl)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "${hrpsys_IDL_DIR} is not found")
endif()

unset(hrpsys_LIBRARIES CACHE) # remove not to add hrpsys_LIBRARIES to hrpsys_ros_bridgeConfig.cmake

# define add_message_files before rtmbuild_init
add_message_files(FILES MotorStates.msg)

# initialize rtmbuild
rtmbuild_init()

# call catkin_package, after rtmbuild_init, before rtmbuild_gen*
catkin_package(
    DEPENDS hrpsys # TODO
    CATKIN-DEPENDS rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure # pr2_controllers_msgs robot_monitor
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
    CFG_EXTRAS compile_robot_model.cmake
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

##
## hrpsys ros bridge tools
##
# pr2_controller_msgs is not catkinized
string(RANDOM _random_string)

# Check ROS distro. since pr2_controller_msgs of groovy is not catkinized
if($ENV{ROS_ROOT} MATCHES "/opt/ros/groovy/share/ros")

message("sed -i s@'<\\(.*_depend\\)>pr2_controllers</\\(.*_depend\\)>'@'<!-- \\1>pr2_controllers</\\2 -->'@g ${PROJECT_SOURCE_DIR}/package.xml")
execute_process(
  COMMAND sh -c "sed -i s@'<\\(.*_depend\\)>pr2_controllers</\\(.*_depend\\)>'@'<!-- \\1>pr2_controllers</\\2 -->'@g ${PROJECT_SOURCE_DIR}/package.xml"
  )

execute_process(
  COMMAND svn co --non-interactive --trust-server-cert https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/tags/groovy/pr2_controllers_msgs /tmp/${_random_string}/pr2_controllers_msgs
  OUTPUT_VARIABLE _download_output
  RESULT_VARIABLE _download_failed)
message("download pr2_controllers_msgs files ${_download_output}")
if (_download_failed)
  message(FATAL_ERROR "Download pr2_controllers_msgs failed : ${_download_failed}")
endif(_download_failed)
file(WRITE /tmp/${_random_string}/rospack
"\#!/bin/sh
echo $@ 1>&2
if [ \"$1\"  = \"deps-manifests\" ];then
   echo \"/opt/ros/groovy/share/genmsg/package.xml /opt/ros/groovy/share/gencpp/package.xml /opt/ros/groovy/share/genlisp/package.xml /opt/ros/groovy/share/genpy/package.xml /opt/ros/groovy/share/message_generation/package.xml /opt/ros/groovy/share/cpp_common/package.xml /opt/ros/groovy/share/rostime/package.xml /opt/ros/groovy/share/roscpp_traits/package.xml /opt/ros/groovy/share/roscpp_serialization/package.xml /opt/ros/groovy/share/message_runtime/package.xml /opt/ros/groovy/share/std_msgs/package.xml /opt/ros/groovy/share/actionlib_msgs/package.xml /opt/ros/groovy/share/trajectory_msgs/package.xml /opt/ros/groovy/share/geometry_msgs/package.xml\"
elif [ \"$1\"  = \"deps-msgsrv\" ];then
   true
elif [ \"$1\"  = \"cflags-only-I\" ];then
   echo \"/tmp/${_random_string}/pr2_controllers_msgs/msg_gen/cpp/include /tmp/${_random_string}/pr2_controllers_msgs/srv_gen/cpp/include /opt/ros/groovy/include\"
elif [ \"$1\"  = \"cflags-only-other\" ];then
   true
elif [ \"$1\"  = \"libs-only-L\" ];then
   echo \"/opt/ros/groovy/lib\"
elif [ \"$1\"  = \"libs-only-l\" ];then
   echo \"roscpp_serialization rostime :/usr/lib/libboost_date_time-mt.so :/usr/lib/libboost_system-mt.so :/usr/lib/libboost_thread-mt.so pthread cpp_common\"
elif [ \"$1\"  = \"libs-only-other\" ];then
   true
elif [ \"$1\"  = \"langs\" ];then
   true
else
   /opt/ros/groovy/bin/rospack $@
fi
")
execute_process(
  COMMAND sh -c "chmod u+x /tmp/${_random_string}/rospack"
  COMMAND sh -c "touch /tmp/${_random_string}/rosdep; chmod u+x /tmp/${_random_string}/rosdep"
  COMMAND sh -c "PATH=/tmp/${_random_string}:$PATH ROS_PACKAGE_PATH=/tmp/${_random_string}/pr2_controllers_msgs:$ROS_PACKAGE_PATH make -C /tmp/${_random_string}/pr2_controllers_msgs"
  OUTPUT_VARIABLE _compile_output
  RESULT_VARIABLE _compile_failed)
message("Compile pr2_controllers_msgs files ${_compile_output}")
if (_compile_failed)
  message(FATAL_ERROR "Compile pr2_controllers_msgs failed : ${_compile_failed}")
endif(_compile_failed)

include_directories(/tmp/${_random_string}/pr2_controllers_msgs/msg_gen/cpp/include)

endif($ENV{ROS_ROOT} MATCHES "/opt/ros/groovy/share/ros")

rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)
rtmbuild_add_executable(HrpsysJointTrajectoryBridge src/HrpsysJointTrajectoryBridge.cpp src/HrpsysJointTrajectoryBridgeComp.cpp)

install(PROGRAMS scripts/rtmlaunch scripts/rtmtest scripts/rtmstart.py
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
#  PATTERN "rtmlaunch" EXCLUDE   copy rtmlaunch to both share and bin for backword compatibility
#  PATTERN "rtmtest" EXCLUDE
#  PATTERN "rtmstart.py" EXCLUDE
  )

install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )

  

##
## test
##

