# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge)

# call catkin depends
find_package(catkin REQUIRED COMPONENTS hrpsys rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure ) # pr2_controllers_msgs robot_monitor

# include rtmbuild
#include(${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
if(EXISTS ${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
  include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
elseif(EXISTS ${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
  include(${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
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
if(EXISTS ${hrpsys_SOURCE_DIR}/build/hrpsys-base-source/idl/)
  file(COPY
    ${hrpsys_SOURCE_DIR}/build/hrpsys-base-source/idl/
    DESTINATION ${PROJECT_SOURCE_DIR}/idl)
elseif(EXISTS ${hrpsys_PREFIX}/share/hrpsys/share/hrpsys/idl/)
  file(COPY
    ${hrpsys_PREFIX}/share/hrpsys/share/hrpsys/idl/
    DESTINATION ${PROJECT_SOURCE_DIR}/idl)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "hrpsys/idl is not found")
endif()

# define add_message_files before rtmbuild_init
add_message_files(FILES MotorStates.msg)

# initialize rtmbuild
rtmbuild_init()

# call catkin_package, after rtmbuild_init, before rtmbuild_gen*
catkin_package(
    DEPENDS # TODO
    CATKIN-DEPENDS hrpsys rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure # pr2_controllers_msgs robot_monitor
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

##
## hrpsys ros bridge tools
##
# pr2_controller_msgs is not catkinized
include_directories(/opt/ros/groovy/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include)

rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)
rtmbuild_add_executable(HrpsysJointTrajectoryBridge src/HrpsysJointTrajectoryBridge.cpp src/HrpsysJointTrajectoryBridgeComp.cpp)

if(TARGET compile_hrpsys)
  add_dependencies(HrpsysSeqStateROSBridge compile_hrpsys)
  add_dependencies(ImageSensorROSBridge compile_hrpsys)
  add_dependencies(HrpsysJointTrajectoryBridge compile_hrpsys)
endif()

##
## test
##

