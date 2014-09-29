# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge)

# call catkin depends
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp rostest sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager hrpsys_tools image_transport dynamic_reconfigure hrpsys nav_msgs) # pr2_controllers_msgs robot_monitor
catkin_python_setup()

# download pr2_controllers_msgs for git
find_package(pr2_controllers_msgs QUIET)
if(NOT pr2_controllers_msgs_FOUND)
  download_pr2_controllers_msgs(hydro-devel) # even for Groovy, download hydro-devel that's wet.
  # catkin_make
  # rosmake pr2_controllers_msgs
  execute_process(COMMAND cmake -E chdir ${CMAKE_SOURCE_DIR}/../ catkin_make -C ${CMAKE_SOURCE_DIR}/../ --build /tmp/pr2_controllers --source ${PROJECT_SOURCE_DIR}/../pr2_controllers_msgs --pkg pr2_controllers_msgs OUTPUT_VARIABLE _compile_output RESULT_VARIABLE _compile_failed)
  message("compile pr2_controllers_msgs ${_compile_output}")
  if (_compile_failed)
    message(FATAL_ERROR "compile pr2_controllers_msgs failed : ${_compile_failed}")
  endif(_compile_failed)
 include_directories(${CMAKE_SOURCE_DIR}/../devel/include)
endif()

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
set(ENV{PKG_CONFIG_PATH} ${hrpsys_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}) #update PKG_CONFIG_PATH for pkg-config
execute_process(COMMAND pkg-config --variable=idldir hrpsys-base
  OUTPUT_VARIABLE hrpsys_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT RESULT EQUAL 0)
  execute_process(COMMAND "pkg-config" "--list-all")
  execute_process(COMMAND "env")
  message(FATAL_ERROR "Fail to run pkg-config ${RESULT}")
endif()
if(EXISTS ${hrpsys_IDL_DIR})
  file(GLOB _hrpsys_idl_files RELATIVE ${hrpsys_IDL_DIR}/ ${hrpsys_IDL_DIR}/*.idl)
  foreach(_hrpsys_idl_file ${_hrpsys_idl_files})
    if(${hrpsys_IDL_DIR}/${_hrpsys_idl_file} IS_NEWER_THAN ${PROJECT_SOURCE_DIR}/idl/${_hrpsys_idl_file})
      execute_process(COMMAND cmake -E copy ${hrpsys_IDL_DIR}/${_hrpsys_idl_file} ${PROJECT_SOURCE_DIR}/idl)
    endif()
  endforeach()
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
    CATKIN_DEPENDS rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure nav_msgs # pr2_controllers_msgs robot_monitor
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

rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)
rtmbuild_add_executable(HrpsysJointTrajectoryBridge src/HrpsysJointTrajectoryBridge.cpp src/HrpsysJointTrajectoryBridgeComp.cpp)

install(PROGRAMS scripts/rtmlaunch scripts/rtmtest scripts/rtmstart.py
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY launch euslisp srv idl scripts models test cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

## fix generated path for model file
install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DISTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/models/*.*\")
   execute_process(COMMAND echo \"                   ${PROJECT_SOURCE_DIR} -> ${CMAKE_INSTALL_PREFIX}\")
   if (EXISTS ${openhrp3_SOURCE_DIR})
     execute_process(COMMAND echo \"                  ${openhrp3_SOURCE_DIR} -> ${openhrp3_PREFIX}/share/openhrp3\")
   endif()
   execute_process(COMMAND echo \"                  ${CATKIN_DEVEL_PREFIX} -> ${CMAKE_INSTALL_PREFIX}\")
   file(GLOB _conf_files \"\$ENV{DISTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/models/*.*\")
   foreach(_conf_file \${_conf_files})
     execute_process(COMMAND echo \"fix \${_conf_file}\")
     if (EXISTS ${openhrp3_SOURCE_DIR})
       execute_process(COMMAND sed -i s@${openhrp3_SOURCE_DIR}/share/OpenHRP-3.1@${CMAKE_INSTALL_PREFIX}/share/openhrp3/share/OpenHRP-3.1@g \${_conf_file})
     endif()
     execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \${_conf_file})
   endforeach()
  ")

install(FILES rqt_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##
## test (Copy from CMakeLists.txt)
##

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
compile_openhrp_model(${_OPENHRP3_MODEL_DIR}/sample1.wrl SampleRobot
  --conf-dt-option "0.002"
  --simulation-timestep-option "0.002")
generate_default_launch_eusinterface_files("\$(find openhrp3)/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl" hrpsys_ros_bridge)
generate_default_launch_eusinterface_files("\$(find openhrp3)/share/OpenHRP-3.1/sample/model/sample1.wrl" hrpsys_ros_bridge SampleRobot)
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

add_rostest(test/test-samplerobot.test)
add_rostest(test/test-pa10.test)
add_rostest(test/test-import-python.test)

