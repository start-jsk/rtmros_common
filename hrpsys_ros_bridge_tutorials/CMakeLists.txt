cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(hrpsys_ros_bridge)
include(${hrpsys_ros_bridge_PACKAGE_PATH}/cmake/compile_robot_model.cmake)

rosbuild_init()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined services
#rosbuild_gensrv()
#rtmbuild_genidl()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

##
## wrl,dae models
##
set(wrl_robots)
rosbuild_find_ros_package(openhrp3)
rosbuild_find_ros_package(hrpsys)
rosbuild_find_ros_package(collada_robots)
rosbuild_find_ros_package(choreonoid)
rosbuild_find_ros_package(webots_simulator)

###
### convert
###
if(EXISTS ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl)
  compile_openhrp_model(
    ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl
    HRP4C
    -a rightarm_torso,BODY,R_WRIST_R_LINK,0,0,0,0.707,0,0.707,0 -a leftarm_torso,BODY,L_WRIST_R_LINK,0,0,0,0.707,0,0.707,0 -a rightarm,BODY,CHEST_Y_LINK,0,0,0,0.707,0,0.707,0 -a leftarm,CHEST_Y_LINK,L_WRIST_R_LINK,0,0,0,0.707,0,0.707,0
    --conf-file-option "virtual_force_sensor: vlhsensor, CHEST_Y, L_HAND_J0, 0,0,0, 0,0,1,0, vrhsensor, CHEST_Y, R_HAND_J0, 0,0,0, 0,0,1,0"
    --conf-file-option "abc_leg_offset: 0.0, 0.06845, 0.0"
    --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
)
endif(EXISTS ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl)

compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/sample1.wrl SampleRobot
  --conf-file-option "abc_leg_offset: 0,0.09,0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10")

compile_collada_model(
  ${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae
  --proj-file-root-option "0,0,0.5,0,0,1,0"
  )
compile_collada_model(${collada_robots_PACKAGE_PATH}/data/robots/willowgarage-pr2.dae)

# generate webots model
add_custom_command(OUTPUT ${webots_simulator_PACKAGE_PATH}/build/webots/resources/projects/robots/darwin-op/protos/DARwInOP.proto
  COMMAND make -C ${webots_simulator_PACKAGE_PATH})
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/DARWIN.wrl
  COMMAND rosrun roseus roseus ${PROJECT_SOURCE_DIR}/euslisp/webotsvrml2openhrp3vrml.l
  "\\(progn \\(convert-webots-vrml-\\>openhrp3-vrml-for-darwin \\\"${PROJECT_SOURCE_DIR}/models/DARWIN.wrl\\\"\\) \\(exit\\)\\)"
  DEPENDS ${webots_simulator_PACKAGE_PATH}/build/webots/resources/projects/robots/darwin-op/protos/DARwInOP.proto)
if (EXISTS ${webots_simulator_PACKAGE_PATH}/build/webots/resources/projects/robots/darwin-op/protos/DARwInOP.proto)
  compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/DARWIN.wrl)
else()
  message("\n\n\n\n${webots_simulator_PACKAGE_PATH}/build/webots/resources/projects/robots/darwin-op/protos/DARwInOP.proto is not found..\n\n\n\n")
endif()

# yaskawa model
rosbuild_download_data(http://openhrp-aist-grx.googlecode.com/files/YaskawaSmartPal5.zip build/YaskawaSmartPal5.zip 3db916d5dc5ef4259c7dfc1ac85dd380)
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/models/YaskawaSmartPal5.wrl
  COMMAND unzip -o ${PROJECT_SOURCE_DIR}/build/YaskawaSmartPal5.zip
  COMMAND cp    -rf ${PROJECT_SOURCE_DIR}/build/YaskawaSmartPal5/* ${PROJECT_SOURCE_DIR}/models/
  DEPENDS ${PROJECT_SOURCE_DIR}/build/YaskawaSmartPal5.zip)
compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/YaskawaSmartPal5.wrl
  YaskawaSmartPal5
  -a leftarm,WAIST_LINK2,LARM_LINK7,0,0,0,0.707,0,0.707,0,LARM_JOINTG,-1 -a leftarm_waist,WAIST_LINK0,LARM_LINK7,0,0,0,0.707,0,0.707,0,LARM_JOINTG,-1 -a rightarm,WAIST_LINK2,RARM_LINK7,0,0,0,0.707,0,0.707,0,RARM_JOINTG,-1 -a rightarm_waist,WAIST_LINK0,RARM_LINK7,0,0,0,0.707,0,0.707,0,RARM_JOINTG,-1)

# choreonoid model
if (EXISTS ${choreonoid_PACKAGE_PATH}/share/choreonoid-1.3/model/GR001/GR001.wrl)
  compile_openhrp_model(${choreonoid_PACKAGE_PATH}/share/choreonoid-1.3/model/GR001/GR001.wrl)
else()
  message("\n\n\n\n ${choreonoid_PACKAGE_PATH}/share/choreonoid-1.3/model/GR001/GR001.wrl is not found..\n\n\n\n")
endif()

## ALL
list(GET compile_robots 0 compile_robot)
list(REMOVE_AT compile_robots 0)
add_custom_target(openhrp_robots ALL)
add_dependencies(openhrp_robots ${compile_robot})
foreach(compile_target ${compile_robots})
  add_dependencies(${compile_robot} ${compile_target})
  list(GET compile_robots 0 compile_robot)
  list(REMOVE_AT compile_robots 0)
endforeach(compile_target ${compile_robots})

##
## test
##

rosbuild_check_for_display(disp)
if(disp)
  file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build/images)
  rosbuild_add_rostest(launch/hrp4c.launch)
  rosbuild_add_rostest(launch/hrp4c_nosim.launch)
  # rosbuild_add_rostest(launch/pa10.launch) #segfault?
  rosbuild_add_rostest(launch/darwin.launch)
  rosbuild_add_rostest(launch/samplerobot.launch)
  rosbuild_add_rostest(launch/hironx.launch)
  rosbuild_add_rostest(launch/smartpal5.launch)
  rosbuild_add_rostest(launch/pr2.launch)
  rosbuild_add_rostest(launch/gr001.launch)
endif(disp)

##
## documents
##
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build/ --nomakefile RESULT_VARIABLE _make_failed)


