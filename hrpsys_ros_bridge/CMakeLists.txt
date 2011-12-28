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
macro(compile_openhrp_model wrlfile workdir daename yamlname lispname)
  add_custom_command(OUTPUT ${workdir}/${lispname}
    COMMAND rosrun euscollada collada2eus ${workdir}/${daename} ${PROJECT_SOURCE_DIR}/models/${yamlname} ${workdir}/${lispname}
    DEPENDS ${workdir}/${daename})
  add_custom_command(OUTPUT ${workdir}/${daename}
    COMMAND rosrun openhrp3 export-collada -i ${wrlfile} -o ${workdir}/${daename}
    DEPENDS ${wrlfile})
  list(APPEND all_robots ${workdir}/${lispname})
endmacro(compile_openhrp_model)

rosbuild_find_ros_package(hrpsys)
if(EXISTS ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl)
  compile_openhrp_model(
    ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl
    ${PROJECT_SOURCE_DIR}/models
    HRP4C.dae hrp4c.yaml hrp4c.l)
endif(EXISTS ${hrpsys_PACKAGE_PATH}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl)

rosbuild_find_ros_package(openhrp3)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl
  ${PROJECT_SOURCE_DIR}/models
  PA10.dae pa10.yaml pa10.l)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/sample1.wrl
  ${PROJECT_SOURCE_DIR}/models
  SampleRobot.dae samplerobot.yaml samplerobot.l)

rosbuild_find_ros_package(collada_robots)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/kawada-hironx.l
  COMMAND rosrun euscollada collada2eus ${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae ${PROJECT_SOURCE_DIR}/models/kawada-hironx.yaml ${PROJECT_SOURCE_DIR}/models/kawada-hironx.l
  DEPENDS ${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae)
list(APPEND all_robots ${PROJECT_SOURCE_DIR}/models/kawada-hironx.l)

# generate webots model
rosbuild_find_ros_package(webots_simulator)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/DARWIN.wrl
  COMMAND rosrun roseus roseus ${PROJECT_SOURCE_DIR}/scripts/webotsvrml2openhrp3vrml.l
  "\\(progn \\(convert-webots-vrml-\\>openhrp3-vrml-for-darwin \\\"${PROJECT_SOURCE_DIR}/models/DARWIN.wrl\\\"\\) \\(exit\\)\\)"
  DEPENDS ${webots_simulator_PACKAGE_PATH}/build/webots/resources/projects/robots/darwin-op/protos/DARwInOP.proto)
compile_openhrp_model(
  ${PROJECT_SOURCE_DIR}/models/DARWIN.wrl
  ${PROJECT_SOURCE_DIR}/models
  DARWIN.dae darwin.yaml darwin.l)

# yaskawa model
rosbuild_download_data(http://openhrp-aist-grx.googlecode.com/files/YaskawaSmartPal5.zip build/YaskawaSmartPal5.zip 3db916d5dc5ef4259c7dfc1ac85dd380)
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/models/YaskawaSmartPal5.wrl
  COMMAND unzip -o ${PROJECT_SOURCE_DIR}/build/YaskawaSmartPal5.zip
  COMMAND mv    -f ${PROJECT_SOURCE_DIR}/build/YaskawaSmartPal5/* ${PROJECT_SOURCE_DIR}/models/
  COMMAND touch ${PROJECT_SOURCE_DIR}/models/YaskawaSmartPal5.wrl
  DEPENDS build/YaskawaSmartPal5.zip)
compile_openhrp_model(
  ${PROJECT_SOURCE_DIR}/models/YaskawaSmartPal5.wrl
  ${PROJECT_SOURCE_DIR}/models
  SmartPal5.dae smartpal5.yaml smartpal5.l)

add_custom_target(openhrp_robots ALL DEPENDS ${all_robots})



##
## hrpsys ros bridge tools
##
rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)

##
## test
##
rosbuild_add_rostest(launch/hrp4c.launch)
rosbuild_add_rostest(launch/hrp4c_nosim.launch)
# rosbuild_add_rostest(launch/pa10.launch) #segfault?
rosbuild_add_rostest(launch/darwin.launch)
rosbuild_add_rostest(launch/samplerobot.launch)
rosbuild_add_rostest(launch/hironx.launch)
rosbuild_add_rostest(launch/smartpal5.launch)
#rosbuild_add_rostest(launch/pr2.launch) not working

##
## documents
##
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)


