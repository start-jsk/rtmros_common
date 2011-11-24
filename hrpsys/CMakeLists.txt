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
rtmbuild_add_executable(AbsTransformToPosRpy src/AbsTransformToPosRpy.cpp src/AbsTransformToPosRpyComp.cpp)


execute_process(COMMAND rosrun hrpsys hrp4c_model_download.sh RESULT_VARIABLE _download_failed)
if (_download_failed)
  message(STATUS "Download HRP-4C model from http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html")
endif(_download_failed)

set(wrl_robots)
macro(compile_openhrp_model wrlfile workdir daename yamlname lispname)
  add_custom_command(OUTPUT ${workdir}/${lispname}
    COMMAND rosrun euscollada collada2eus ${workdir}/${daename} ${PROJECT_SOURCE_DIR}/scripts/${yamlname} ${workdir}/${lispname}
    DEPENDS ${workdir}/${daename})
  add_custom_command(OUTPUT ${workdir}/${daename}
    COMMAND rosrun openhrp3 export-collada -i ${wrlfile} -o ${workdir}/${daename}
    DEPENDS ${wrlfile})
  list(APPEND all_robots ${workdir}/${lispname})
endmacro(compile_openhrp_model)

if(EXISTS ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl)
  compile_openhrp_model(
    ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl
    ${PROJECT_SOURCE_DIR}/scripts
    HRP4C.dae hrp4c.yaml hrp4c.l)
endif(EXISTS ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl)
rosbuild_find_ros_package(openhrp3)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl
  ${PROJECT_SOURCE_DIR}/scripts
  PA10.dae pa10.yaml pa10.l)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/sample1.wrl
  ${PROJECT_SOURCE_DIR}/scripts
  SampleRobot.dae samplerobot.yaml samplerobot.l)
rosbuild_find_ros_package(collada_robots)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/scripts/kawada-hironx.l
  COMMAND rosrun euscollada collada2eus ${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae ${PROJECT_SOURCE_DIR}/scripts/kawada-hironx.yaml ${PROJECT_SOURCE_DIR}/scripts/kawada-hironx.l
  DEPENDS ${collada_robots_PACKAGE_PATH}/data/robots/kawada-hironx.dae)
list(APPEND all_robots ${PROJECT_SOURCE_DIR}/scripts/kawada-hironx.l)

# generate darwin model
rosbuild_find_ros_package("webots")
if(EXISTS "${webots_PACKAGE_PATH}/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto")
  message("webots package found. convert darwin model.")
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/scripts/DARWIN.wrl
    COMMAND rosrun roseus roseus ${PROJECT_SOURCE_DIR}/scripts/webotsvrml2openhrp3vrml.l
    "\\(progn \\(convert-webots-vrml-\\>openhrp3-vrml-for-darwin \\\"${PROJECT_SOURCE_DIR}/scripts/DARWIN.wrl\\\"\\) \\(exit\\)\\)"
    DEPENDS ${webots_PACKAGE_PATH}/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto)
  compile_openhrp_model(
    ${PROJECT_SOURCE_DIR}/scripts/DARWIN.wrl
    ${PROJECT_SOURCE_DIR}/scripts
    DARWIN.dae darwin.yaml darwin.l)
else(EXISTS "${webots_PACKAGE_PATH}/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto")
  message("webots package not found. not convert darwin model.")
endif(EXISTS "${webots_PACKAGE_PATH}/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto")

add_custom_target(openhrp_robots ALL DEPENDS ${all_robots})


rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)

rosbuild_add_rostest(launch/hrp4c.launch)
rosbuild_add_rostest(launch/pa10.launch)

