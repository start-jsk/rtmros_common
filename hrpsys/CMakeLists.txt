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
target_link_libraries(AbsTransformToPosRpy SimulationEC)


execute_process(COMMAND rosrun hrpsys hrp4c_model_download.sh RESULT_VARIABLE _download_failed)
if (_download_failed)
  message(FATAL_ERROR "Download HRP-4C model from http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html")
endif(_download_failed)

macro(compile_openhrp_model wrlfile workdir daename yamlname lispname)
  execute_process(COMMAND rosrun openhrp3 export-collada
    -i ${wrlfile} -o ${workdir}/${daename}
    RESULT_VARIABLE _export_collada_failed)
  if (_export_collada_failed)
    message(FATAL_ERROR "convert from ${wrlfile} to ${workdir}/${daename} failed")
  endif(_export_collada_failed)
  execute_process(COMMAND rosrun euscollada collada2eus
    ${workdir}/${daename}
    ${PROJECT_SOURCE_DIR}/scripts/${yamlname}
    ${workdir}/${lispname}
    RESULT_VARIABLE _export_collada2eus)
  if (_export_collada2eus)
    message(FATAL_ERROR "convert from ${workdir}/${daename} to ${workdir}/${lispname} failed")
  endif(_export_collada2eus)
endmacro(compile_openhrp_model)

compile_openhrp_model(
  ${PROJECT_SOURCE_DIR}/share/hrpsys/samples/HRP-4C/HRP4Cmain.wrl
  ${PROJECT_SOURCE_DIR}/scripts
  HRP4C.dae hrp4c.yaml hrp4c.l)
rosbuild_find_ros_package(openhrp3)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl
  ${PROJECT_SOURCE_DIR}/scripts
  PA10.dae pa10.yaml pa10.l)
compile_openhrp_model(
  ${openhrp3_PACKAGE_PATH}/share/OpenHRP-3.1/sample/model/sample1.wrl
  ${PROJECT_SOURCE_DIR}/scripts
  SampleRobot.dae samplerobot.yaml samplerobot.l)
# compile_openhrp_model(
#   /home/k-okada/prog/OpenHRP2/etc/DARWIN/DARWINmain.wrl
#   ${PROJECT_SOURCE_DIR}/scripts
#   Darwin.dae darwin.yaml darwin.l)




