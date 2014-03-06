cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge_cmake_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)

# Use pa10.dae to test compile_robot_model.cmake
set(PKG_CONFIG_PATH "${openhrp3_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}") # for openrtm3.1.pc
execute_process(
  COMMAND pkg-config openhrp3.1 --variable=idl_dir
  OUTPUT_VARIABLE OPENHRP_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
  return(1)
endif()

# ./openrtm_common/openhrp3/share/OpenHRP-3.1/sample/model/PA10/pa10.dae
set(OPENHRP_SAMPLE_DIR ${OPENHRP_IDL_DIR}/../sample)
set(PA10_MODEL ${OPENHRP_SAMPLE_DIR}/model/PA10/pa10.main.wrl)
message("test compiling openhrp model: ${PA10_MODEL}")
compile_openhrp_model(${PA10_MODEL})

catkin_package(
  DEPENDS openhrp3
  CATKIN_DEPENDS hrpsys_ros_bridge
)
