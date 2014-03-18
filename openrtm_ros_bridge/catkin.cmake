# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openrtm_ros_bridge)

# call catking depends
find_package(catkin REQUIRED COMPONENTS roscpp openrtm_tools rtmbuild rostest)

# include rtmbuild
if(EXISTS ${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
  include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
elseif(EXISTS ${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
  include(${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "rtmbuild is not found")
endif()

# copy idl files from openrtm_aist
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
execute_process(COMMAND pkg-config --variable=data_prefix openrtm-aist
  OUTPUT_VARIABLE openrtm_aist_DATA_PREFIX
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT RESULT EQUAL 0)
  message(FATAL_ERROR "fail to run pkg-config")
endif()
set(openrtm_aist_EXAMPLE_IDL_DIR ${openrtm_aist_DATA_PREFIX}/openrtm-1.1/example/)
if(EXISTS ${openrtm_aist_EXAMPLE_IDL_DIR})
  file(GLOB_RECURSE _idl_files "${openrtm_aist_EXAMPLE_IDL_DIR}/*/*/*.idl") #fix me
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "${openrtm_aist_EXAMPLE_IDL_DIR}/*.idl is not found")
endif()
foreach(_idl_file ${_idl_files})
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_idl_file} ${PROJECT_SOURCE_DIR}/idl/)
endforeach()

unset(openrtm_tools_LIBRARIES CACHE) # remove not to add openrtm_aist_LIBRARIES to hrpsys_ros_bridgeConfig.cmake

# initialize rtmbuild
rtmbuild_init()

# call catkin_package, after rtmbuild_init, before rtmbuild_gen*
catkin_package(
    DEPENDS #
    CATKIN_DEPENDS openrtm_tools roscpp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

##
## install
##
install(DIRECTORY test samples DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

##
## tests
##
add_rostest(test/test_myservice_rosbridge.test)

