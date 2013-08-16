# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openrtm_ros_bridge)

# call catking depends
find_package(catkin REQUIRED COMPONENTS roscpp openrtm_aist openrtm_tools rtmbuild rostest)

# include rtmbuild
if(EXISTS ${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
  include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
elseif(EXISTS ${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
  include(${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "rtmbuild is not found")
endif()

# copy idl files from openrtm_aist
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
if(EXISTS ${openrtm_aist_SOURCE_DIR}/share/openrtm-1.1/example/)
  file(GLOB_RECURSE _idl_files "${openrtm_aist_SOURCE_DIR}/share/openrtm-1.1/example/*.idl")
elseif(EXISTS ${openrtm_aist_PREFIX}/share/openrtm_aist/share/openrtm-1.1/example/)
  file(GLOB_RECURSE _idl_files "${openrtm_aist_PREFIX}/share/openrtm_aist/share/openrtm-1.1/example/*.idl")
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "openrtm-1.1/example/*.idl is not found")
endif()
foreach(_idl_file ${_idl_files})
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_idl_file} ${PROJECT_SOURCE_DIR}/idl/)
endforeach()

# initialize rtmbuild
rtmbuild_init()

# call catkin_package, after rtmbuild_init, before rtmbuild_gen*
catkin_package(
    DEPENDS #
    CATKIN-DEPENDS # openrtm_tools rtmbuild roscpp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

##
## tests
##

add_rostest(samples/test_myservice_rosbridge.launch)

