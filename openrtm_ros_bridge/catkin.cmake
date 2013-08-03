# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(openrtm_ros_bridge)

# call catking depends
find_package(catkin REQUIRED COMPONENTS roscpp openrtm_aist openrtm_tools rtmbuild rostest)

# include rtmbuild
find_package(PkgConfig)
pkg_check_modules(rtmbuild rtmbuild) # REQUIRED
include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)

# copy idl files from openrtm_aist
pkg_check_modules(openrtm_aist openrtm_aist) # REQUIRED
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
file(GLOB_RECURSE _idl_files "${openrtm_aist_SOURCE_DIR}/share/openrtm-1.1/example/*.idl")
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

