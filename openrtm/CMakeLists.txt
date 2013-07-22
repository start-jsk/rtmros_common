cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(OPENRTM_BIN_PATH "${PROJECT_SOURCE_DIR}/bin")

if("$ENV{OPENRTM_VERSION}" STREQUAL "")
  set(OPENRTM_VERSION "1.1.0")
else("$ENV{OPENRTM_VERSION}" STREQUAL "")
  set(OPENRTM_VERSION $ENV{OPENRTM_VERSION})
endif("$ENV{OPENRTM_VERSION}" STREQUAL "")


configure_file(patch/openrtm-bin.patch-${OPENRTM_VERSION}.in openrtm-bin-${OPENRTM_VERSION}.patch)
add_custom_target(ALL DEPENDS openrtm-bin-${OPENRTM_VERSION}.patch)

# openrtm-c++ is installed via apt-get system
# Build OpenHRP3
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-c++-${OPENRTM_VERSION}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM C++ failed")
endif(_make_failed)
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-python-${OPENRTM_VERSION}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM Python failed")
endif(_make_failed)
#
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-java
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM Java failed")
endif(_make_failed)
#
#
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-rtctree-${OPENRTM_VERSION}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM Tools(rtctree) failed")
endif(_make_failed)
#
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-rtsprofile-${OPENRTM_VERSION}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM Tools(rtsprofile) failed")
endif(_make_failed)
#
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.openrtm-rtshell-${OPENRTM_VERSION}
                RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of OpenRTM Tools(rtshell) failed")
endif(_make_failed)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


###
###
include(FindPythonInterp)
## check python
find_package(PythonLibs) # using PYTHON_INCLUDE_PATH instead of PYTHON_INCLUDE_DIR

execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; from os.path import relpath; print relpath(get_python_lib(1,prefix='${PROJECT_SOURCE_DIR}'))"
      OUTPUT_VARIABLE OPENRTM_PYTHON_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _python_failed2)

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/src)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${OPENRTM_PYTHON_INSTALL_DIR} ${PROJECT_SOURCE_DIR}/src/openrtm RESULT_VARIABLE _symlink_failed)

## clean
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/bin;${CMAKE_SOURCE_DIR}/lib;${CMAKE_SOURCE_DIR}/src/openrtm;${CMAKE_SOURCE_DIR}/share")


### test code
rosbuild_add_rostesT(test/test-rtmlaunch.launch)
rosbuild_add_rostesT(test/test-rtmlaunch-tag.launch)
