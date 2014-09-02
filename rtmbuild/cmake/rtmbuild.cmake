cmake_minimum_required(VERSION 2.8.3)

#set(DEBUG_RTMBUILD_CMAKE TRUE)

set(use_catkin TRUE)

# for rosbuild
if(NOT COMMAND _rtmbuild_genbridge_init)
  include(${rtmbuild_PACKAGE_PATH}/cmake/servicebridge.cmake)
  set(use_catkin FALSE)
endif()


##
## GLOBAL VARIABLES
##
## openrtm_aist_INCLUDE_DIRS
## openrtm_aist_LIBRARIES
## openhrp3_INCLUDE_DIRS
## openhrp3_LIBRARIES
## idl2srv_EXECUTABLE
## rtmskel_EXECUTABLE
## ${PROJECT_NAME}_idl_files
## ${PROJECT_NAME}_autogen_files
## ${PROJECT_NAME}_autogen_msg_files
## ${PROJECT_NAME}_autogen_srv_files
## ${PROJECT_NAME}_autogen_interfaces
## rtm_idlc, rtm_idlflags, rtm_idldir
## rtm_cxx,  rtm_cflags
## hrp_idldir


#
# setup global variables
#
macro(rtmbuild_init)
  if(NOT use_catkin) ## rosbuild_init cleans all variable so we first defind project and call rosbuild_init later with ROSBUILD_DONT_REDEFINE_PROJECT=TRUE
    get_filename_component(_project ${CMAKE_SOURCE_DIR} NAME)
    project(${_project})

    rosbuild_find_ros_package(openrtm_aist)
    rosbuild_find_ros_package(openhrp3)
    set(ENV{PKG_CONFIG_PATH} ${openrtm_aist_PACKAGE_PATH}/lib/pkgconfig:${openhrp3_PACKAGE_PATH}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
    message("[rtmbuild_init] - ENV{PKG_CONFIG_PATH} -  > $ENV{PKG_CONFIG_PATH}")
  endif()
  #
  # use pkg-config to set --cflags --libs plus rtm-related flags
  #
  find_package(PkgConfig)
  pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
  pkg_check_modules(openhrp3 openhrp3.1)
  message("[rtmbuild_init] Building package ${CMAKE_SOURCE_DIR} ${PROJECT_NAME}")
  message("[rtmbuild_init] - CATKIN_TOPLEVEL = ${CATKIN_TOPLEVEL}")
  if(DEBUG_RTMBUILD_CMAKE)
    message("[rtmbuild_init] - openrtm_aist_INCLUDE_DIRS -> ${openrtm_aist_INCLUDE_DIRS}")
    message("[rtmbuild_init] - openrtm_aist_LIBRARIES    -> ${openrtm_aist_LIBRARIES}")
    message("[rtmbuild_init] - openhrp3_INCLUDE_DIRS -> ${openhrp3_INCLUDE_DIRS}")
    message("[rtmbuild_init] - openhrp3_LIBRARIES    -> ${openhrp3_LIBRARIES}")
  endif()

  if(EXISTS ${rtmbuild_SOURCE_PREFIX}) # catkin
    set(idl2srv_EXECUTABLE ${rtmbuild_SOURCE_PREFIX}/scripts/idl2srv.py)
  elseif(EXISTS ${rtmbuild_PACKAGE_PATH}) ## for rosbuild
    set(idl2srv_EXECUTABLE ${rtmbuild_PACKAGE_PATH}/scripts/idl2srv.py)
  else()
    pkg_check_modules(rtmbuild rtmbuild REQUIRED)
    set(idl2srv_EXECUTABLE ${rtmbuild_PREFIX}/share/rtmbuild/scripts/idl2srv.py)
  endif()
  message("[rtmbuild_init] - idl2srv_EXECUTABLE     -> ${idl2srv_EXECUTABLE}")

  execute_process(COMMAND pkg-config openrtm-aist --variable=prefix      OUTPUT_VARIABLE rtm_prefix    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(EXISTS ${rtm_prefix}/bin/rtm-skelwrapper)
    set(_rtm_exe_path ${rtm_prefix}/bin)
  else()
    set(_rtm_exe_path ${rtm_prefix}/lib/openrtm_aist/bin)
  endif()
  set(rtmskel_EXECUTABLE PATH=${_rtm_exe_path}:$ENV{PATH} PYTHONPATH=${openrtm_aist_PREFIX}/lib/openrtm-1.1/py_helper:$ENV{PYTHONPATH} ${_rtm_exe_path}/rtm-skelwrapper)
  message("[rtmbuild_init] - rtmskel_EXECUTABLE     -> ${rtmskel_EXECUTABLE}")

  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idlc     OUTPUT_VARIABLE rtm_idlc     OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idlflags OUTPUT_VARIABLE rtm_idlflags OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_idldir   OUTPUT_VARIABLE rtm_idldir   OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_cxx      OUTPUT_VARIABLE rtm_cxx      OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_cflags   OUTPUT_VARIABLE rtm_cflags   OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openrtm-aist --variable=rtm_libs     OUTPUT_VARIABLE rtm_libs     OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND pkg-config openhrp3.1   --variable=idl_dir      OUTPUT_VARIABLE hrp_idldir   OUTPUT_STRIP_TRAILING_WHITESPACE)
  separate_arguments(rtm_idlflags)
  separate_arguments(rtm_cflags)
  separate_arguments(rtm_libs)
  set(rtm_cxx "c++") ## openrtm-aist --variable=rtm_cxx sometimes returns /usr/lib/ccache/c++
  message("[rtmbuild_init] - rtm_idlc               -> ${rtm_idlc}")
  message("[rtmbuild_init] - rtm_idlflags           -> ${rtm_idlflags}")
  message("[rtmbuild_init] - rtm_idldir             -> ${rtm_idldir}")
  message("[rtmbuild_init] - rtm_cxx                -> ${rtm_cxx}")
  message("[rtmbuild_init] - rtm_cflags             -> ${rtm_cflags}")
  message("[rtmbuild_init] - rtm_libs               -> ${rtm_libs}")
  message("[rtmbuild_init] - hrp_idldir             -> ${hrp_idldir}")

  ##
  ## get idl files and store to _idl_list
  message("[rtmbuild_init] Generating bridge compornents from ${PROJECT_SOURCE_DIR}/idl")
  set(${PROJECT_NAME}_idl_files "")
  _rtmbuild_get_idls() ## set ${PROJECT_NAME}_idl_files
  message("[rtmbuild_init] - ${PROJECT_NAME}_idl_files : ${${PROJECT_NAME}_idl_files}")
  if(NOT ${PROJECT_NAME}_idl_files)
    message(AUTHOR_WARNING "[rtmbuild_init] - no idl file is defined")
  endif()

  ## generate msg/srv/cpp from idl
  set(${PROJECT_NAME}_autogen_msg_files "")
  set(${PROJECT_NAME}_autogen_srv_files "")
  _rtmbuild_genbridge_init()
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_msg_files  : ${${PROJECT_NAME}_autogen_msg_files}")
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_srv_files  : ${${PROJECT_NAME}_autogen_srv_files}")
  message("[rtmbuild_init] - ${PROJECT_NAME}_autogen_interfaces : ${${PROJECT_NAME}_autogen_interfaces}")
  set(rtmbuild_${PROJECT_NAME}_autogen_msg_files ${${PROJECT_NAME}_autogen_msg_files}) 

  ##
  ## rosbulid_init for rosbuild
  if(NOT use_catkin)
    set(ROSBUILD_DONT_REDEFINE_PROJECT TRUE)
    rosbuild_init()
  endif()

  if(use_catkin)
    add_message_files(DIRECTORY msg FILES "${${PROJECT_NAME}_autogen_msg_files}")
    add_service_files(DIRECTORY srv FILES "${${PROJECT_NAME}_autogen_srv_files}")
    generate_messages(DEPENDENCIES std_msgs)
  else()
    rosbuild_genmsg()
    rosbuild_gensrv()
  endif()

  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${openhrp3_LIBRARY_DIRS})

endmacro(rtmbuild_init)

# add_custom_command to compile idl/*.idl file into c++
macro(rtmbuild_genidl)
  message("[rtmbuild_genidl] add_custom_command for idl files in package ${PROJECT_NAME}")

  set(_autogen "")

  if (use_catkin)
    set(_output_cpp_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set(_output_lib_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(_output_python_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME})
  else()
    set(_output_dir ${PROJECT_SOURCE_DIR}/idl_gen)
    set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${PROJECT_NAME})
    set(_output_lib_dir ${PROJECT_SOURCE_DIR}/idl_gen/lib)
    set(_output_python_dir ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME})
    include_directories(${PROJECT_SOURCE_DIR}/idl_gen/cpp/)
  endif()

  set(_output_idl_py_files "")
  file(MAKE_DIRECTORY ${_output_cpp_dir}/idl)
  file(MAKE_DIRECTORY ${_output_lib_dir})
  link_directories(${_output_lib_dir})

  message("[rtmbuild_genidl] - _output_cpp_dir : ${_output_cpp_dir}")
  message("[rtmbuild_genidl] - _output_lib_dir : ${_output_lib_dir}")
  message("[rtmbuild_genidl] - _output_python_dir : ${_output_python_dir}")

  ## RTMBUILD_${PROJECT_NAME}_genrpc) depends on each RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genrpc)
  if(NOT ${PROJECT_NAME}_idl_files)
    message(AUTHOR_WARNING "[rtmbuild_genidl] - no idl file is defined")
  endif()
  foreach(_idl_file ${${PROJECT_NAME}_idl_files})
    get_filename_component(_idl_name ${_idl_file} NAME_WE)
    message("[rtmbuild_genidl] - _idl_file : ${_idl_file}")
    message("[rtmbuild_genidl] - _idl_name : ${_idl_name}")

    # set(_input_idl ${PROJECT_SOURCE_DIR}/idl/${_idl})

    set(_output_idl_hh ${_output_cpp_dir}/idl/${_idl_name}.hh)
    set(_output_idl_py ${_output_python_dir}/${_idl_name}_idl.py)
    set(_output_stub_h ${_output_cpp_dir}/idl/${_idl_name}Stub.h)
    set(_output_skel_h ${_output_cpp_dir}/idl/${_idl_name}Skel.h)
    set(_output_stub_cpp ${_output_cpp_dir}/idl/${_idl_name}Stub.cpp)
    set(_output_skel_cpp ${_output_cpp_dir}/idl/${_idl_name}Skel.cpp)
    set(_output_stub_lib ${_output_lib_dir}/lib${_idl_name}Stub.so)
    set(_output_skel_lib ${_output_lib_dir}/lib${_idl_name}Skel.so)
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS lib${_idl_name}Stub.so lib${_idl_name}Skel.so)
    # call the  rule to compile idl
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl] ${_output_idl_hh}\n -> ${_idl_file} ${${_idl}_depends}")
      message("[rtmbuild_genidl] ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}\n -> ${_output_idl_hh}")
      message("[rtmbuild_genidl] ${_output_stub_lib} ${_output_skel_lib}\n -> ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h}")
    endif()
    # cpp
    add_custom_command(OUTPUT ${_output_idl_hh}
      COMMAND ${rtm_idlc} ${rtm_idlflags} -C${_output_cpp_dir}/idl ${_idl_file}
      DEPENDS ${_idl_file})
    add_custom_command(OUTPUT ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      COMMAND cp ${_idl_file} ${_output_cpp_dir}/idl
      COMMAND rm -f ${_output_stub_cpp} ${_output_skel_cpp} ${_output_stub_h} ${_output_skel_h}
      COMMAND ${rtmskel_EXECUTABLE} --include-dir="" --skel-suffix=Skel --stub-suffix=Stub  --idl-file=${_idl_file}
      WORKING_DIRECTORY ${_output_cpp_dir}/idl
      DEPENDS ${_output_idl_hh})
    add_custom_command(OUTPUT ${_output_stub_lib} ${_output_skel_lib}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_stub_lib} ${_output_stub_cpp} ${rtm_libs}
      COMMAND ${rtm_cxx} ${rtm_cflags} -I. -shared -o ${_output_skel_lib} ${_output_skel_cpp} ${rtm_libs}
      DEPENDS ${_output_stub_cpp} ${_output_stub_h} ${_output_skel_cpp} ${_output_skel_h})
    list(APPEND ${PROJECT_NAME}_IDLLIBRARY_DIRS ${_output_stub_lib} ${_output_skel_lib})
    if(use_catkin)
      install(PROGRAMS ${_output_stub_lib} ${_output_skel_lib} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    endif()
    # python
    list(APPEND _output_idl_py_files ${_output_idl_py})
    #
    list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib} ${_output_idl_py})

    # add custom target
    add_custom_target(RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc DEPENDS ${_output_stub_lib} ${_output_skel_lib})
    add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_${_idl_name}_genrpc)

  endforeach(_idl_file)
  # python
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genpy DEPENDS ${_output_idl_py_files})
  add_custom_command(OUTPUT ${_output_idl_py_files}
    COMMAND mkdir -p ${_output_python_dir}
    COMMAND echo \"${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}\"
    COMMAND ${rtm_idlc} -bpython -I${rtm_idldir} -C${_output_python_dir} ${${PROJECT_NAME}_idl_files}
    COMMENT "Generating python/idl from ${${PROJECT_NAME}_idl_files}"
    DEPENDS ${${PROJECT_NAME}_idl_files})
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_genpy)
  ##

  if(_autogen)
    if(DEBUG_RTMBUILD_CMAKE)
      message("[rtmbuild_genidl] ADDITIONAL_MAKE_CLEAN_FILES : ${_autogen}")
    endif()
    # Also set up to clean the srv_gen directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${_autogen})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genidl)

macro(rtmbuild_genbridge)
  message("[rtmbuild_genbridge] generate OpenRTM-ROS bridges")
  #
  add_custom_target(RTMBUILD_${PROJECT_NAME}_genbridge)

  message("[rtmbuild_genbridge] - ${PROJECT_NAME}_autogen_interfaces : ${${PROJECT_NAME}_autogen_interfaces}")
  if(NOT ${PROJECT_NAME}_autogen_interfaces)
    message(AUTHOR_WARNING "[rtmbuild_genbridge] - no interface is defined")
  endif()
  foreach(_comp ${${PROJECT_NAME}_autogen_interfaces})
    message("[rtmbuild_genbridge] - rtmbuild_add_executable : ${_comp}ROSBridgeComp")
    rtmbuild_add_executable("${_comp}ROSBridgeComp" "src_gen/${_comp}ROSBridge.cpp" "src_gen/${_comp}ROSBridgeComp.cpp")
    add_custom_target(RTMBUILD_${PROJECT_NAME}_${_comp}_genbridge DEPENDS src_gen/${_comp}ROSBridge.cpp src_gen/${_comp}ROSBridgeComp.cpp)
    add_dependencies(RTMBUILD_${PROJECT_NAME}_genbridge RTMBUILD_${PROJECT_NAME}_${_comp}_genbridge)
  endforeach(_comp)

  # TARGET
  ## RTMBUILD_${PROJECT_NAME}_gencpp    : generated cpp files from idl with id2srv.py
  ##                                    -> depends on idl
  ## RTMBUILD_${PROJECT_NAME}_genrpc    : stub/skel files generated from genidl
  ##                                    -> RTMBUILD_${PROJECT_NAME}_gencpp
  ## RTMBUILD_${PROJECT_NAME}_genbridge : bridge component files generated from stub skel
  ##                                    -> RTMBUILD_${PROJECT_NAME}_genrpc
  ## ${exe}                             -> RTMBUILD_${PROJECT_NAME}_genbridge

  add_dependencies(RTMBUILD_${PROJECT_NAME}_gencpp RTMBUILD_${PROJECT_NAME}_genidl ROSBUILD_genmsg_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_generate_messages_py)
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genrpc RTMBUILD_${PROJECT_NAME}_gencpp)
  add_dependencies(RTMBUILD_${PROJECT_NAME}_genbridge RTMBUILD_${PROJECT_NAME}_genrpc)

endmacro(rtmbuild_genbridge)

macro(rtmbuild_add_executable exe)
  if (use_catkin)
    add_executable(${ARGV})
    install(TARGETS ${exe} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    rosbuild_add_executable(${ARGV})
  endif()
  add_dependencies(${exe} RTMBUILD_${PROJECT_NAME}_genbridge ${${_package}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
  target_link_libraries(${exe} ${catkin_LIBRARIES} ${openrtm_aist_LIBRARIES} ${openhrp3_LIBRARIES} ${${PROJECT_NAME}_IDLLIBRARY_DIRS} )
endmacro(rtmbuild_add_executable)

macro(rtmbuild_add_library lib)
  if (use_catkin)
    add_library(${ARGV})
    install(TARGETS ${LIB} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  else()
    rosbuild_add_library(${ARGV})
  endif()
  target_link_libraries(${lib} ${openrtm_aist_LIBRARIES} ${openhrp3_LIBRARIES} ${${PROJECT_NAME}_IDLLIBRARY_DIRS})
endmacro(rtmbuild_add_library)

