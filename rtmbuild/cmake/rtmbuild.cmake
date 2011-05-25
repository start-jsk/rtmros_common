cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(_RTMBUILD_GENERATED_IDL_FILES "")
macro(rtmbuild_get_idls idlvar)
  file(GLOB _idl_files RELATIVE "${PROJECT_SOURCE_DIR}/idl" "${PROJECT_SOURCE_DIR}/idl/*.idl")
  set(${idlvar} ${_RTMBUILD_GENERATED_IDL_FILES})
  foreach(_idl ${_idl_files})
    # copy from rosbuild_get_msgs to avoid .#Foo.idl, by emacs
    if(${_idl} MATCHES "^[^\\.].*\\.idl$")
      list(APPEND ${idlvar} ${_idl})
    endif(${_idl} MATCHES "^[^\\.].*\\.idl$")
  endforeach(_idl)
endmacro(rtmbuild_get_idls)

#compile idl/*.idl file into c++
macro(rtmbuild_genidl)
  message("[rtmbuild_genidl] Compiling idls in package ${_project}")
  rtmbuild_get_idls(_idllist)
  set(_autogen "")
  if(NOT _idllist)
    _rosbuild_warn("rtmbuild_genidl() was called, but no .idl files ware found")
  else(NOT _idllist)
    file(WRITE ${PROJECT_SOURCE_DIR}/idl_gen/generated "yes")
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${_project}/idl)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl_gen/lib)
  endif(NOT _idllist)

  set(_output_dir ${PROJECT_SOURCE_DIR}/idl_gen)
  set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${_project}/idl)
  set(_output_lib_dir ${PROJECT_SOURCE_DIR}/idl_gen/lib)

  foreach(_idl ${_idllist})
    message("[rtmbuild_genidl] ${_idl}")
    execute_process(COMMAND rtm-config --idlc OUTPUT_VARIABLE _genidl_exe
      OUTPUT_STRIP_TRAILING_WHITESPACE)
#    --cxx is grx option???
#    execute_process(COMMAND rtm-config --cxx OUTPUT_VARIABLE _rtmcxx_ex
#      OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(_rtmcxx_exe "g++")

    set(_input_idl ${PROJECT_SOURCE_DIR}/idl/${_idl})
    set(_output_idl_hh ${_output_cpp_dir}/${_idl})
    set(_output_stub_cpp ${_output_cpp_dir}/${_idl})
    set(_output_skel_cpp ${_output_cpp_dir}/${_idl})
    set(_output_stub_lib ${_output_lib_dir}/lib${_idl})
    set(_output_skel_lib ${_output_lib_dir}/lib${_idl})
    string(REPLACE ".idl" ".hh" _output_idl_hh ${_output_idl_hh})
    string(REPLACE ".idl" "Stub.cpp" _output_stub_cpp ${_output_stub_cpp})
    string(REPLACE ".idl" "Stub.so"  _output_stub_lib ${_output_stub_lib})
    string(REPLACE ".idl" "Skel.cpp" _output_skel_cpp ${_output_skel_cpp})
    string(REPLACE ".idl" "Skel.so"  _output_skel_lib ${_output_skel_lib})

    # call the  rule to compile idl
    add_custom_command(OUTPUT ${_output_idl_hh}
      COMMAND ${_genidl_exe} `rtm-config --idlflags` -I`rtm-config --prefix`/include/rtm/idl -I`rospack find openhrp3`/share/OpenHRP-3.1/idl -C${_output_cpp_dir} ${_input_idl}
      DEPENDS ${_input})
    add_custom_command(OUTPUT ${_output_stub_cpp} ${_output_skel_cpp}
      COMMAND cp ${_input_idl} ${_output_cpp_dir}
      COMMAND rtm-skelwrapper --include-dir="" --skel-suffix=Skel --stub-suffix=Stub  --idl-file=${_idl}
      COMMAND rm ${_output_cpp_dir}/${_idl}
      WORKING_DIRECTORY ${_output_cpp_dir}
      DEPENDS ${_output_idl_hh})
    add_custom_command(OUTPUT ${_output_stub_lib}
      COMMAND ${_rtmcxx_exe} `rtm-config --cflags` -I. -shared -o ${_output_stub_lib} ${_output_stub_cpp} `rtm-config --libs`
      DEPENDS ${_output_stub_cpp})
    add_custom_command(OUTPUT ${_output_skel_lib}
      COMMAND ${_rtmcxx_exe} `rtm-config --cflags` -I. -shared -o ${_output_skel_lib} ${_output_skel_cpp} `rtm-config --libs`
      DEPENDS ${_output_skel_cpp})
    list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib})
  endforeach(_idl)
  ##
  add_dependencies(rtmbuild_genidl RTMBUILD_genidl_cpp)
  add_custom_target(RTMBUILD_genidl_cpp DEPENDS ${_autogen})

  #
  if(_autogen)
    # Also set up to clean the srv_gen directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${PROJECT_SOURCE_DIR}/idl_gen)
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genidl)

##
macro(rtmbuild_init)
  rosbuild_init()
  message("[rtmbuild] Building package ${_project}")
  add_custom_target(rtmbuild_genidl ALL)
  file(REMOVE ${PROJECT_SOURCE_DIR}/idl_gen/generated)

  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} DIRS depends-manifests)
  foreach(_dir ${${_prefix}_DIRS})
    string(REPLACE "manifest.xml" "idl_gen/cpp" _cpp_dir ${_dir})
    list(APPEND ${_prefix}_INCLUDE_DIRS ${_cpp_dir})
    string(REPLACE "manifest.xml" "idl_gen/lib" _lib_dir ${_dir})
    list(APPEND ${_prefix}_LIBRARY_DIRS ${_lib_dir})
  endforeach(_dir)

  ## use pkgconfig with openhrp3.1 and openrtm-aist
  # catch the error output to suppress it
  include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
  execute_process(
    COMMAND rospack find openhrp3
    OUTPUT_VARIABLE _openhrp3_pkg_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  SET(ENV{PKG_CONFIG_PATH} ${_openhrp3_pkg_dir}/lib/pkgconfig)
  pkg_check_modules(OPENHRP REQUIRED openhrp3.1)
  pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
  list(APPEND ${_prefix}_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS})
  list(APPEND ${_prefix}_INCLUDE_DIRS ${OPENHRP_INCLUDE_DIRS})
  list(APPEND ${_prefix}_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS})
  list(APPEND ${_prefix}_LIBRARY_DIRS ${OPENHRP_LIBRARY_DIRS})
  include_directories(${${_prefix}_INCLUDE_DIRS})
  link_directories(${${_prefix}_LIBRARY_DIRS})
  foreach(_lib ${OPENRTM_LIBRARIES})
    list(APPEND ${_prefix}_LIBRARIES ${_lib})
  endforeach(_lib)
  foreach(_lib ${OPENHRP_LIBRARIES})
    list(APPEND ${_prefix}_LIBRARIES ${_lib})
  endforeach(_lib)
endmacro(rtmbuild_init)

macro(rtmbuild_add_executable exe)
  rosbuild_add_executable(${ARGV})
#  rosbuild_add_executable(${ARGV})
#  message(${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})
#  target_link_libraries(${exe} ${PROJECT_NAME}_LD)
endmacro(rtmbuild_add_executable)



