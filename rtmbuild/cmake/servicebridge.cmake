#cmake_minimum_required(VERSION 2.4.6) ## this file is included from rtmbuild.cmake and cmake_minimum_required is defined in that file

# generate msg/srv files from idl, this will be called in rtmbuild_init
macro(rtmbuild_genbridge_init)
  if (${use_catkin}) # catkin set PROJECT_NAME on the top of catkin.cmake file
    set(_project_name ${PROJECT_NAME})
  else() # rtmbuild_genbridge_init to generate msg/srv files must be called before rosbuild, and rosbuild set PROJECT_NAME, so manually extract _project_name
    get_filename_component(_project_name ${CMAKE_SOURCE_DIR} NAME)
  endif()
  message("[rtmbuild_genbridge_init] Generating bridge compornents from ${_project_name}/idl")

  rtmbuild_get_idls(_idllist)
  if(NOT _idllist)
    message("[rtmbuild_genbridge_init] WARNING: rtmbuild_genbridge() was called, but no .idl files ware found")
  else(NOT _idllist)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/msg)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/srv)
  endif(NOT _idllist)

  message("[rtmbuild_genbridge_init] rtm_include_dir: ${OPENRTM_INCLUDE_DIRS}")
  set(_servicebridge_include_dirs "")
  foreach(_dirs ${OPENRTM_INCLUDE_DIRS})
    set(_servicebridge_include_dirs "${_servicebridge_include_dirs} ${_dirs}")
  endforeach()

  set(_servicebridge_include_dirs "${_servicebridge_include_dirs} ${_openhrp3_pkg_dir}/share/OpenHRP-3.1/idl")
  message("[rtmbuild_genbridge_init] _servicebridge_include_dir: ${_servicebridge_include_dirs}")

  set(_autogen "")
  set(_autogen_msg_files "")
  set(_autogen_srv_files "")
  foreach(_idl ${_idllist})
    message("[rtmbuild_genbridge_init] Generate msgs/srvs from ${_idl}")
    message("[rtmbuild_genbridge_init] ${_rtmbuild_pkg_dir}/scripts/idl2srv.py --filenames -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs=\"${_servicebridge_include_dirs}\" --package-name=${_project_name}")
    execute_process(COMMAND ${_rtmbuild_pkg_dir}/scripts/idl2srv.py --filenames -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_servicebridge_include_dirs}" --package-name=${_project_name} OUTPUT_VARIABLE _autogen_files OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _idl2srv_failed ERROR_VARIABLE _idl2srv_error)
    if (_idl2srv_failed)
      message(FATAL_ERROR "idl2srv.py failed ${_idl2srv_error} ${_autogen_files}")
    endif(_idl2srv_failed)
    if(_autogen_files)
      string(REPLACE "\n" ";" _autogen_files  ${_autogen_files})
      # remove already generated msg(_autogen) from _autogen_files
      foreach(_autogen_file ${_autogen_files})
        ## setup _autogen_msg_files, _autogen_srv_files
        get_filename_component(_ext ${_autogen_file} EXT)
        get_filename_component(_nam ${_autogen_file} NAME)
        if (${_ext} STREQUAL ".msg" )
          list(APPEND _autogen_msg_files ${_nam})
        elseif(${_ext} STREQUAL ".srv" )
          list(APPEND _autogen_srv_files ${_nam})
        endif()
        ##
	list(FIND _autogen ${_autogen_file} _found_autogen_file)
	if(${_found_autogen_file} GREATER -1)
	  list(REMOVE_ITEM _autogen_files ${_autogen_file})
	  message("[rtmbuild_genbridge_init] remove already generated file ${_autogen_file}")
	endif(${_found_autogen_file} GREATER -1)
      endforeach(_autogen_file ${_autogen_files})

      # check if idl or idl2srv servicebridge.cmake are newer than generated files
      set(_remove_autogen_files 0)
      foreach(_autogen_file ${_autogen_files})
        if(${PROJECT_SOURCE_DIR}/idl/${_idl} IS_NEWER_THAN ${_autogen_file} OR
           ${rtmbuild_PACKAGE_PATH}/scripts/idl2srv.py IS_NEWER_THAN ${_autogen_file} OR
           ${rtmbuild_PACKAGE_PATH}/cmake/servicebridge.cmake IS_NEWER_THAN ${_autogen_file})
         set(_remove_autogen_files 1)
       endif()
      endforeach(_autogen_file ${_autogen_files})
      if(_remove_autogen_files)
	message("[rtmbuild_genbridge_init] idl or idl2srv or cervicebridge.cmake is newer than generated fils")
	message("[rtmbuild_genbridge_init] remove ${_autogen_files}")
        file(REMOVE ${_autogen_files})
      endif()

      list(APPEND _autogen ${_autogen_files})
      execute_process(COMMAND ${_rtmbuild_pkg_dir}/scripts/idl2srv.py -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs=${_servicebridge_include_dirs} --tmpdir=/tmp/idl2srv/${_project_name} --package-name=${_project_name} RESULT_VARIABLE _idl2srv_failed ERROR_VARIABLE _idl2srv_error)
      if (_idl2srv_failed)
        message(FATAL_ERROR "idl2srv.py failed ${_idl2srv_error}")
      endif(_idl2srv_failed)

    endif(_autogen_files)
    set(_generated_msgs_from_idl "")
  endforeach(_idl)

  if (${use_catkin})
    # generated .h file
    string(REPLACE ";" " ${CATKIN_DEVEL_PREFIX}/include/${_project_name}/" _autogen_srv_h_files  "${CATKIN_DEVEL_PREFIX}/include/${_project_name}/${_autogen_srv_files}")
    string(REPLACE ".srv" ".h" _autogen_srv_h_files  ${_autogen_srv_h_files})
    string(REPLACE ";" " ${CATKIN_DEVEL_PREFIX}/include/${_project_name}/" _autogen_msg_h_files  "${CATKIN_DEVEL_PREFIX}/include/${_project_name}/${_autogen_msg_files}")
    string(REPLACE ".srv" ".h" _autogen_msg_h_files  ${_autogen_msg_h_files})
    # message generation
    foreach(${_project_name}_msg_files ${${_project_name}_MESSAGE_FILES})
      get_filename_component(${_project_name}_msg_file ${${_project_name}_msg_files} NAME)
      list(APPEND _autogen_msg_files ${${_project_name}_msg_file})
    endforeach()
    foreach(${_project_name}_srv_files ${${_project_name}_SERVICE_FILES})
      get_filename_component(${_project_name}_srv_file ${${_project_name}_srv_files} NAME)
      list(APPEND _autogen_srv_files ${${_project_name}_srv_file})
    endforeach()
    message("[rosbuild_genbridge] msg: ${_autogen_msg_files}")
    message("[rosbuild_genbridge] srv: ${_autogen_srv_files}")
    add_message_files(DIRECTORY msg FILES "${_autogen_msg_files}")
    add_service_files(DIRECTORY srv FILES "${_autogen_srv_files}")
    generate_messages(DEPENDENCIES std_msgs)
  endif()

  if(_autogen)
    # Also set up to clean the generated msg/srv/cpp/h files
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${_autogen})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genbridge_init)

macro(rtmbuild_genbridge)
  if (${use_catkin})
  else() # if (NOT ${use_catkin}) does not work
    message("[rtmbuild_genbridge] call rosbuild_genmsg/rosbuild_gensrv")
    rosbuild_genmsg()
    rosbuild_gensrv()
    message("[rtmbuild_genbridge] call rosbuild_genmsg ${_msglist}")
    message("[rtmbuild_genbridge] call rosbuild_gensrv ${_srvlist}")
  endif()
  rtmbuild_get_idls(_idllist)
  # rm tmp/idl2srv
  add_custom_command(OUTPUT /_tmp/idl2srv
    COMMAND rm -fr /tmp/idl2srv/${PROJECT_NAME})
  add_dependencies(rtmbuild_${PROJECT_NAME}_genidl RTMBUILD_${PROJECT_NAME}_rm_idl2srv)
  add_custom_target(RTMBUILD_${PROJECT_NAME}_rm_idl2srv ALL DEPENDS /_tmp/idl2srv ${_rtmbuild_pkg_dir}/scripts/idl2srv.py ${_rtmbuild_pkg_dir}/cmake/servicebridge.cmake)
  #
  foreach(_idl ${_idllist})
    execute_process(COMMAND ${_rtmbuild_pkg_dir}/scripts/idl2srv.py --interfaces -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_servicebridge_include_dirs}" --package-name=${PROJECT_NAME} OUTPUT_VARIABLE _interface
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_interface)
      string(REPLACE "\n" ";" _interface ${_interface})
      foreach(_comp ${_interface})
	message("[rtmbuild_genbridge] ${_idl} -> ${_comp}ROSBridgeComp")
        #add_custom_target(${PROJECT_NAME}_${_comp}ROSBridge_cpp DEPENDS ${_autogen} ) # cpp depends on compiled idl
	rtmbuild_add_executable("${_comp}ROSBridgeComp" "src_gen/${_comp}ROSBridge.cpp" "src_gen/${_comp}ROSBridgeComp.cpp")
        add_dependencies(${_comp}ROSBridgeComp DEPENDS ${PROJECT_NAME}_${_comp}ROSBridge_cpp ${PROJECT_NAME}_generate_messages_cpp) # comp depends on cpp
      endforeach(_comp)
    endif(_interface)
  endforeach(_idl)
  get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
  list(APPEND _old_clean_files ${PROJECT_SOURCE_DIR}/src_gen)
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
endmacro(rtmbuild_genbridge)
