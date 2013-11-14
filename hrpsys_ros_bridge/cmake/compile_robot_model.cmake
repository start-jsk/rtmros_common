##
## define macros
##
macro(get_export_collada_option _export_collada_option_ret)
  set(_arg_list ${ARGV})
  set(_export_collada_option "")
  foreach(anarg ${ARGV})
    # for collada manipulator
    if ("${anarg}" STREQUAL "-a")
      if (NOT DEFINED _export_collada_option)
        set(_export_collada_option ${ARGV1})
      endif (NOT DEFINED _export_collada_option)
      list(GET _arg_list 1 _collada_arg_manipulator)
      set(_export_collada_option ${_export_collada_option} -a ${_collada_arg_manipulator})
    endif ("${anarg}" STREQUAL "-a")
    list(REMOVE_AT _arg_list 0)
  endforeach(anarg ${ARGV})
  set(${_export_collada_option_ret} ${_export_collada_option})
endmacro(get_export_collada_option _export_collada_option_ret)

macro(get_option_from_args _option_ret _option_name _separator _quater _ret_add_str)
  set(_arg_list ${ARGV})
  set(_arg_list2 ${ARGV})
  # remove arguments of this macro
  list(REMOVE_AT _arg_list 0 1 2)
  list(REMOVE_AT _arg_list2 0 1 2)
  set(_tmp_option "")
  foreach(anarg ${_arg_list2})
    if ("${anarg}" STREQUAL "${_option_name}")
      if (NOT "${_tmp_option}" STREQUAL "")
        list(GET _arg_list 1 _tmp_option2)
        set(_tmp_option "${_tmp_option}\ ${_separator}${_quater}${_tmp_option2}${_quater}")
      else(NOT "${_tmp_option}" STREQUAL "")
        list(GET _arg_list 1 _tmp_option2)
        set(_tmp_option "${_separator}${_quater}${_tmp_option2}${_quater}")
      endif (NOT "${_tmp_option}" STREQUAL "")
    endif ("${anarg}" STREQUAL "${_option_name}")
    list(REMOVE_AT _arg_list 0)
  endforeach(anarg ${_arg_list2})
  if (NOT "${_tmp_option}" STREQUAL "" AND NOT "${_ret_add_str}" STREQUAL "")
    set(_tmp_option "${_ret_add_str}${_tmp_option}")
  endif (NOT "${_tmp_option}" STREQUAL "" AND NOT "${_ret_add_str}" STREQUAL "")
  set(${_option_ret} "${_tmp_option}")
endmacro(get_option_from_args _option_ret _option_name)

macro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret _conf_dt_option_ret)
  get_option_from_args(${_conf_file_option_ret} "--conf-file-option" "--conf-file-option\ " ' "CONF_FILE_OPTION:=" ${ARGV})
  get_option_from_args(${_robothardware_conf_file_option_ret} "--robothardware-conf-file-option" "\ --robothardware-conf-file-option\ " ' "ROBOTHARDWARE_CONF_FILE_OPTION:=" ${ARGV})
  get_option_from_args(${_conf_dt_option_ret} "--conf-dt-option" "\ --dt\ " ' "CONF_DT_OPTION:=" ${ARGV})
endmacro()

macro(get_proj_file_root_option _proj_file_root_option_ret)
  get_option_from_args(${_proj_file_root_option_ret} "--proj-file-root-option" "" "" "," ${ARGV})
endmacro()

macro(get_euscollada_option _euscollada_option_ret)
  get_option_from_args(${_euscollada_option_ret} "--euscollada-option" "" "" "" ${ARGV})
endmacro()

set(_corba_port 2889)
macro(compile_openhrp_model wrlfile)
  math(EXPR _corba_port "${_corba_port}+1")
  message("compile openhrp model ${wrlfile} at port ${_corba_port}")
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if(NOT EXISTS ${_workdir})
    file(MAKE_DIRECTORY ${_workdir})
  endif(NOT EXISTS ${_workdir})
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
    set(_export_collada_option "")
    set(_conf_file_option "")
    set(_robothardware_conf_file_option "")
    set(_conf_dt_option "")
  else()
    set(_name ${ARGV1})
    get_export_collada_option(_export_collada_option ${ARGV})
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option _conf_dt_option ${ARGV})
  endif()
  set(_daefile "${_workdir}/${_name}.dae")
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  string(TOLOWER ${_name} _sname)
  set(_yamlfile "${_workdir}/${_sname}.yaml")
  set(_lispfile "${_workdir}/${_sname}.l")
  # rtm-naming
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(openrtm_aist)
    set(_rtm_naming_exe ${openrtm_aist_PACKAGE_PATH}/bin/rtm-naming)
  else()
    find_package(PkgConfig)
    pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
    set(openrtm_aist_PACKAGE_PATH ${openrtm_aist_SOURCE_DIR})
    set(_rtm_naming_exe ${openrtm_aist_PREFIX}/lib/openrtm_aist/bin/rtm-naming)
  endif()
  # use euscollada
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(euscollada)
    set(_collada2eus_exe ${euscollada_PACKAGE_PATH}/bin/collada2eus)
  else()
    find_package(euscollada)
    set(euscollada_PACKAGE_PATH ${euscollada_SOURCE_DIR})
    set(_collada2eus_exe ${euscollada_PREFIX}/lib/euscollada/collada2eus)
  endif()
  # check if binary exists
  if(NOT EXISTS ${_rtm_naming_exe})
    message(FATAL_ERROR "-- ${_rtm_naming_exe} not found")
  endif()
  if(NOT EXISTS ${_collada2eus_exe})
    message(AUTHOR_WARNING "-- ${_collada2eus_exe} not found")
  else()
    #
    set(_euscollada_dep_files ${_collada2eus_exe} ${euscollada_PACKAGE_PATH}/src/euscollada-robot.l)
    if(EXISTS ${_yamlfile})
      add_custom_command(OUTPUT ${_lispfile}
        COMMAND ${_collada2eus_exe} ${_daefile} ${_yamlfile} ${_lispfile}
        DEPENDS ${_daefile} ${_yamlfile} ${_euscollada_dep_files})
    else(EXISTS ${_yamlfile})
      add_custom_command(OUTPUT ${_lispfile}
        COMMAND ${_collada2eus_exe} ${_daefile} ${_lispfile}
        DEPENDS ${_daefile} ${_euscollada_dep_files})
    endif(EXISTS ${_yamlfile})
  endif()
  # use export-collada
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(openhrp3)
    set(_export_collada_exe ${openhrp3_PACKAGE_PATH}/bin/export-collada)
  else()
    #set(openhrp3_PACKAGE_PATH ${openhrp3_SOURCE_DIR})
    set(_export_collada_exe ${CATKIN_DEVEL_PREFIX}/lib/openhrp3/export-collada)
  endif()
  if(EXISTS ${_export_collada_exe})
    message("using export-collada to convert models ${_export_collada_exe}")
  else()
    # when openhrp3 is catkin installed
    set(_export_collada_exe )
    message("assuming export-collada is already compiled")
  endif()
  if(EXISTS ${_export_collada_exe})
    add_custom_command(OUTPUT ${_daefile}
      COMMAND ${_export_collada_exe} -i ${wrlfile} -o ${_daefile} ${_export_collada_option}
      DEPENDS ${wrlfile} ${_export_collada_exe})
  endif()
  # use _gen_project.launch
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(hrpsys)
    rosbuild_find_ros_package(hrpsys_tools)
  else()
    find_package(hrpsys)
    find_package(hrpsys_tools)
    if(EXISTS ${hrpsys_SOURCE_DIR})
      set(hrpsys_PACKAGE_PATH ${hrpsys_SOURCE_DIR})
    else()
      set(hrpsys_PACKAGE_PATH ${hrpsys_PREFIX}/lib/hrpsys)
    endif()
    if(EXISTS ${hrpsys_tools_SOURCE_DIR})
      set(hrpsys_tools_PACKAGE_PATH ${hrpsys_tools_SOURCE_DIR})
    else()
      set(hrpsys_tools_PACKAGE_PATH ${hrpsys_tools_PREFIX}/share/hrpsys_tools)
    endif()
  endif()
  if(EXISTS ${hrpsys_PACKAGE_PATH}/bin/ProjectGenerator)
    set(_gen_project_dep_files ${hrpsys_PACKAGE_PATH}/bin/ProjectGenerator ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch)
  else()
    # when hrpsys is catkin installed
    set(_gen_project_dep_files)
    message("assuming hrpsys/ProjectGenerator is already compiled")
  endif()
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND ${_rtm_naming_exe} ${_corba_port}
    COMMAND rostest -t ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch CORBA_PORT:=${_corba_port} INPUT:=${wrlfile} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option}
    COMMAND pkill -KILL -f "omniNames -start ${_corba_port}" || echo "no process to kill"
    DEPENDS ${daefile} ${_gen_project_dep_files})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND ${_rtm_naming_exe} ${_corba_port}
    COMMAND rostest -t ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch CORBA_PORT:=${_corba_port} INPUT:=${wrlfile} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option}
    COMMAND pkill -KILL -f "omniNames -start ${_corba_port}" || echo "no process to kill"
    DEPENDS ${daefile} ${_gen_project_dep_files} ${_xmlfile})
  if(EXISTS ${_collada2eus_exe})
    add_custom_target(${_sname}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_xmlfile_nosim} ${_daefile})
  else()
    add_custom_target(${_sname}_compile DEPENDS ${_xmlfile} ${_xmlfile_nosim} ${_daefile})
  endif()
  ## make sure to kill nameserver
  add_custom_command(OUTPUT ${_sname}_compile_cleanup
    COMMAND echo "pkill -KILL -f omniNames -start ${_corba_port} for compile_openhrp_model"
    COMMAND echo "pkill -KILL -f omniNames\\ -start\\ ${_corba_port}" >  ./pkill-omninames-${_corba_port}.sh
    COMMAND sh ./pkill-omninames-${_corba_port}.sh || echo "no process to kill"
    COMMAND ps -C omniNames || true
    DEPENDS  ${_sname}_compile
    VERBATIM)
  add_custom_target(${_sname}_compile_all ALL DEPENDS ${_sname}_compile_cleanup)
  get_directory_property(_current_directory_properties ADDITIONAL_MAKE_CLEAN_FILES)
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${_workdir}/${_name}.conf;${_workdir}/${_name}.RobotHardware.conf;${_workdir}/${_name}_nosim.conf;${_workdir}/${_name}_nosim.RobotHardware.conf;${_current_directory_properties}")

  list(APPEND compile_robots ${_sname}_compile)
endmacro(compile_openhrp_model)

macro(compile_collada_model daefile)
  math(EXPR _corba_port "${_corba_port}+1")
  message("compile collada model ${daefile} at port ${_corba_port}")
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if(NOT EXISTS ${_workdir})
    file(MAKE_DIRECTORY ${_workdir})
  endif(NOT EXISTS ${_workdir})
  get_filename_component(_name ${daefile} NAME_WE)
  if("${ARGN}" STREQUAL "")
    set(_conf_file_option "")
    set(_robothardware_conf_file_option "")
    set(_conf_dt_option "")
    set(_proj_file_root_option "")
    set(_euscollada_option "")
  else()
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option _conf_dt_option ${ARGV})
    get_proj_file_root_option(_proj_file_root_option ${ARGV})
    get_euscollada_option(_euscollada_option ${ARGV})
  endif()
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  string(TOLOWER ${_name} _sname)
  set(_yamlfile "${_workdir}/${_sname}.yaml")
  set(_lispfile "${_workdir}/${_sname}.l")
  # rtm-naming
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(openrtm_aist)
    set(_rtm_naming_exe ${openrtm_aist_PACKAGE_PATH}/bin/rtm-naming)
  else()
    find_package(PkgConfig)
    pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
    set(openrtm_aist_PACKAGE_PATH ${openrtm_aist_SOURCE_DIR})
    set(_rtm_naming_exe ${openrtm_aist_PREFIX}/lib/openrtm_aist/bin/rtm-naming)
  endif()
  # use euscollada
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(euscollada)
    set(_collada2eus_exe ${euscollada_PACKAGE_PATH}/bin/collada2eus)
    set(_euscollada_dep_files ${euscollada_PACKAGE_PATH}/bin/collada2eus ${euscollada_PACKAGE_PATH}/src/euscollada-robot.l)
  else()
    find_package(euscollada)
    set(euscollada_PACKAGE_PATH ${euscollada_SOURCE_DIR})
    set(_collada2eus_exe ${euscollada_PREFIX}/lib/euscollada/collada2eus)
  endif()
  # check if binary exists
  if(NOT EXISTS ${_rtm_naming_exe})
    message(FATAL_ERROR "-- ${_rtm_naming_exe} not found")
  endif()
  if(NOT EXISTS ${_collada2eus_exe})
    message(AUTHOR_WARNING "-- ${_collada2eus_exe} not found")
  else()
    if(EXISTS ${_yamlfile})
      add_custom_command(OUTPUT ${_lispfile}
        COMMAND ${_collada2eus_exe} ${daefile} ${_yamlfile} ${_lispfile} ${_euscollada_option} ||  echo "[WARNING] ### Did not run collada2eus for ${_lispfile}"
        DEPENDS ${daefile} ${_euscollada_dep_files})
    else(EXISTS ${_yamlfile})
      add_custom_command(OUTPUT ${_lispfile}
        COMMAND ${_collada2eus_exe} ${daefile} ${_lispfile} ${_euscollada_option} || echo "[WARNING] ### Did not run collada2eus $for {_lispfile}"
        DEPENDS ${daefile} ${_euscollada_dep_files})
    endif(EXISTS ${_yamlfile})
  endif()
  # use _gen_project.launch
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(hrpsys)
    rosbuild_find_ros_package(hrpsys_tools)
  else()
    find_package(hrpsys)
    find_package(hrpsys_tools)
    if(EXISTS ${hrpsys_SOURCE_DIR})
      set(hrpsys_PACKAGE_PATH ${hrpsys_SOURCE_DIR})
    else()
      set(hrpsys_PACKAGE_PATH ${hrpsys_PREFIX}/lib/hrpsys)
    endif()
    if(EXISTS ${hrpsys_tools_SOURCE_DIR})
      set(hrpsys_tools_PACKAGE_PATH ${hrpsys_tools_SOURCE_DIR})
    else()
      set(hrpsys_tools_PACKAGE_PATH ${hrpsys_tools_PREFIX}/share/hrpsys_tools)
    endif()
  endif()
  if(EXISTS ${hrpsys_PACKAGE_PATH}/bin/ProjectGenerator)
    set(_gen_project_dep_files ${hrpsys_PACKAGE_PATH}/bin/ProjectGenerator ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch)
  else()
    # when hrpsys is catkin installed
    set(_gen_project_dep_files)
    message("assuming hrpsys/ProjectGenerator is already compiled")
  endif()
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND ${_rtm_naming_exe} ${_corba_port}
    COMMAND rostest -t ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch CORBA_PORT:=${_corba_port} INPUT:=${daefile}${_proj_file_root_option} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option}
    COMMAND pkill -KILL -f "omniNames -start ${_corba_port}" || echo "no process to kill"
    DEPENDS ${daefile} ${_gen_project_dep_files})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND ${_rtm_naming_exe} ${_corba_port}
    COMMAND rostest -t ${hrpsys_tools_PACKAGE_PATH}/launch/_gen_project.launch CORBA_PORT:=${_corba_port} INPUT:=${daefile}${_proj_file_root_option} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option}
    COMMAND pkill -KILL -f "omniNames -start ${_corba_port}" || echo "no process to kill"
    DEPENDS ${daefile} ${_gen_project_dep_files} ${_xmlfile})
  if(EXISTS ${_collada2eus_exe})
    add_custom_target(${_sname}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_xmlfile_nosim})
  else()
    add_custom_target(${_sname}_compile DEPENDS ${_xmlfile} ${_xmlfile_nosim})
  endif()
  ## make sure to kill nameserver
  add_custom_command(OUTPUT ${_sname}_compile_cleanup
    COMMAND echo "pkill -KILL -f omniNames -start ${_corba_port} for compile_collada_model"
    COMMAND echo "pkill -KILL -f omniNames\\ -start\\ ${_corba_port}" >  ./pkill-omninames-${_corba_port}.sh
    COMMAND sh ./pkill-omninames-${_corba_port}.sh || echo "no process to kill"
    COMMAND ps -C omniNames || true
    DEPENDS  ${_sname}_compile
    VERBATIM)
  add_custom_target(${_sname}_compile_all ALL DEPENDS ${_sname}_compile_cleanup)
  get_directory_property(_current_directory_properties ADDITIONAL_MAKE_CLEAN_FILES)
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${_workdir}/${_name}.conf;${_workdir}/${_name}.RobotHardware.conf;${_workdir}/${_name}_nosim.conf;${_workdir}/${_name}_nosim.RobotHardware.conf;${_current_directory_properties}")

  list(APPEND compile_robots ${_sname}_compile)
endmacro(compile_collada_model daefile)

macro (generate_default_launch_eusinterface_files wrlfile project_pkg_name)
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
  else()
    set(_name ${ARGV2})
  endif()
  string(TOLOWER ${_name} _sname)
  set(PROJECT_PKG_NAME ${project_pkg_name})
  set(MODEL_FILE ${wrlfile})
  set(ROBOT ${_name})
  set(robot ${_sname})
  rosbuild_find_ros_package(hrpsys_ros_bridge)
  set(${_sname}_generated_launch_euslisp_files)
  configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_startup.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_startup.launch)
  list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/launch/${_sname}_startup.launch)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/models/${_name}_controller_config.yaml)
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_controller_config.yaml.in ${PROJECT_SOURCE_DIR}/models/${_name}_controller_config.yaml)
    list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/models/${_name}_controller_config.yaml)
  endif()
  configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_ros_bridge.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_ros_bridge.launch)
  list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/launch/${_sname}_ros_bridge.launch)
  if (NOT "${ARGV3}" STREQUAL "--no-toplevel-launch")
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}.launch)
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_nosim.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_nosim.launch)
    list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/launch/${_sname}.launch)
    list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/launch/${_sname}_nosim.launch)
  endif()
  if (NOT "${ARGV3}" STREQUAL "--no-euslisp")
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default-robot-interface.l.in ${PROJECT_SOURCE_DIR}/build/${_sname}-interface.l)
    list(APPEND ${_sname}_generated_launch_euslisp_files ${PROJECT_SOURCE_DIR}/build/${_sname}-interface.l)
  endif()
  get_directory_property(_current_directory_properties ADDITIONAL_MAKE_CLEAN_FILES)
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${${_sname}_generated_launch_euslisp_files};${_current_directory_properties}")
endmacro ()
