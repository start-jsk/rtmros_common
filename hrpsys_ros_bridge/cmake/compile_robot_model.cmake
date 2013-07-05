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

macro(get_option_from_args _option_ret _option_name _separator _quater)
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
        set(_tmp_option "${_tmp_option}${_separator}${_quater}${_tmp_option2}${_quater}\ ")
      else(NOT "${_tmp_option}" STREQUAL "")
        list(GET _arg_list 1 _tmp_option2)
        set(_tmp_option "${_separator}${_quater}${_tmp_option2}${_quater}\ ")
      endif (NOT "${_tmp_option}" STREQUAL "")
    endif ("${anarg}" STREQUAL "${_option_name}")
    list(REMOVE_AT _arg_list 0)
  endforeach(anarg ${_arg_list2})
  set(${_option_ret} "${_tmp_option}")
endmacro(get_option_from_args _option_ret _option_name)

macro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret)
  set(_conf_file_option "_conf_file_option")
  set(_robothardware_conf_file_option "_robothardware_conf_file_option")
  get_option_from_args(${_conf_file_option} "-conf-file-option" "--conf-file-option\ " ' ${ARGV})
  get_option_from_args(${_robothardware_conf_file_option} "-robothardware-conf-file-option" "\ --robothardware-conf-file-option\ " ' ${ARGV})
  if (NOT "${_conf_file_option}" STREQUAL "")
    set(_conf_file_option "CONF_FILE_OPTION:=${_conf_file_option}")
  endif (NOT "${_conf_file_option}" STREQUAL "")
  if (NOT "${_robothardware_conf_file_option}" STREQUAL "")
    set(_robothardware_conf_file_option "ROBOTHARDWARE_CONF_FILE_OPTION:=${_robothardware_conf_file_option}")
  endif (NOT "${_robothardware_conf_file_option}" STREQUAL "")
  set(${_conf_file_option_ret} ${_conf_file_option})
  set(${_robothardware_conf_file_option_ret} ${_robothardware_conf_file_option})
endmacro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret)

macro(get_proj_file_root_option _proj_file_root_option_ret)
  set(_proj_file_root_option "_proj_file_root_option")
  get_option_from_args(${_proj_file_root_option} "-proj-file-root-option" "" "" ${ARGV})
  set(${_proj_file_root_option_ret} ${_proj_file_root_option})
  if (NOT "${_proj_file_root_option}" STREQUAL "")
    set(_proj_file_root_option ",${_proj_file_root_option}")
  endif (NOT "${_proj_file_root_option}" STREQUAL "")
endmacro(get_proj_file_root_option _proj_file_root_option_ret)

macro(get_euscollada_option _euscollada_option_ret)
  set(_euscollada_option "_euscollada_option")
  get_option_from_args(${_euscollada_option} "-euscollada-option" "" "" ${ARGV})
  set(${_euscollada_option_ret} ${_euscollada_option})
endmacro(get_euscollada_option _euscollada_option_ret)

macro(compile_openhrp_model wrlfile)
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if(NOT EXISTS ${_workdir})
    file(MAKE_DIRECTORY ${_workdir})
  endif(NOT EXISTS ${_workdir})
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
    set(_export_collada_option "")
    set(_conf_file_option "")
    set(_robothardware_conf_file_option "")
  else()
    set(_name ${ARGV1})
    get_export_collada_option(_export_collada_option ${ARGV})
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option ${ARGV})
  endif()
  set(_daefile "${_workdir}/${_name}.dae")
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  string(TOLOWER ${_name} _name)
  set(_yamlfile "${_workdir}/${_name}.yaml")
  set(_lispfile "${_workdir}/${_name}.l")
  if(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${_daefile} ${_yamlfile} ${_lispfile}
      DEPENDS ${_daefile} ${_yamlfile})
  else(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${_daefile} ${_lispfile}
      DEPENDS ${_daefile})
  endif(EXISTS ${_yamlfile})
  add_custom_command(OUTPUT ${_daefile}
    COMMAND rosrun openhrp3 export-collada -i ${wrlfile} -o ${_daefile} ${_export_collada_option}
    DEPENDS ${wrlfile})
  #message("AA rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option}")
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${wrlfile})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${wrlfile})
  add_custom_target(${_name}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_xmlfile_nosim} ${_daefile})
  list(APPEND compile_robots ${_name}_compile)
endmacro(compile_openhrp_model)

macro(compile_collada_model daefile)
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if(NOT EXISTS ${_workdir})
    file(MAKE_DIRECTORY ${_workdir})
  endif(NOT EXISTS ${_workdir})
  get_filename_component(_name ${daefile} NAME_WE)
  if("${ARGN}" STREQUAL "")
    set(_conf_file_option "")
    set(_robothardware_conf_file_option "")
    set(_proj_file_root_option "")
    set(_euscollada_option "")
  else()
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option ${ARGV})
    get_proj_file_root_option(_proj_file_root_option ${ARGV})
    get_euscollada_option(_euscollada_option ${ARGV})
  endif()
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  string(TOLOWER ${_name} _name)
  set(_yamlfile "${_workdir}/${_name}.yaml")
  set(_lispfile "${_workdir}/${_name}.l")

  if(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_yamlfile} ${_lispfile} ${_euscollada_option} ||  echo "[WARNING] ### Did not run collada2eus for ${_lispfile}"
      DEPENDS ${daefile})
  else(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_lispfile} ${_euscollada_option} || echo "[WARNING] ### Did not run collada2eus $for {_lispfile}"
      DEPENDS ${daefile})
  endif(EXISTS ${_yamlfile})
  # start name server
  execute_process(COMMAND hostname OUTPUT_VARIABLE _hostname OUTPUT_STRIP_TRAILING_WHITESPACE)
  add_custom_command(OUTPUT omninames-${_hostname}.log COMMAND rosrun openrtm rtm-naming 2888)
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND rostest -t hrpsys _gen_project.launch CORBA_PORT:=2888 INPUT:=${daefile}${_proj_file_root_option} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${daefile} omninames-${_hostname}.log)
  #message("rostest -t hrpsys _gen_project.launch CORBA_PORT:=2888 INPUT:=${daefile}${_proj_file_root_option} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option}")
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND rostest -t hrpsys _gen_project.launch CORBA_PORT:=2888 INPUT:=${daefile}${_proj_file_root_option} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${daefile} omninames-${_hostname}.log)
  add_custom_target(${_name}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_xmlfile_nosim})
  ## kill nameserver
  add_custom_command(OUTPUT ${_name}_compile_cleanup
    COMMAND -pkill -KILL -f "omniNames -start 2888" || echo "no process to kill"
    DEPENDS  ${_name}_compile
    VERBATIM)
  add_custom_target(${_name}_compile_all ALL DEPENDS ${_name}_compile_cleanup)

  list(APPEND compile_robots ${_name}_compile)
endmacro(compile_collada_model daefile)


