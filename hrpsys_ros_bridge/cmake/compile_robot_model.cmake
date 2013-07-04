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

macro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret)
  set(_arg_list ${ARGV})
  set(_conf_file_option "")
  set(_robothardware_conf_file_option "")
  foreach(anarg ${ARGV})
    # for _gen_project.launch
    if ("${anarg}" STREQUAL "-conf-file-option")
      list(GET _arg_list 1 _conf_file_option2)
      set(_conf_file_option "${_conf_file_option} --conf-file-option\ \"${_conf_file_option2}\"")
    endif ("${anarg}" STREQUAL "-conf-file-option")
    if ("${anarg}" STREQUAL "-robothardware-conf-file-option")
      list(GET _arg_list 1 _conf_file_option2)
      set(_robothardware_conf_file_option "${_robothardware_conf_file_option} --robothardware-conf-file-option\ \"${_conf_file_option2}\"")
    endif ("${anarg}" STREQUAL "-robothardware-conf-file-option")
    list(REMOVE_AT _arg_list 0)
  endforeach(anarg ${ARGV})
  if (NOT "${_conf_file_option}" STREQUAL "")
    set(_conf_file_option "CONF_FILE_OPTION:= ${_conf_file_option}")
  endif (NOT "${_conf_file_option}" STREQUAL "")
  if (NOT "${_robothardware_conf_file_option}" STREQUAL "")
    set(_robothardware_conf_file_option "ROBOTHARDWARE_CONF_FILE_OPTION:= ${_robothardware_conf_file_option}")
  endif (NOT "${_robothardware_conf_file_option}" STREQUAL "")
  set(${_conf_file_option_ret} ${_conf_file_option})
  set(${_robothardware_conf_file_option_ret} ${_robothardware_conf_file_option})
endmacro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret)

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
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${wrlfile})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile_nosim} ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${wrlfile})
  add_custom_target(${_name}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_daefile})
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
  else()
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option ${ARGV})
  endif()
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  string(TOLOWER ${_name} _name)
  set(_yamlfile "${_workdir}/${_name}.yaml")
  set(_lispfile "${_workdir}/${_name}.l")

  if(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_yamlfile} ${_lispfile} ${ARGV1}
      DEPENDS ${daefile})
  else(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_lispfile} ${ARGV1} || echo 'ok'
      DEPENDS ${daefile})
  endif(EXISTS ${_yamlfile})
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${daefile} OUTPUT:=${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${daefile})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${daefile} OUTPUT:=${_xmlfile_nosim} INTEGRATE:=false ${_conf_file_option} ${_robothardware_conf_file_option}
    DEPENDS ${daefile})
  add_custom_target(${_name}_compile ALL DEPENDS ${_lispfile} ${_xmlfile})
  list(APPEND compile_robots ${_name}_compile)
endmacro(compile_collada_model daefile)


