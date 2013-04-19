##
## define macros
##
macro(compile_openhrp_model wrlfile)
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
    set(_export_collada_option "")
    set(_conf_file_option "")
  else()
    set(_name ${ARGV1})
    set(_arg_list ${ARGV})
    set(_export_collada_option "")
    set(_conf_file_option "")
    foreach(anarg ${ARGV})
      # for collada manipulator
      if ("${anarg}" STREQUAL "-a")
        if (NOT DEFINED _export_collada_option)
          set(_export_collada_option ${ARGV1})
        endif (NOT DEFINED _export_collada_option)
        list(GET _arg_list 1 _collada_arg_manipulator)
        set(_export_collada_option ${_export_collada_option} -a ${_collada_arg_manipulator})
      endif ("${anarg}" STREQUAL "-a")
      # for _gen_project.launch
      if ("${anarg}" STREQUAL "-conf-file-option")
        list(GET _arg_list 1 _conf_file_option)
        set(_conf_file_option "CONF_FILE_OPTION:=--conf-file-option \"${_conf_file_option}\"")
        message("FIND CONF FILE OPTION ${_conf_file_option} ${_name}")
      endif ("${anarg}" STREQUAL "-conf-file-option")
      list(REMOVE_AT _arg_list 0)
    endforeach(anarg ${ARGV})
  endif()
  set(_daefile "${_workdir}/${_name}.dae")
  set(_xmlfile "${_workdir}/${_name}.xml")
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
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile} ${_conf_file_option}
    DEPENDS ${wrlfile})
  add_custom_target(${_name}_compile DEPENDS ${_lispfile} ${_xmlfile} ${_daefile})
  list(APPEND compile_robots ${_name}_compile)
endmacro(compile_openhrp_model)

macro(compile_collada_model daefile)
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  get_filename_component(_name ${daefile} NAME_WE)
  set(_xmlfile "${_workdir}/${_name}.xml")
  string(TOLOWER ${_name} _name)
  set(_yamlfile "${_workdir}/${_name}.yaml")
  set(_lispfile "${_workdir}/${_name}.l")

  if(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_yamlfile} ${_lispfile}
      DEPENDS ${daefile})
  else(EXISTS ${_yamlfile})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND rosrun euscollada collada2eus ${daefile} ${_lispfile} || echo 'ok'
      DEPENDS ${daefile})
  endif(EXISTS ${_yamlfile})
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${daefile} OUTPUT:=${_xmlfile}
    DEPENDS ${daefile})
  add_custom_target(${_name}_compile DEPENDS ${_lispfile} ${_xmlfile})
  list(APPEND compile_robots ${_name}_compile)
endmacro(compile_collada_model daefile)


