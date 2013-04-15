##
## define macros
##
macro(compile_openhrp_model wrlfile)
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
    set(_export_collada_option "")
  else()
    set(_name ${ARGV1})
    set(_export_collada_option ${ARGN})
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
    COMMAND rostest -t hrpsys _gen_project.launch INPUT:=${wrlfile} OUTPUT:=${_xmlfile}
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


