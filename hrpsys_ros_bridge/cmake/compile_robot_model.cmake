###
### load old cmake
find_program(_export_collada_exe openhrp-export-collada)
find_program(_project_generator_exe openhrp-project-generator)

if(NOT (EXISTS ${_export_collada_exe} AND EXISTS ${_project_generator_exe}))
  message(WARNING "[compile_robot_model.cmake] Using deprecated compile_robot_model.cmake program")
  if(EXISTS ${PROJECT_SOURCE_DIR}/cmake/compile_robot_model_old.cmake) # for hrpsys_ros_bridge
    include(${PROJECT_SOURCE_DIR}/cmake/compile_robot_model_old.cmake)
  else()
    find_package(hrpsys_ros_bridge REQUIRED)
    if(hrpsys_ros_bridge_SOURCE_PREFIX)
      include(${hrpsys_ros_bridge_SOURCE_PREFIX}/cmake/compile_robot_model_old.cmake)
    else()
      include(${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge/cmake/compile_robot_model_old.cmake)
    endif()
  endif()
  return()
endif()
###

###
### convert OpenHRP3 vrml file into collada file, urdf+mesh file and euslisp file
###
macro(compile_collada_model daefile)
  # dummy set to wrl
  if(${CMAKE_VERSION} VERSION_LESS "2.8.12")
    get_filename_component(_dae_dir_name ${daefile} PATH)
  else()
    get_filename_component(_dae_dir_name ${daefile} DIRECTORY)
  endif()
  get_filename_component(_dae_file_name ${daefile} NAME_WE)
  compile_openhrp_model(${_dae_dir_name}/${_dae_file_name}.dae)
endmacro()

macro(compile_openhrp_model wrlfile)
  message(STATUS "[compile_robot_model.cmake] compile_openhrp_model(${wrlfile} ${ARGV})")

  # setup basic libraries
  include(FindPkgConfig)
  pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
  # create output directory
  set(_workdir ${PROJECT_SOURCE_DIR}/models)
  if(NOT EXISTS ${_workdir})
    file(MAKE_DIRECTORY ${_workdir})
  endif(NOT EXISTS ${_workdir})
  # parse agrument
  if("${ARGN}" STREQUAL "")
    get_filename_component(_name ${wrlfile} NAME_WE)
    set(_export_collada_option "")
    set(_conf_file_option "")
    set(_robothardware_conf_file_option "")
    set(_conf_dt_option "")
    set(_simulation_timestep_option "")
    set(_simulation_joint_properties_option "")
  else()
    set(_name ${ARGV1})
    get_export_collada_option(_export_collada_option ${ARGV})
    get_conf_file_option(_conf_file_option _robothardware_conf_file_option _conf_dt_option _simulation_timestep_option _simulation_joint_properties_option ${ARGV})
  endif()
  string(TOLOWER ${_name} _sname)
  set(_daefile "${_workdir}/${_name}.dae")
  set(_xmlfile "${_workdir}/${_name}.xml")
  set(_xmlfile_nosim "${_workdir}/${_name}_nosim.xml")
  set(_controller_config "${_workdir}/${_name}_controller_config.yaml")
  set(_yamlfile "${_workdir}/${_sname}.yaml")
  set(_lispfile "${_workdir}/${_sname}.l")
  set(_urdffile "${_workdir}/${_name}.urdf")

  # if yaml is not found, we do not use them
  if(NOT EXISTS ${_yamlfile})
    set(_yamlfile)
  endif()
  # if controller ocnfig is not augoteneretead, do not overwrite
  if(EXISTS ${_controller_config})
    file(READ ${_controller_config} _controller_config_text)
    string(REGEX MATCH "auto generate" _controller_config_auto_generated ${_controller_config_text})
    if(NOT _controller_config_auto_generated)
      set(_controller_config)
    endif()
  endif()

  # if wrl is not present, we stat from dae (in this case wrlfile and _daefile is same), see compile_collada_model daefile
  if(${wrlfile} STREQUAL ${_daefile})
    set(_wrlfile)
  else()
    set(_wrlfile ${wrlfile})
  endif()

  # use euscollada
  get_collada2eus(_collada2eus_exe)
  # use controller_config_converter
  get_controller_config_converter(_controller_config_converter)
  # get path to collada_to_urdf
  get_collada_to_urdf(_collada_to_urdf_exe)

  # ready to go
  message(STATUS " wrl file ${_wrlfile}")
  message(STATUS "yaml file ${_yamlfile}")
  message(STATUS " dae file ${_daefile}")
  message(STATUS "urdf file ${_urdffile}")
  message(STATUS "lisp file ${_lispfile}")
  message(STATUS " xml file ${_xmlfile}")
  message(STATUS "          ${_xmlfile_nosim}")
  message(STATUS "config file ${_controller_config}")
  #message(STATUS "Found ${_export_collada_exe}")
  #message(STATUS "Found ${_project_generator_exe}")
  #message(STATUS "Found ${_collada2eus_exe}")
  #message(STATUS "Found ${_collada_to_urdf_exe}")
  #message(STATUS "Found ${_controller_config_converter}")

  set(${_sname}_${PROJECT_NAME}_compile_all_target)

  # output collada (wrl -> collada )
  if(EXISTS ${_wrlfile})
    add_custom_command(OUTPUT ${_daefile}
      COMMAND ${_export_collada_exe} -i ${_wrlfile} -o ${_daefile} ${_export_collada_option}
      DEPENDS ${_wrlfile} ${_export_collada_exe})
  endif()
  add_custom_target(${_sname}_${PROJECT_NAME}_compile_dae DEPENDS ${_daefile})
  list(APPEND ${_sname}_${PROJECT_NAME}_compile_all_target ${_sname}_${PROJECT_NAME}_compile_dae)

  # output urdf files (collada -> urdf)
  set(_mesh_dir "${_workdir}/${_name}_meshes")
  set(_mesh_prefix "package://${PROJECT_NAME}/models/${_name}_meshes")
  add_custom_command(OUTPUT ${_urdffile} ${_mesh_dir}
    COMMAND ${_collada_to_urdf_exe} ${_daefile} --output_file ${_urdffile} -G -A --mesh_output_dir ${_mesh_dir} --mesh_prefix ${_mesh_prefix}
    DEPENDS ${_sname}_${PROJECT_NAME}_compile_dae)
  add_custom_target(${_sname}_${PROJECT_NAME}_compile_urdf DEPENDS ${_urdffile} ${_mesh_dir})
  list(APPEND ${_sname}_${PROJECT_NAME}_compile_all_target ${_sname}_${PROJECT_NAME}_compile_urdf)

  # output euslisp files (dae, yaml -> euslisp)
  if(EXISTS ${_collada2eus_exe})
    add_custom_command(OUTPUT ${_lispfile}
      COMMAND ${_collada2eus_option} ${_collada2eus_exe} ${_daefile} ${_yamlfile} ${_lispfile}
      DEPENDS ${_sname}_${PROJECT_NAME}_compile_dae ${_yamlfile})
    add_custom_target(${_sname}_${PROJECT_NAME}_compile_lisp DEPENDS ${_lispfile})
    list(APPEND ${_sname}_${PROJECT_NAME}_compile_all_target ${_sname}_${PROJECT_NAME}_compile_lisp)
  endif()

  if(_controller_config)
    # output controller config (yaml -> config) if yaml is not found write dummy files
    add_custom_command(OUTPUT ${_controller_config}
      COMMAND ${_controller_config_converter} ${_yamlfile} ${_controller_config}
      DEPENDS ${_yamlfile})
    add_custom_target(${_sname}_${PROJECT_NAME}_compile_conf DEPENDS ${_controller_config})
    list(APPEND ${_sname}_${PROJECT_NAME}_compile_all_target ${_sname}_${PROJECT_NAME}_compile_conf)
  endif()

  # project generator (collada -> xml)
  add_custom_command(OUTPUT ${_xmlfile}
    COMMAND ${_project_generator_exe} ${_wrlfile} ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,0,0,1,0 --integrate true --output ${_xmlfile} ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option} ${_simulation_timestep_option} ${_simulation_joint_properties_option}
    DEPENDS ${_sname}_${PROJECT_NAME}_compile_dae ${_gen_project_dep_files})
  add_custom_command(OUTPUT ${_xmlfile_nosim}
    COMMAND ${_project_generator_exe} ${_wrlfile} ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,0,0,1,0 --integrate false --output ${_xmlfile_nosim} ${_conf_file_option} ${_robothardware_conf_file_option} ${_conf_dt_option} ${_simulation_timestep_option} ${_simulation_joint_properties_option}
    DEPENDS ${_sname}_${PROJECT_NAME}_compile_dae ${_gen_project_dep_files})
  add_custom_target(${_sname}_${PROJECT_NAME}_compile_xml DEPENDS ${_xmlfile} ${_xmlfile_nosim})
  list(APPEND ${_sname}_${PROJECT_NAME}_compile_all_target ${_sname}_${PROJECT_NAME}_compile_xml)

  # run all target
  add_custom_target(${_sname}_${PROJECT_NAME}_compile)
  add_custom_target(${_sname}_${PROJECT_NAME}_compile_all ALL)
  add_dependencies(${_sname}_${PROJECT_NAME}_compile_all ${_sname}_${PROJECT_NAME}_compile)
  add_dependencies(${_sname}_${PROJECT_NAME}_compile ${${_sname}_${PROJECT_NAME}_compile_all_target})

  ### JUST FOR BACKWORD COMPATIBILITY https://github.com/start-jsk/rtmros_tutorials/blob/a84d63f599acc442a8dd2ad679b04bb6dc4e70c7/hrpsys_ros_bridge_tutorials/CMakeLists.txt#L102
  set(compile_robots ${${_sname}_${PROJECT_NAME}_compile_all_target})

endmacro(compile_openhrp_model)

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
  find_package(hrpsys_ros_bridge REQUIRED)
  if(hrpsys_ros_bridge_SOURCE_PREFIX)
    set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_SOURCE_PREFIX})
  else()
    set(hrpsys_ros_bridge_PACKAGE_PATH ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge)
  endif()
  message("Generate launch files for ${ROBOT}")
  message("  PROJECT_PKG_NAME = ${PROJECT_PKG_NAME}")
  message("        MODEL_FILE = ${MODEL_FILE}")
  message("             ROBOT = ${ROBOT}")
  message("hrpsys_ros_bridge_PACKAGE_PATH = ${hrpsys_ros_bridge_PACKAGE_PATH}")
  if(NOT EXISTS ${hrpsys_ros_bridge_PACKAGE_PATH}/scripts)
    message(FATAL_ERROR "hrpsys_ros_bridge_PACKAGE_PATH could not found")
  endif()

  # generate files
  set(${_sname}_generated_launch_euslisp_files)
  #   generate hrpsys_config.py to use unstable RTCs
  set(ROSBRIDGE_ARGS "    <arg name=\"BASE_LINK\" default=\"WAIST_LINK0\" />\n")
  set(USE_UNSTABLE_RTC "false")
  set(STARTUP_ARGS "    <arg name=\"HRPSYS_PY_PKG\" default=\"${PROJECT_PKG_NAME}\" if=\"$(arg USE_UNSTABLE_RTC)\"/>\n    <arg name=\"HRPSYS_PY_NAME\" default=\"${_sname}_hrpsys_config.py\" if=\"$(arg USE_UNSTABLE_RTC)\"/>")
  if ("${ARGV3}" STREQUAL "--use-unstable-hrpsys-config")
    set(USE_UNSTABLE_RTC "true")
    set(STARTUP_ARGS "    <arg name=\"HRPSYS_PY_ARGS\" default=\"--use-unstable-rtc\" />")
  elseif ("${ARGV3}" STREQUAL "--use-robot-hrpsys-config")
    set(USE_UNSTABLE_RTC "true")
  endif()
  #  generate startup.launch
  configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_startup.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_startup.launch)
  #  generate ros_bridge.launch
  configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_ros_bridge.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_ros_bridge.launch)
  add_custom_target(${_sname}_${PROJECT_NAME}_compile_launch DEPENDS ${PROJECT_SOURCE_DIR}/launch/${_sname}_startup.launch ${PROJECT_SOURCE_DIR}/launch/${_sname}_ros_bridge.launch)
  #  generate toplevel launch which includes ros_bridge.launch and startup.launch
  if (NOT "${ARGV3}" STREQUAL "--no-toplevel-launch")
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}.launch)
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default_robot_nosim.launch.in ${PROJECT_SOURCE_DIR}/launch/${_sname}_nosim.launch)
    add_custom_target(${_sname}_${PROJECT_NAME}_compile_top_launch DEPENDS ${PROJECT_SOURCE_DIR}/launch/${_sname}.launch ${PROJECT_SOURCE_DIR}/launch/${_sname}_nosim.launch)
    add_dependencies(${_sname}_${PROJECT_NAME}_compile DEPENDS ${_sname}_${PROJECT_NAME}_compile_top_launch)
  endif()
  #  generate euslisp robot-interface file
  if (NOT "${ARGV3}" STREQUAL "--no-euslisp")
    configure_file(${hrpsys_ros_bridge_PACKAGE_PATH}/scripts/default-robot-interface.l.in ${PROJECT_SOURCE_DIR}/build/${_sname}-interface.l)
    message(STATUS "Output ${PROJECT_SOURCE_DIR}/build/${_sname}-interface.l")
    add_custom_target(${_sname}_${PROJECT_NAME}_compile_eusif DEPENDS ${PROJECT_SOURCE_DIR}/build/${_sname}-interface.l)
    add_dependencies(${_sname}_${PROJECT_NAME}_compile DEPENDS ${_sname}_${PROJECT_NAME}_compile_eusif)
  endif()
endmacro ()


###
### utilities
###
macro(get_option_from_args _option_ret _option_name _separator)
  set(_arg_list ${ARGV})
  set(_arg_list2 ${ARGV})
  # remove arguments of this macro
  list(REMOVE_AT _arg_list 0 1 2)
  list(REMOVE_AT _arg_list2 0 1 2)
  set(_tmp_option "")
  foreach(anarg ${_arg_list2})
    if ("${anarg}" STREQUAL "${_option_name}")
        list(GET _arg_list 1 _tmp_option2)
	if(${_tmp_option2} MATCHES "#.*")
	  set(_tmp_option2 "\"${_tmp_option2}\"")
	endif()
        set(_tmp_option "${_tmp_option};${_separator};${_tmp_option2}")
    endif ("${anarg}" STREQUAL "${_option_name}")
    list(REMOVE_AT _arg_list 0)
  endforeach(anarg ${_arg_list2})
  set(${_option_ret} "${_tmp_option}")
endmacro(get_option_from_args _option_ret _option_name)

macro(get_conf_file_option _conf_file_option_ret _robothardware_conf_file_option_ret _conf_dt_option_ret _simulation_timestep_option_ret _simulation_joint_properties_option_ret)
  # set user valur
  get_option_from_args(${_conf_file_option_ret} --conf-file-option --conf-file-option ${ARGV})
  get_option_from_args(${_robothardware_conf_file_option_ret} --robothardware-conf-file-option --robothardware-conf-file-option ${ARGV})
  get_option_from_args(${_conf_dt_option_ret} --conf-dt-option --dt  ${ARGV})
  get_option_from_args(${_simulation_timestep_option_ret} --simulation-timestep-option --timestep ${ARGV})
  get_option_from_args(${_simulation_joint_properties_option_ret} --simulation-joint-properties-option --joint-properties ${ARGV})
  # set default value
  if(NOT EXISTS ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample/model/longfloor.wrl)
    message(FATAL_ERROR "could not find ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample/model/longfloor.wrl")
  endif()
  set(_conf_file_option_ret "${_conf_file_option_ret} OBJECT_MODELS:=${openhrp3_PREFIX}/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,0,0,1,0")
endmacro()

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

macro(get_proj_file_root_option _proj_file_root_option_ret)
  get_option_from_args(${_proj_file_root_option_ret} "--proj-file-root-option" "" "" "," ${ARGV})
endmacro()

macro(get_euscollada_option _euscollada_option_ret)
  get_option_from_args(${_euscollada_option_ret} "--euscollada-option" "" "" "" ${ARGV})
endmacro()

###
### get executables
###
# get path, option and dependending files to collada2eus
macro(get_collada2eus _collada2eus_exe)
  find_package(euscollada)
  set(${_collada2eus_exe} ${euscollada_PREFIX}/lib/euscollada/collada2eus)
  if(NOT EXISTS ${${_collada2eus_exe}})
    message(WARNING "could not find ${${_collada2eus_exe}}")
  endif()
endmacro()

# use controller_config_converter
macro(get_controller_config_converter _controller_config_converter)
  find_package(hrpsys_ros_bridge REQUIRED)
  if(hrpsys_ros_bridge_SOURCE_PREFIX)
    set(${_controller_config_converter} ${hrpsys_ros_bridge_SOURCE_PREFIX}/scripts/controller_config_converter.py)
  else()
    set(${_controller_config_converter} ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge/scripts/controller_config_converter.py)
  endif()
  if(NOT EXISTS ${${_controller_config_converter}})
    message(FATAL_ERROR "could not find ${${_controller_config_converter}}")
  endif()
endmacro()

# get path to collada_to_urdf
macro(get_collada_to_urdf _collada_to_urdf_exe)
  find_package(collada_urdf_jsk_patch QUIET)
  find_package(collada_urdf REQUIRED)
  if (collada_urdf_jsk_patch_FOUND)
    set(${_collada_to_urdf_exe} ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/collada_to_urdf)
  elseif (collada_urdf_FOUND)
    set(${_collada_to_urdf_exe} ${collada_urdf_PREFIX}/lib/collada_urdf/collada_to_urdf)
  endif (collada_urdf_jsk_patch_FOUND)
  if(NOT EXISTS "${${_collada_to_urdf_exe}}")
    message(FATAL_ERROR "could not find ${${_collada_to_urdf_exe}}")
  endif()
endmacro(get_collada_to_urdf _collada_to_urdf_exe)

