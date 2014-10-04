^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtmbuild
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2014-10-04)
------------------

1.2.4 (2014-09-08)
------------------

1.2.3 (2014-09-03)
------------------
* fix rtmbuild.cmake
* Contributors: terasawa

1.2.2 (2014-08-31)
------------------

1.2.1 (2014-06-23)
------------------
* (idl2srv.py) : Check pointer or not ;; At previous revision, non-pointer variables are deleted
* (idl2srv.py) : use random string for idl2srv generated file
* (package.xml files) Update repository URLs
* Contributors: Isaac Isao Saito, Kei Okada, Shunichi Nozawa

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0 for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------

1.0.11 (2014-04-16)
-------------------
* compile idl files at once, omniidl -bpython support compile multiple idl
* Contributors: Kei Okada

1.0.10 (2014-04-15)
-------------------

1.0.9 (2014-04-14)
------------------

1.0.8 (2014-04-14)
------------------
* fix to work on rosbuild
* fix for installed test, use rtmbuild_SOURCE_PREFIX to check if devel/src environment
* rtmbuild: install test directory so that others can uses this
* rtmbuild: rewrite rtmbulid.cmake
* rtmbuild: cleanup catkin.cmake and put everything into CMakeLists.txt
* Contributors: Kei Okada

1.0.7 (2014-03-20)
------------------
* add depends from RTMBUILD__rm_idl2srv to _generate_messages_cpp, so that we can generate message in the beginning
* rtmbuild: check if openrtm_aist_PREFIX/lib/openrtm_aist/bin exists, if not, try rospack find, this happens when you compile your catkin code over rosbuild compiled rtmbuild (that never happens?)
* rtmbuild: add test program to see if rtmbuild works
* (rtmbuild.cmake, idl2srv.py) fix for rtmbuild/deb environment
* (rtmbuild.cmake) display variables on both rtmbuild and cmake
* (rtmbuild.cmake) add find_package(PkgConfig)
* Wrong catkin macro (CATKIN-DEPENDS to CATKIN_DEPENDS).
* Contributors: Kei Okada

1.0.6
-----
* rtmros_common-1.0.6
* (hrpsys_tools, rosnode_rtc, rtmbuild) maintainer name adjusted to the one used in other places

1.0.5
-----
* "rtmros_common-1.0.5"
* fix typo: add cmake_modules to depends
* add cmake_modules to depends
* add cmake_modules to build/run depends

1.0.4
-----
* "rtmros_common-1.0.4"
* do not copmpile ild file parallely, hope this fixxes Issue 238

1.0.3
-----
* "rtmros_common-1.0.3"

1.0.2
-----
* "rtmros_common-1.0.2"
* rename _include_dirs -> _servicebridge_include_dirs

1.0.1
-----
* rtmros_common) 1.0.1
* rtmros_common) Revert wrong commit r5588
* rtmros_common) 1.0.1
* set output_python_dir to // for catkin
* fix typo and debug code for catkin
* fix : rosbuild must be called after generatng msg/srv file from idl
* add debug message
* revert r5463, which is not effective
* add dependencies rospack_genmsg, rospack_gensrv to rtmbuild_add_{executable,librariey}
* fix debug message
* invoke rosbuild_init before rtmbuild_genbridge_init so to set PACKAGE_NAME
* add openrtm_aist_LIBRARIES to target_link_libraries
* set package-name for idl2srv.py
* fix openhrp3.1 is not required for rtmbuild, that's optional
* add run_depends to message_generation and rtmbuild.cmake requires message_generation
* use openrtm_aist_{INCLUDE_DIRS,LIBRARY_DIRS}
* fix typo
* fix rtmbuild.cmake for catkin
* remove debug code : clean up cmake dependency
* celan up cmake dependency
* add dependencies from rtmbulid_PROJECT_NAME_genbridge -> rtmbuild_PROJECT_NAME_genidl
* fix typo FTRUE -> TRUE
* use use_catkin variable to check if catkin compiled
* check if CATKIN_BUILD_BINARY_PACKAGE for build
* add omniorb to build_depend
* add pkg-config to depend
* fix set pkg_dir rtmbuild_init
* check if idl2srv fails
* preserve directory permissions for install script/ directory
* add openrtm_aist_pkg_dir to PATH
* use message instead of message_warn
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* remove rosun process from catkin/rtmbuil
* create idl_gen/{cpp,lib} direcotry for catkin_package
* fix _ ->  catkin does not uses OPENRTM_INCLUDE_DIRS/OPENHRP_LIBRARY_DIRS/OPENRTM_LIBRARY_DIRS/OPENHRP_LIBRARY_DIRS
* use _generate_messages_cpp
* use PROJECT_NAME instead of _project and _prefix
* rtmbulid_genidl : checks if user already called add_message_files and append the information, use PROJECT_NAME instaed of _project
* pkg_check_modles could not find catkin package on compile time
* add depends from exported targets to exe
* change target name rtmbuild_genidl/rtmbuild_genbridge -> rtmbuild__genidl/rtmbuild__genbridge
* do not need link_directories
* add install, depends ROSBridge.cpp to generate_message_cpp
* support CATKIN
* add message_generation on depends
* use _rtmbuild_pkg_dir instead of rtmbuild_PACKAGE_PATH
* fix for catkin
* update catkin.cmake
* add omniorb, blas, lapack to run_depend
* use USE_ROSBUILD
* _rosbuild->message, remove rosrun
* move servicebridge.cmake idl2srv.py from openrtm_ros_bridge to rtmbuild
* remove debug message
* change INCLUDE_DIRS from {prefix}/idl_gen/cpp/{prefix}/idl to {prefix}/idl_gen/cpp, so need to include <prefix/idl/hoge.h> instead of <hoge.h>, Issue 137
* openhrp3.1.pc requires blas and lapack Issue 145
* fix openrtm -> openrtm_aist, due to split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* catkinize package : add catkin.cmake and package.xml
* rtm-skelwrapper does not overwrite existing files that breaks makefile dependency, so remove generated file before rtm-skelwrapper. add depends on stub.h and skel.h
* fix for catkin packages Fixes Issue 78
* update manifest for fuerte
* add current PKG_CONFIG_PATH
* add sample shell scripts, cmake functions moved
* add ROSBUILD_GENERATED_{MSG/SRV}_FILES in rtmbuild_genbridge
* moved idl2srv.py to openrtm_ros_bridge package
* do not rm idl file
* removing generated file in idl2srv.py causes deleting necessary files, when the file is shared by another idl files. so do not remove the file in idl2srv.py, remove the generated file from rtmbuild.cmake. also we generate files under /tmp/idl2srv/<project> since different project may share same idl file
* put generated code under src -> src_gen
* do not remove tmpdir for parallel make
* check for duplicate output rule that causes error
* update to support rtmbuild_genbridge, see fmk_ros_bridge and hrpsys for best practice
* fixed the code to get rtc default idl directory
* try to add a library path, copied from omniild main program
* add try-catch block for _omniidl.compile newer version
* added dataport_ros_bridge package, which convert ROS/topic -> RTM/dataport
* added rtm-ros data port converter module, rtmros-data-bridge.py
* use rtm-config --cflags to get openrtm include path
* use rtm-config --cflags to get -I flags for genidl
* comment out adding /opt/grx to PKG_CONFIG_PATH
* use /opt/grx/lib's pkgconfig if exist
* add libomniorb4-dev and rosdep from rtmbuild
* changed to not generate unused msgs in Operations
* added StringMultiArray, ignore attributes in interface
* add namespace of msg/srv
* change to use omniidl_be.cxx.types.basic_map for c++ builtin types
* add type conversion code for 32/64bit environment
* add multiarray type for multi-dimensional sequence/array
* parameterize interface name of service, (port_name), and small fixes
* use static_cast to convert int <=> enum, in idl2srv.py
* update idl2srv.py to convert array type in idl / boost::array in ROS
* fix idl2srv.py to convert struct with seq/struct member
* move idl2srv.py under scripts directory, according to http://www.ros.org/wiki/Packages#Common_Files_and_Directories
* update idl2srv to output convert functions only needed in the srvs
* update idl2srv.py but not implemented yet
* fix enum type mapping in 32bit OS
* add idl2srv.py in rtmbuild, and add cmake macros for use it
* fix dependency
* fix to use rtm-config under openrtm directory
* use use compiled openrtm-c++ and openrtm-python
* add python idl compile script
* use ${${_idl}_depends} to depends .hh on other .hh, we want to look for automatic generation
* add OPENHRP_PRIVATE_LIBRARIES for compiling idl files in openhrp3 package
* add -I to idl_gen/cpp and idl_gen/cpp/{package}/idl, add rtmbuild_add_library
* add openinvent/ from http://www.openrtp.jp/INVENT/
* add current directory to idl_dirs, inlcude_dirs and library_dirs
* remove REQUIRED for openhrp3.1
* rtmbuild_add_executable automatically add all stub and skel libraries
* clean up
* add rtmbuild
* Contributors: Isao Isaac Saito, Kei Okada, gm130s@gmail.com, kei.okada, kei.okada@gmail.com, manabu1118, manabu1118@gmail.com, notheworld@gmail.com
