^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openrtm_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2014-06-23)
------------------
* (index.rst) Add usage api doc
* (package.xml files) Update repository URLs
* Contributors: Isaac IY Saito, Isaac Isao Saito

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------

1.0.11 (2014-04-16)
-------------------

1.0.10 (2014-04-15)
-------------------

1.0.9 (2014-04-14)
------------------

1.0.8 (2014-04-14)
------------------
* use start_omninames.sh for rosdevel build environment, see `#400 <https://github.com/start-jsk/rtmros_common/issues/400>`_
* Contributors: Kei Okada

1.0.7 (2014-03-20)
------------------
* openrtm_ros_bridge: (test_myservice_rosbridge.test) add retry=4
* openrtm_ros_bridge: mv samples/test_myservice_rosbridge.* -> test/test_myservice_rosbridge.* and add test to CMakeLists.txt/catkin.cmake
* disable openrtm_ros_bridge/rosbuil test for now
* Wrong catkin macro (CATKIN-DEPENDS to CATKIN_DEPENDS).
* use pkg-config to find directories
* disable test_myservice_rosbridge.launch, is is not working right now

1.0.6
-----
* rtmros_common-1.0.6
* (rtmros_common, openrtm_ros_bridge, hrpsys_ros_bridge) Cleaning and elaborating package.xml

1.0.5
-----
* "rtmros_common-1.0.5"

1.0.4
-----
* "rtmros_common-1.0.4"

1.0.3
-----
* "rtmros_common-1.0.3"

1.0.2
-----
* "rtmros_common-1.0.2"

1.0.1
-----
* rtmros_common) 1.0.1
* rtmros_common) Revert wrong commit r5588
* rtmros_common) 1.0.1
* unset openrtm_tools_LIBRARIES since openrtm_aist_LIBRARIES used in rtmbuild_init, use rtmbuild_PREFIX instaed of CMAKE_INSTALL_PREFIX to get rtmbuild.cmake
* add openrtm_aist to catkin_package/DEPENDS
* unset \*_LIBRARIES not to add library names to Config.cmake
* fix directory structure, see Issue 188
* fix catkin.cmake, since openrtm_aist/openhrp3/hrpsys is compiled as non-catkin package (this means catkin_make does not install <package>.pc <package>Config.cmake and use DEPENDS not CATKIN-DEPENDS)
* fix copy idl file from openrtm-1.1/example/\*.idl
* install rtmbuild from rtmbuild_SOURCE_DIR/CMAKE_INSTALL_DIR
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* remove REQUIRED from pkg_check_modules: catkin compile does not find rtmbuild/openrtm_aist package in the first time
* minior fix
* add rostest
* clean up catkin.cmake
* catkinize package
* openrtm_tools depends on openrtm_aist
* depends rosdep is requried
* del depends to roscpp
* use USE_ROSBUILD to swicth rosbuild/catkin
* move servicebridge.cmake idl2srv.py from openrtm_ros_bridge to rtmbuild
* add depends to rtmbuild
* fix wrong commit r4747, PROJECT_SOURCE_DIR -> openrtm_ros_bridge_PACKAGE_PATH, idl2srv.py, servicebridge.cmake
* fix r4737,  idl2srv servicebridge.cmake
* check if idl or idl2srv servicebridge.cmake are newer than generated files
* rm old files if idl2srv.py or servicebridge.cmake has changed
* change INCLUDE_DIRS from {prefix}/idl_gen/cpp/{prefix}/idl to {prefix}/idl_gen/cpp, so need to include <prefix/idl/hoge.h> instead of <hoge.h>, Issue 137
* rtm-config is in openrtm_aist, due to split openrtm into openrtm_aist, openrtm_aist_python, rtshell, rtctree and rtsprofile, Issue 137
* fix MyService{Provider,Consumer}Comp is under openrtm_aist, not openrtm_tools, due to split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* fix openrtm -> openrtm_aist, due to split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* fix wrong commit, r4692
* fix wrong commit, r4692
* add debug message
* set HAVE_DISPLAY to false
* add test code in python, add HAVE_DISPLAY arg for test environment
* add clean
* use copy_if_different
* Memory leak on OpenRTM service all, thanks to mitu.kojima. Fixes issue 94
* fix for openrtm-1.1
* check timestamp to compile #41
* Debugged convert functions for no allocbuf classes in idl2srv.py
* Add allocbuf for CORBA dblsequence
* back to 2011 + special case code for RTC::LightweightRTObject
* undo change commited in r3108
* ignore convert from RTM interface to ROS msg
* add try/catch to call openrtm service
* fixed index.rst and conf.py conflict problem
* use ros node name as openrtm component name
* update documents
* update documents
* add conf.py index.rst
* remove cmake_minimum_required, which is defined in rtmbuild.cmake
* add test_myservice_rosbridge.launch
* fix test
* add image for documents
* update example and document
* add comments in sample script
* fix generate msg/srv from idl before rosbuild_init
* add sample shell scripts, cmake functions moved
* add test for SimpleService.echo
* push down services into bridge node namespace, idl2srv.py
* mv latest idl2srv.py
* add openrtm_ros_bridge package, autogen service bridge component
* Contributors: Isao Isaac Saito, Kei Okada, chen.jsk@gmail.com, gm130s@gmail.com, ik0313@gmail.com, kei.okada, kei.okada@gmail.com, manabu1118@gmail.com
