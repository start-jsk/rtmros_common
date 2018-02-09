^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openrtm_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2017-12-22)
------------------

1.4.1 (2017-10-25)
------------------

1.4.0 (2017-08-09)
------------------
* fix code to run both Indido and Kinetic (`#1025 <https://github.com/start-jsk/rtmros_common/issues/1025>`_)
  * add retry=3 to openrtm_tools/test/test-rtmlaunch.test

* Contributors: Kei Okada

1.3.3 (2017-02-12)
------------------
* [hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch, openrtm_tools/src/openrtm_tools/rtmlaunch.py] Add argument to change buffer length of rtmlaunch connection. Set 8 by default (default setting in openrtm). (`#980 <https://github.com/start-jsk/rtmros_common/issues/980>`_)
* re-enable rosnode_rtc (`#961 <https://github.com/start-jsk/rtmros_common/issues/961>`_)
  * openrtm_tools: find_package openrtm_aist for DEPENDS section in catkin_package
* Contributors: Kei Okada, Shunichi Nozawa

1.3.2 (2016-04-26)
------------------
* [fix ] Revert "workaround for ros-groovy-rqt-top installs wrong(?) psutil" (#938)  This reverts commit 94975a81eadbf251a85b2fd3c77137b1f96f248b. This closes `#936 <https://github.com/start-jsk/rtmros_common/issues/936>`_
* [refactor] add more mesage to prevent confusion
* [refactor] remove old rosbuild files
* [feat] Enable to set push_policy and push_rate for DataPorts from hrpsys_ros_bridge.launch argument. Use all and 50.0 by default to keep backward compatibility.
* [feat] Enable to set subscription_type for DataPorts from hrpsys_ros_bridge.launch argument. Use new by default to keep backward compatibility.
* Contributors: Shunichi Nozawa, Kei Okada, Iori Yanokura

1.3.1 (2015-12-16)
------------------
* deb release only targeting to indigo

1.3.0 (2015-12-09)
------------------
* deb release only targeting to indigo

* [src/openrtm_tools/rtmlaunch.py] check rtc connection once in rtmlaunch.py when "RTC_CONNECTION_CHECK_ONCE" env is set `#834 <https://github.com/start-jsk/rtmros_common/pull/835>`_
* [src/openrtm_tools/rtmlaunch.py] update tree to reflesh current status
* Contributors: Kei Okada, Masaki Murooka

1.2.14 (2015-06-23)
-------------------

1.2.13 (2015-06-11)
-------------------
* [src/openrtm_tools/rtmlaunch.py] keep one RTCTree() instance and reflesh when needed, to speedup
* Contributors: Kei Okada

1.2.12 (2015-04-27)
-------------------

1.2.11 (2015-04-25)
-------------------

1.2.10 (2015-04-24)
-------------------

1.2.9 (2015-04-11)
------------------
* [openrtm_tools/scripts/rtshell-setup.sh] modify rtmlaunch alias
* [openrtm_tools/rtmlaunch.py] disable execute bit and remove __main__, we have script/rtmluanch.py for this purpose
* [src/openrtm_tools/rtmlaunch.py] forget to import signal
* [openrtm_tools/scripts/rtmluanch.py] forget to import sys
* [openrtm_tools] create openrtm example program under openrtm_examles to use rosrun command in test code
* move rtmlaunch/rtmtest from hrpsys_ros_bridge to openrtm_tools, add envhook for catkin users
* Contributors: Kei Okada, Kunio Kojima

1.2.8 (2015-03-09)
------------------
* [test-*.py] test name could not contain minus? any changet to underscore (http://answers.ros.org/question/197377/rostest-not-finding-the-actual-test/)
* [FIXME][rtshell-setup.sh] source bash_completion, since shell_suport is not working for now
* search rtshell path from CMAKE_PREFIX_PATH
* [openrtm_tools] rtshell-setup.sh: fix shell_support location
* Contributors: Kei Okada

1.2.7 (2015-01-06)
------------------
* remove openrtm_aist_python and rtshell from find_package
* Hide print debug message of replacing of arg tag
* Add function to replace arg tag by env. SIMULATOR_NAME is replaced by default.
* Contributors: Kei Okada, Shunichi Nozawa

1.2.6 (2014-10-10)
------------------

1.2.5 (2014-10-04)
------------------

1.2.4 (2014-09-08)
------------------

1.2.3 (2014-09-03)
------------------

1.2.2 (2014-08-31)
------------------

1.2.1 (2014-06-23)
------------------
* (rtmlaunch.py) rtmlaunch should support unless attribute in group tag. the current implementation only takes into account if attribute of group tag. this commit adds supporting unless attribute.
* (package.xml files) Update repository URLs
* Contributors: Isaac Isao Saito, Ryohei Ueda

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------
* Merge pull request #404 from k-okada/403_check_rtprint
  add add check to rtprint
* add python path to openrtm_aist_python, this will fix #403
* add test program for rtshell/rtprint
* remove redundant rosdep name from manifest.xml
* does not call shopt on zsh
* Contributors: Kei Okada, Ryohei Ueda

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
* (rtmlaunch.py) trap SIGINT and exits with 0
* openrtm_tools: add rosbash to depends
* openrtm_tools : add test code to check if rtmlaunch works
* comment out import OpenRTM_aist.RTM_IDL # for catkin, see #375
* openrtm_tools: add rosbash to depends
* openrtm_tools : add test code to check if rtmlaunch works test-rtmlaunch.{py,test}
* comment out import OpenRTM_aist.RTM_IDL # for catkin, see #375
* display output when test fails
* fix PATH/PYTHONPATH for rosbuild/catkin
* add test code for openrtm_tools (rtshell-setup.sh)
* fix rtshell-setup.sh to support catkin environment
* add link to more useful information, suggested by Isaac
* remove debug code
* display more verbose information of weird connection error
* add rosdep hrpsys/openrtm_aist to fake rosdep

1.0.6
-----
* rtmros_common-1.0.6

1.0.5
-----
* "rtmros_common-1.0.5"
* support if/unless roslaunch Issue 253

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
* fix comment for new directory structure
* enable set alias on non-interactive shell
* unset \*_LIBRARIES not to add library names to Config.cmake
* add rtmtest.py from rtmlaunch in order to add rtmtest, add alias rtmtest to rtshell-setup.sh
* add alias to rtmlaunch
* fix corba port to 15005, see Issue 141
* fix catkin.cmake for installed project
* add import OpenRTM_aist.RTM_IDL, for catkin
* fix not using sub shell, [#179]
* fix catkin.cmake, since openrtm_aist/openhrp3/hrpsys is compiled as non-catkin package (this means catkin_make does not install <package>.pc <package>Config.cmake and use DEPENDS not CATKIN-DEPENDS)
* use localhost:5005
* switch completion settings of rtmlaunch by CATKIN_SHELL (for zsh users)
* set default corbaport to 5005, Fixes Issue 141
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* openrtm_tools depends on openrtm_aist, openrtm_aist_python, rtshell
* catkinze package, add rtshell to build_depend
* revert wrong commit : add rtshell as run_depend
* add rtshell as run_depend
* connect, then activate
* add path to rtshell, rtctree, rtsprofile, Issue 137
* fix install rtshell under /bin, /src, see Issue 146
* load manifest(openrtm_tools) Issue 137
* del wrong comit
* copy from openrtm/script, due to split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* copy from openrtm/script, due to split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* add rtshell-setup.sh
* split openrtm to openrtm_aist, openrtm_aist_python, opnertm_tools, rtshell, rtctree, rtsprofile, Issue 137
* Contributors: Kei Okada, gm130s@gmail.com, kei.okada, nakaokat@gmail.com, youhei@jsk.imi.i.u-tokyo.ac.jp
