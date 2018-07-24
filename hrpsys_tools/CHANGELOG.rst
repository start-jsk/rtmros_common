^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2017-12-22)
------------------

1.4.1 (2017-10-25)
------------------

1.4.0 (2017-08-09)
------------------
* fix code to run both Indido and Kinetic (`#1025 <https://github.com/start-jsk/rtmros_common/issues/1025>`_)
  * hrpsys_tools/test/test-pa10.test: increase sleep time to 10

* set USE_UNSTABLE_RTC as not all unstable rtc defiend in hrpsys_config.py, just for basic walking test, other latest development rtc is passed by arg (`#1004 <https://github.com/start-jsk/rtmros_common/issues/1004>`_)
  * [hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch,hrpsys_ros_bridge/scripts/default_robot_ros_bridge.launch.in,hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l,hrpsys_tools/launch/hrpsys.launch] Update for ObjectTurnaroundDetector RTC. add USE_REFERENCEFORCEUDPATER and USE_OBJECTCONTACTTURNAROUNDDETECTOR.

* Contributors: Kei Okada, Shunichi Nozawa

1.3.3 (2017-02-12)
------------------

1.3.2 (2016-04-26)
------------------

* [refactor] remove old rosbuild files
* [hrpsys_tools/launch/hrpsys.launch] add ReferenceForceUpdater
* Contributors: Koyama Ryo, Kei Okada

1.3.1 (2015-12-16)
------------------
* deb release only targeting to indigo

1.3.0 (2015-12-09)
------------------
* deb release only targeting to indigo

* bug fixes

  * [hrpsys_tools/samples/pa10.launch] Fix VRML path for pa10  hrpsys_tools test (hrpsys_tools/samples/pa10.launch)  `#829 <https://github.com/start-jsk/rtmros_common/issues/829>`_

* Chorenoid support (launch/hrpsys.launch )

  * fix hrpsys.launch because choreonoid do not need options for   periodic_rate and context_type `#810 <https://github.com/start-jsk/rtmros_common/issues/810>`_

   * add USE_CHOREONOID argument to hrpsys.launch for using choreonoid instead of hrpsys-simulator `#787 <https://github.com/start-jsk/rtmros_common/issues/787>`_

* scripts/rtmroswtf.sh updates `#764 <https://githbub.com/start-jsk/rtmros_common/issues/764>`_

  * [scripts/rtmroswtf.sh] disply LD_LIBRARY_PATH too
  * [scripts/rtmroswtf.sh] rtls is now global bin
  * [scripts/rtmroswtf.sh] check all package with git status
  * [rtmroswtf] Copyrights
  * [rtmroswtf] Add printing git status
  * [rtmroswtf] Add printing pkg path
  * [rtmroswtf] Add printing rosversion
  * [rtmroswtf] Remove verbose from tar that confuses users to
    sendback which tarball

* Contributors: Isaac IY Saito, Kei Okada, Shunichi Nozawa, YoheiKakiuchi

1.2.14 (2015-06-23)
-------------------
* Add EmergencyStopper setting to hrpsys.launch
* Contributors: Shunichi Nozawa

1.2.13 (2015-06-11)
-------------------

1.2.12 (2015-04-27)
-------------------

1.2.11 (2015-04-25)
-------------------

1.2.10 (2015-04-24)
-------------------

1.2.9 (2015-04-11)
------------------
* [hrpsys_tools/hrpsys.launch] support DEBUG_HRPSYS argument to run rtcd with gdb
* Contributors: Kei Okada

1.2.8 (2015-03-09)
------------------
* [test-*.py] test name could not contain minus? any changet to underscore (http://answers.ros.org/question/197377/rostest-not-finding-the-actual-test/)
* (diagnosis info) Renamed a script to show respect for the great ROS ancestors.
* (diagnosis info) Obtain package versions in a more generic way.
* (diagnosis info) Catch error stream.
* (diagnosis info) Variablize corba hostname and port.
* Add a very simple tool for recording system diagnosis info.
* Contributors: Isaac IY Saito, Kei Okada

1.2.7 (2015-01-06)
------------------
* (_gen_project) : Add argument to specify highgain mode
* (hrpsys.launch) : Support PDController setting for hrpsys.launch.
* Contributors: Shunichi Nozawa

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
* (hrpsys.launch) : Add config_file setting for HGcontroller
* (compile_robot_model.cmake, hrpsys.launch, hrpsys_tools_config.py) Add argument to use Unstable RTC List and configure it from cmake discussed in https://github.com/start-jsk/rtmros_gazebo/pull/61
* Contributors: Shunichi Nozawa

1.2.1 (2014-06-23)
------------------
* hrpsys_tools/launch/hrpsys.launch: Added ThermoLimiter/ThermoEstimator definition to hrpsys.launch
* (package.xml files) Update repository URLs
* Contributors: Isaac Isao Saito, iori

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------
* add RemoveForceSensorLinkOffset setting ;; I write both AbsoluteForceSensor and RemoveForceSensorLinkOffset for compatibility
* make fail when pa10 is fail to launch
* Contributors: Kei Okada, Shunichi Nozawa

1.0.11 (2014-04-16)
-------------------

1.0.10 (2014-04-15)
-------------------

1.0.9 (2014-04-14)
------------------

1.0.8 (2014-04-14)
------------------
* use start_omninames.sh for rosdevel build environment, see `#400 <https://github.com/start-jsk/rtmros_common/issues/400>`_
* add_py_launch_prefix
* Contributors: Kei Okada, Yohei Kakiuchi, Isaac Isao Saito

1.0.7 (2014-03-20)
------------------
* add respawn to rtcd/hrpsys-simulator, RESPAWN_MODELLOADER, RESPAWN_SIMULATOR, RESPAWN_RTCD, see Issue #380
* hrpsys_tools: (test-pa10.test) run unittest first, then test hcf
* hrpsys_tools, hrpsys_ros_bridge: increase retly to 4 for test
* hrpsys_tools: add rosbuild_add_rostest to CMakeList.txt and fix test-hrpsys-config.py to load_mafest for rosbuild environment
* add retry=2 tag, since we staill have trouble on connection sometimes (https://code.google.com/p/hrpsys-base/issues/detail?id=192)
* add launch-prefix argument for hrpsys_py node
* add samples direcotry to install
* add -c option to specify commands to execute
* set RobotHardware name due to api change of hrpsys_tools_config.py
* add samples direcotry to install
* use -c to specify commands
* add -c option to specify commands to execute
* forge to add
* set RobotHardware name due to api change of hrpsys_tools_config.py
* add test code that uses hrpsys.launch
* add waitForRTCManagerAndRobotHardware for interactive mode
* remove openrtm_aist_python from find_package
* add interactive mode, invoke with ipython ... or -i option
* add sample code of interactive mode of hrpsys_tools_config.py in test-hrpsys-config-test
* fix test code to work with clean environment
* fix python code syntax error
* fix typo
* write contents of test-hrpsys.test
* Update test-hrpsys-config.py
  rename testHrpsysConfigurator -> TestHrpsysConfigurator
* add test/test-hrpsys-config.test
* add depend hrpsys to fke rosdep install

1.0.6
-----
* rtmros_common-1.0.6
* add hrpsys_tools to depends on openrtm_tools, see Issue 309
* (hrpsys_tools, rosnode_rtc, rtmbuild) maintainer name adjusted to the one used in other places
* add config file for TorqueController
* add argument to set joint propeties according to hrpsys-base -r906 commit ;; we can set joint properties such as .angle, .mode, and so on ;; http://www.openrtp.jp/openhrp3/jp/create_project.html
* add argument to set timeStep according to hrpsys-base -r905 commit ;; we can set timeStep <= dt ;; http://www.openrtp.jp/openhrp3/jp/create_project.html

1.0.5
-----
* "rtmros_common-1.0.5"

1.0.4
-----
* "rtmros_common-1.0.4"
* rename hrpsys_config.py because there is the package with the same name in hrpsys

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
* forget to fix corba port to 15005, see Issue 141
* fix catkin.cmake for installed project
* add import OpenRTM_aist.RTM_IDL, for catkin
* fix catkin.cmake, since openrtm_aist/openhrp3/hrpsys is compiled as non-catkin package (this means catkin_make does not install <package>.pc <package>Config.cmake and use DEPENDS not CATKIN-DEPENDS)
* set default corbaport to 5005, Fixes Issue 141
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* install scripts
* catkinize
* remove export /scripts, see https://code.google.com/p/rtm-ros-robotics/wiki/rtm_ros_common_201307_migration
* create dummy hrpsys_config for ros users : due to hrpsys_config.py move to hrpsys-base, see https://code.google.com/p/hrpsys-base/source/browse/trunk/python/hrpsys_config.py
* revert : hrpsys_config.py move to hrpsys-base, see https://code.google.com/p/hrpsys-base/source/browse/trunk/python/hrpsys_config.py
* hrpsys_config.py move to hrpsys-base, see https://code.google.com/p/hrpsys-base/source/browse/trunk/python/hrpsys_config.py
* hrpsys_config.py move to hrpsys-base, see https://code.google.com/p/hrpsys-base/source/browse/trunk/python/hrpsys_config.py
* add depends to hrpsys
* fix package name openrtm->openrtm_tools
* circular dependency is not alloweded
* add hrpsys_tools: cleanup hrpsys, split hrpsys and hrpsys_tools, Issue 137
* Contributors: Kei Okada, gm130s@gmail.com, kei.okada, notheworld@gmail.com, youhei@jsk.imi.i.u-tokyo.ac.jp
