^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2014-10-04)
------------------
* Improvement on rtm-ros-robot-interface, datalogger-log-parser, samplerobot
* Fix stamp of odom and imu
* Improve dependency: robot_pose_ekf.launch, robot_pose_ekf
* Contributors: Kei Okada, Shunichi Nozawa, YoheiKakiuchi

1.2.4 (2014-09-08)
------------------
* (test/test-pa10.test) support GUI argument
* (test/test-pa10.py)   add test for /command
* (hrpsys_ros_bridge/src/HrpsysJointTrajectoryBridge.cpp) support <controller>/command, see #537
* (README.md) fix document, based on snozawa's comment
* Contributors: Kei Okada

1.2.3 (2014-09-03)
------------------

1.2.2 (2014-08-31)
------------------
* (rtm-ros-robot-interface)

  * Define Euslisp setter and getter from param slots names
  * Update set-st-param for Stabilizer
  * Add KalmanFilter ROS Bridge and euslisp interface to hrpsys_ros_bridge.launch
  * Add calibrate-inertia-sensor
  * Add new arguments for new st param
  * Fix end-effector name (without colon) according to https://github.com/fkanehiro/hrpsys-base/pull/301
  * Update abc and st euslisp interface according to idl update, `fkanehiro/hrpsys-base#239 <https://github.com/fkanehiro/hrpsys-base/issues/239>`_
  * Access robot-state's imu in callback to fix https://github.com/start-jsk/rtmros_tutorials/issues/67
* Use catkin_make -C to change direcotry (Fix `#523 <https://github.com/start-jsk/rtmros_common/issues/523>`_)
* (datalogger-log-parser.l)

  * Support https://github.com/jsk-ros-pkg/jsk_pr2eus/commit/3200b63dfcbd3c02b919fe6ad03c425e5057ee5c commit
  * Support both reference worldcoords and actual worldcoords ;; StateHolder's value is reference and kf is actual.
* added make-default-ForceCalibPosesForLegs to euslisp/calib-force-sensor-params.l
* (Force sensor)

  * fixed accessing to force sensor in calibration function
  * fix AbsoluteForceSensorService -> RemoveForceSensorLinkOffsetService
* (compile_robot_model.cmake, hrpsys.launch, hrpsys_tools_config.py) Add argument to use Unstable RTC List and configure it from cmake discussed in https://github.com/start-jsk/rtmros_gazebo/pull/61
* Contributors: Kei Okada, Kunio Kojima, Shunichi Nozawa, Masaki Murooka, Isaac IY Saito

1.2.1 (2014-06-23)
------------------
* add euscollada path for compile collada model
* Merge pull request #495 from k-okada/add_servo_controller
  hrpsys_ros_bridge.launch: add servo controller
* (datalogger-log-parser.l) : Check existence of zmp value and rpy value
* hrpsys_ros_bridge.launch: add servo controller fix rtmlaunch options (thanks to @hyaguchijsk)
* does not use group tag, just use unless tag to disable connection
  between sh.rtc:baseTformOut and baseTform
* (hrpsys_ros_bridge/package.xml) run_depend on ipython
* (hrpsys_ros_bridge/package.xml) Sort in alphabetical order
* (rtm-ros-robot-interface.l, datalogger-log-parser.l) Update imu euslisp methods ;; We should use /imu instead of tf according to https://github.com/start-jsk/rtmros_common/pull/477
* hrpsys_ros_bridge/euslisp/datalogger-log-parser.l:add reading zmp data from hrpsys log file
* Merge pull request #458 from snozawa/rotate_camera_z_axis
  Rotate sensor->localR 180[deg] because OpenHRP3 camera -Z axis equals to ROS camera Z axis
* Merge pull request #477 from garaemon/add-imu-topic-remove-imu-floor
  Add /imu topic and /odom topic remove imu_floor
* Merge pull request #460 from k-okada/hrpsys_version_gte
  set hrpsys version greater than 315.2.0
* update pa10 test code
* publish Imu value to /imu topic and odometory value to /odom.
* fix ProjectGenerator path for compile_collada_model
* update .gitignore for auto generated files under hrpsys_ros_bridge directory
* rtmlaunch; Add usage api doc
* hrpsys_ros_bridge/euslisp/datalogger-log-parser.l : update state methods and add eof check
* hrpsys_ros_bridge/README.md, hrpsys_ros_bridge/euslisp/README.md : add README for hrpsys_ros_bridge and euslisp usage
* hrpsys_ros_bridge/euslisp/datalogger-log-parser.l : support force sensor reading
* hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l : update sensor accessor commited in https://github.com/euslisp/jskeus/pull/92
* (package.xml files) Update repository URLs
* (start-jsk/rtmros_common/pull/397) hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch, hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py : rename AbsoluteForceSensor => RemoveForceSensorLinkOffset
* (collision_detector.launch) add default model file path using one fromreal robot
* ( https://github.com/start-jsk/rtmros_gazebo/issues/44 ) hrpsys_dashboard.py: add 'import re' for using regular expression
* add generation of hrpsys_config for robots using unstable RTCs ;; add configuration for unstable RTCs in cmake ;; by default, do not nothing, so there will be no side effect for robots only using Stable RTCs
* set RTCTREE_NAMESERVERS environment variables, for #453
* pass args to super class
* set hrpsys version greater than 315.2.0
* Rotate sensor->localR 180[deg] because OpenHRP3 camera -Z axis equals to ROS camera Z axis http://www.openrtp.jp/openhrp3/jp/create_model.html
* add samplerobot.rviz
* hrpsys_ros_bridge/collision_state.py) Use logerr to highlight an error that could be overlooked
* (HrpsysJointTrajectoryBridge) ros cpp style macro applied.
* Contributors: Isaac Desu Saito, Isaac IY Saito, Isaac Isao Saito, Kei Okada, Kunio Kojima, Ryohei Ueda, Tatsushi Karasawa, YoheiKakiuchi, mmurooka, nozawa

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------
* (collision_detector.launch) typo
* use odom instad of imu_floor
* do not connect port when afs is not found
* set WALKING and IMPEDANCE controller as non-default RTC
* update generate config.yaml
* add both Robot(Robot)0 and RobotHadware0 in datalogger
* add scripts for auto generating controller config yaml
* Use subprocess.Popen rather than check_call in order to call
  external process asynchronouslly
* pass argument to get-ROSBridge-method-defmacro method
* add generating urdf file to compile_robot_model.cmake
* Merge pull request #433 from k-okada/do_not_compile_idl_twice
  do not update manifets.xml and copy idl when it is not needed (#429)
* implement hrpsys_dashboard base on rqt, not on rxtools
* do not update manifets.xml and copy idl when it is not needed (#429)
* rename base-pos and base-rpy => root-pos and root-rpy
* add reading of datalogger properties
* set REALTIME=ture as default
* update :start-auto-balancer and :stop-auto-balancer method according to hrpsys-base trunk update at https://code.google.com/p/hrpsys-base/source/detail?r=1039  commit ;; we do not need to change usage of these methods
* Contributors: Isaac Saito, Kei Okada, Ryohei Ueda, YoheiKakiuchi, Shunichi Nozawa

1.0.11 (2014-04-16)
-------------------
* use random staritng  port number for rtm_naming, also try to continue if name server is already running
* Contributors: Kei Okada

1.0.10 (2014-04-15)
-------------------
* remove sed to comment out pr2_controllers
* Contributors: Kei Okada

1.0.9 (2014-04-14)
------------------
* (hrpsys_ros_bridge) add git to build_depend
* Contributors: Isaac IY Saito

1.0.8 (2014-04-14)
------------------
* use git:// for download pr2_controllers (potential fix for `#410 <https://github.com/start-jsk/rtmros_common/issues/410>`_)
* add rosdnode to depends(see https://github.com/jsk-ros-pkg/jsk_roseus/pull/65, `#411 <https://github.com/start-jsk/rtmros_common/issues/411>`_)
* add euscollada_SOURCE_PREFIX and euscollada_PREFIX
* use start_omninames.sh for rosdevel build environment, see `#400 <https://github.com/start-jsk/rtmros_common/issues/400>`_
* remove deprecate function to generate conf parameter
* Added procps, hostname, net-tools build_depends. These tools are used during building and testing
* Contributors: Kei Okada, Scott K Logan, Shunichi Nozawa

1.0.7 (2014-03-20)
------------------
* call find_package(catkin) fist
* `#22 <https://github.com/start-jsk/rtmros_common/issues/22>`_: add PROJECT_NAME to the target used in compile_robot_model to avoid collision of the target names
* add rostest/hrpsys_tools to depends and find_package
* hrpsys_ros_bridge: (test-compile-robot.launch) add test-compile-robot.launch (but is is not includeded in CMakeLists.txt and use .launch instead of .test due to Issue `#87 <https://github.com/start-jsk/rtmros_common/issues/87>`_), fix compile_robot_model.cmake work with devel of hrpsys_ros_bridge, disable launch script from test-compile-robot.sh
* hrpsys_ros_bridge: add test-compile-robot.sh test-compile-robot.xml  test-compile-robot.cmake
* check hrpsys_ros_bridge SOURCE_DIR for both SOURCE_DIR and PREFIX
* hrpsys_ros_bridge: use catkin package of pr2_controllers_msgs
* hrpsys_tools, hrpsys_ros_bridge: increase retly to 4 for test
* hrpsys_ros_bridge: install/lib/python2.7/dist-packages/hrpsys_ros_bridge/__init__.py disappeared somehow? this is  temprarily fix (FIXME)
* hrpsys_ros_bridge: fix compile_robot_model.cmkae, use find_package to set /lib/openrtm_aist/bin/rtm-naming /lib/openhrp3/export-collada
* collision_state.py: display with logwarn when CollisionDetector is not found
* hrpsys_ros_bridge, fix path for installed conf/dae/launch files
* compile_robot_model.cmake: add debug message in generate_default_launch_eusinterface_files
* hrpsys_ros_bridge: (test-samplerobot.py, test-p10.py), increase timeout of watForTransform() and catch exception if tf is not found
* collision_state, wait until co is found, if not found exit with 0, not 1
* hrpsys_profile.py add Exception
* hrpsys_ros_bridge: add collada_urdf to depends
* hrpsys_profile.py: run hrpsys_profile within try block
* hrpsys_profile.py: remove undefined variables
* hrpsys_ros_bridge: add visualization_msgs to depeds
* hrpsys_ros_bridge: fix ProjectGenerator location, see #353
* hrpsys_ros_bridge: add test code for samplerobot and pa10
* hrpsys_ros_bridge: add diagnostic_aggregator to depend (manifest.xml package.xml)
* hrpsys_ros_bridge: add more args to default_robot_*.in (GUI, SIMULATOR_NAME, corbaport)
* add comment on why we remove depend to pr2_controllers_msgs
* (package.xml) add angles to build_depend intentinally dut to build_depend to tf does ont install angles
* (manifest.xml) commented out depend package pr2_controllers_msgs for rosmake dependency graph generation, but comment in for rosmake build objects
* (manifest.xml) if you have both rosdep and depend, rosmake does not work well, see https://github.com/jsk-ros-pkg/jsk_common/issues/301
* (manifest.xml) users are expected to manually install ros-groovy-pr2-dashboard
* (CMakeLists.txt) download pr2_controllers_msgs for groovy/rosbuild
* fix typo ;; elif -> elseif
* add
* Wrong catkin macro (CATKIN-DEPENDS to CATKIN_DEPENDS).
* change destination of stdout of rtmlaunch.py by OUTPUT arg
* add CMAKE_PREFIX_PATH so that rosrun hrpsys ProjectGenerator works
* Merge pull request #334 from k-okada/master
* add openhrp3_PREFIX, more debug message
* add / after hrpsys_idl_DIR
* add hrpsys to find_package
* add more verbose log when error
* update PKG_CONFIG_PATH for hrpsys-base
* display error output
* add depend to pkg-config
* add depend to pkg-config
* added euslisp, srv, idl directories to install
* move to git repository
* add hrpsys to find_package
* add more verbose log when error
* Merge branch 'master' of http://github.com/k-okada/rtmros_common
* display error output
* add depend to pkg-config
* add depend to pkg-config
* update PKG_CONFIG_PATH for hrpsys-base
* Merge branch 'master' into garaemon-master
* not generating sh but running rostes directory to avoid escape problem
* Merge branch 'master' of http://github.com/k-okada/rtmros_common
* use pkg-config to find directories
* Merge branch 'master' of https://github.com/start-jsk/rtmros_common
* move to git repository
* added euslisp, srv, idl directories to install
* fixing list syntax
* force to set ROS_PACKAGE_PATH when calling euscollada for catkin build
* adding LD_LIBRARY_PATH
* remove depend to robot_monitor
* changing the working directory when call export-collada
* do not compile lisp code if euxport collada is not exists
* add diagnositcs_msgs to fake rosdep
* add dynamic_recofigure to fake rosdep
* add several rosdep names to fake rosdep
* add rosdep hrpsys/openrtm_aist to fake rosdep
* profibit to run rostest parallel
* installing src directory as python package
* (hrpsys_ros_bridge/package.xml) Partially revert r6936 where a dependency was removed by mistake.
* Contributors: Kei Okada, Ryohei Ueda

1.0.6
-----
* rtmros_common-1.0.6
* (rtmros_common, openrtm_ros_bridge, hrpsys_ros_bridge) Cleaning and elaborating package.xml
* add setting for CONF_DT_OPTION
* set corba port to 15005 in euslisp model dump function
* fix proble due to r6908, also remove find_package since hrpsys can not find-packed-ed
* add hrpsys_tools to depends on openrtm_tools, see Issue 309
* set custom cmake file under CFG_EXTRAS, so that other package is abel to use macros defined in the cmake file
* generate /tmp/_gen_project_**.sh file to invoke rostest since rostest needs package path, see Issue 306
* installing euslisp directory of hrpsys_ros_bridge when catkining
* not use rosbuild for catkin
* fixing close tag in nosim.launch
* add RUN_RVIZ arg in default launch template files
* add REALTIME arg in hrpsys-simulator launch template files
* fix position of long floor
* add :torque-vector for playpattern
* add reference torque vector data reading
* update initialization to neglect log files which lines are not match ;; convert rad2deg
* fix typos ;; wf -> zf
* add .zmp file dump
* add fname for datalogger-log-parser
* use .rviz file for .in file
* add datalogger log parser for euslisp
* enable to se PROJECT_FILE name
* remove unnecessary print function
* fix hrpsy_ros_bridge.launch for hrpsys version 315.1.5
* add :zmp-vector and callback ;; acc print function for end_effectors in conf file
* add argument for tm
* add type_name to SensorInfo ;; use it in imu_floor tf sending ;; for almost all robots, frame name is 'gyrometer'
* write seq pattern file separatively ;; do not dump .waist file if :root-coords does not exist
* publish zmp as PointStamped
* add zmp data port for HrpsysSeqStateRosBridge
* add euslisp utility fnuction for seq pattern file
* fix syntax error
* set_stale when op_mode is not Servo ON nor Servo Off
* move hrpsys_profiel out of USE_DIAGNOSTICS
* rename USE_DIAGNOSTICS <- LAUNCH_DIAGNOSTICS args
* add LAUNCH_DIAGNOSTICS args
* add use_diagnostics, use_rosout, use_battery option in __init__
* using prefix for sensor-name
* add debug print, when body->joints().size() and m_servoState.data.length() is not equal
* split hrpsys_dashboard into hrpsys_dashboard and src/hrpsys_ros_bridge/hrpsys_dashboard.py, see Issue 270
* fix color
* change sphere radius depends on dangerness
* rosbuild does not raise exception when import roslib, use import hrpsys
* add utility functions for project generator xml file
* add argument to set timeStep according to hrpsys-base -r905 commit ;; we can set timeStep <= dt ;; http://www.openrtp.jp/openhrp3/jp/create_project.html
* enable to add :root-coords which is optional robot states
* fix typo ;; tile -> time
* change line width depends on the distance

1.0.5
-----
* "rtmros_common-1.0.5"
* reverse rpy-angle because euslisp rpy is 'yaw pitch roll' and openhrp3 rpy is roll pitch yaw
* pkill kill cmake itself, not why, so create shell script and execute it
* use find_package for hrpsys and hrpsys_tools
* fix Issue 256
* do not depends to lisp file if euscollada is not exists
* fix missing variable _collada2eus to _collada2eus_exe
* do not FATAL_ERROR when collada2eus is not found
* fix for catkin compile
* add debug message
* use full path instead of rosrun/rostet for when two binaries are installed
* fix for catkin, catkin does not requires load_manifest
* apply patch in Issue 254
* fix ROS_DISTRO->groovy, this code only used in groovy
* fix sed command
* fix typo ENV=>
* add rostest to run_depend
* set pr2_controllers to build/run depends and fix catkin.cmake to comment out therese tags if groovy
* distance under 0.05 always red
* translate line information relative to root_link_frame
* add lifetime
* add standalone collision detector launch file, see #249
* fix color
* add error message more verbose
* catch CORBA.OBJECT_NOT_EXISTS
* rename typo collistion_state.py  collision_state.py
* add collistion_state.py in launch file, Issue 249
* add collistion_state.py, Issue 249
* fix length of force list for resetting
* fix compile_openhrp_model for catkin, see Issue 241

1.0.4
-----
* "rtmros_common-1.0.4"
* update with r5733

1.0.3
-----
* "rtmros_common-1.0.3"
* remove :start and :stop abc in go-pos and go-velocity
* update to use from hrpsys.hrpsys_config import \*, due to change at https://code.google.com/p/hrpsys-base/issues/detail?id=128, Fixed Issue 231, Fixed Issue 219
* add end-effector parameter print function for abc

1.0.2
-----
* "rtmros_common-1.0.2"
* use connection as new
* add calc-zmp-from-state method
* add loading of utils file if exists
* fix typoes ;; _name -> _sname
* get current additional_make_clean_files and append setting to it ;; use _sname instead of overwriting _name
* add generated launch and euslisp files to clean target
* fix when rtmros_common is installed as catkin package, see Issue 214
* install cmake directory

1.0.1
-----
* rtmros_common) 1.0.1
* rtmros_common) Revert wrong commit r5588
* rtmros_common) 1.0.1
* hrpsys_ros_bridge) fix to issue 205 (incomplete but worth trying)
* hrpsys_ros_bridge) Correction to r5584.
* hrpsys_ros_bridge) Improve package.xml description.
* add pr2_dashboard path to sys.path for those who could not import pr2_dashboard
* add comment when pr2_dashboard is not found
* hrpsys_dashboard) Fix to run with rosbuild.
* hrpsys_ros_bridge) add Isaac as a maintainer to get notice from buildfarm.
* add diagnostic_msgs to depends
* add roslang to depend
* use rtmbuild_PREFIX instaed of CMAKE_INSTALL_PREFIX to get rtmbuild.cmake
* fake rospack not to sudo rosdep install
* add python-rosdep and rosdep udpate to different execute_process
* do not set local for jenkins debuild
* copy rtmlaunch/rtmtest/rtmstart.py to both share and bin for backword compatibility
* add random string to pr2_controllers_msgs directory path
* workaround for ros-groovy-rqt-top installs wrong(?) psutil
* fix for new rtmstart.py
* add LC_ALL=en_US.UTF-8
* fix syntax error on previous commit
* add rosdep update when compile pr2_controlers_msgs
* unset \*_LIBRARIES not to add library names to Config.cmake
* add python-psutil to depend
* add rosbuild to depends
* add mk
* add --non-interactive --trust-server-cert for svn
* use corba port number 15005
* install rtmtest and rtmstart.py
* remove svn:executable from rtmstart.py
* add rtmtest.py from rtmlaunch in order to add rtmtest, add alias rtmtest to rtshell-setup.sh
* add subversion for groovy, that download and compile non-catkinized pr2_controllers_msgs package
* download and compile pr2_controllers msgs which is not catlinized on groovy
* pr2_controllers is not catkinized on groovy
* add pr2_controllers to bulid/run_depend and use ROS_DISTRO to include pr2_controller_msgs directory
* add code for restarting jointGroup
* add remove-joint-group and remove obsolate code
* fix directory structure, see Issue 188
* add generating code of controller_config.yaml for using joint group
* checking the existance all joints in required group on HrpsysJointTrajectoryBridge (ignore not existing joint)
* fix : rearrange joint order in required goal
* forget to fix corba port to 15005, see Issue 141
* fix catkin.cmake for installed project
* add import OpenRTM_aist.RTM_IDL, for catkin
* print error message on detection of nan in imu_floor tf
* install FILES does not need USE_SOURCE_PERMISSIONS
* fix catkin.cmake, since openrtm_aist/openhrp3/hrpsys is compiled as non-catkin package (this means catkin_make does not install <package>.pc <package>Config.cmake and use DEPENDS not CATKIN-DEPENDS)
* fix to work on both rtmbuild and catkin environment
* ignore srv files whose name starts with . in rtm-ros-robot-interface, closes Issue 149
* fix for catkin
* check if port_number(5005) is already used, Fixed Issue 141
* comment out pr2_controllers
* change pr2_controllers_msgs -> pr2_controllers
* set default corbaport to 5005, Fixes Issue 141
* fix method name for set-joint-angles-of-group
* add add_dependencies to compile_hrpsys
* fix copy idl file from hrpsys/idl
* install rtmbuild from rtmbuild_SOURCE_DIR/CMAKE_INSTALL_DIR
* fix typo ;; COLLISIONCHEC => COLLISIONCHECK
* print name of ports
* add arguments to specify generated files
* add arguments to specify file generation
* rename macro ;; fix typo
* add nosim template and fix euslisp file name
* add macro to generate default launch files and euslisp interface
* wirte more debug comment to ROS_INFO_STREAM
* hrpsys_ros_bridge) Delete unused dependency. Alphabetize dependency.
* Remove unused components (robot_monitor).
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* check if USE_ROSBUILD is defined
* fix 2888 -> 2890 according to discussion of http://code.google.com/p/rtm-ros-robotics/issues/detail?id=139
* fix for catkin, with USE_ROSBUILD
* catkinize package
* add depends to openrtm_ros_bridge
* add openrtm_tools as depend
* add depends to hrpsys_tools
* workaround for Issue 150
* use hrpsys_ros_bridge for generating ROS service methods
* rtmlaunch.py was moved to openrtm_tools
* import from hrpsys_ros_bridge.srv instead of hrpsys.srv
* minor change
* hrpsys.launch now goes under hrpsys_tools and rename hrpsys.py to hrpsys_config
* add depends from hrpsys to hrpsys_tools, (hrpsys_tools to hrpsys is also exists, this this is intentially looped) so that users just depends on hrpsys
* change INCLUDE_DIRS from {prefix}/idl_gen/cpp/{prefix}/idl to {prefix}/idl_gen/cpp, so need to include <prefix/idl/hoge.h> instead of <hoge.h>, Issue 137
* ProjectGenerator is under hrpsys_tools not hrpsys: cleanup hrpsys, split hrpsys and hrpsys_tools, Issue 137
* copy idl from hrpsys/share/hrpsys/idl to hrpsys_ros_bridge/idl, Issue 137
* add depends to hrpsys_tools, Issue 137
* update openrtm -> openrtm_aist, hrpsys -> hrpsys_toolss: cleanup hrpsys, split hrpsys and hrpsys_tools, Issue 137
* fix port num ;; 2888 -> 2889
* run omniNames with port 2888 in compile_openhrp_model
* add USE_ROBOTHARDWARE
* check if server is active before send set preempt, this enables exit component cleanly, related to Issue 47
* fix miss commit ;; add temperature
* enable to store extra_data of motor_states
* use PeriodicExecutionContext instead of hrpExecutionContext, Issue 136
* launch node within group tags not to invoke unused component, see Issue 114
* add dt conf file option
* remove unnecessary copying of return value
* add _ret_add_str and remove duplicate codes
* fix position of return value setting
* do not specify if base-name is nil
* add connection for hrpsys_ros_bridge
* add dependency of _xmlfile on _xmlfile_nosim to achieve exclusive execution of rostest
* print port name in sensor_ros_bridge_connect
* adding some wait for waiting to update rosnode information
* fix return value if limb is nil
* fix separator of macro argumens
* fixes Issue 64, some packaed does not depends on euscollada, invoke rtm-naming within add_custom_command
* add dependency on euscollada, export-collada, and _gen_project.launch
* rename -xxx options -> --xxx options
* specify 2888 in pkill
* add macros to obtain options for _gen_project.launch and euscollada
* add get_option macro
* add INTEGRATE argument
* start model loader with 2888 port when launch _gen_project.launch
* do not make error, when collada2eus is not found, some package(hironx_ros_bridge) does not depends on euslisp
* add xml nosim to target
* add nosim option and generate nosim xml
* add robothardware conf option
* rename abs_xx -> off_xx
* add compile_collada_model to ALL
* fix (arg SIMULATOR_NAME)0 -> (arg SIMULATOR_NAME)
* rtmlauch.use_if.py -> rtmlaunch.py
* cleanup hrpsys_ros_bridge.launch, use if group, by miyake. Issue 114
* enable to load only link offset ;; use average value
* fix typo ;; params -> param
* add :reset-force-moment-offset
* support euslisp style argument for forcemoment and impedance methods
* use euscollada-model's :force-sensor methods instead of get-force-sensor-from-limb-name
* revert to use force-vector and moment-vector
* update to use absolute force vector for calibration
* add :imucoords method
* add port and connection for absolute force sensor values
* use AbsoluteForceSensor for impedanc Controller
* call :update-robot-state in super's :state methods
* check return value of imucoords from TF
* call rtc_init before rospy.init so that Issue 117
* neglect arguments added by roslaunch
* add rtmlaunch that runs omninaming and runs roslaunch, Issue 105, this is not tested on NameServer running on remote computer, see to https://code.google.com/p/hrpsys-base/issues/detail?id=117
* imu reading of update-robot-state
* fix wrong length of angles while uing JointGroup
* add ForwardKinematicsServiceROSBridge
* cleanup launch file Issue 114
* fix: condition for checking num of publisher
* fix: using walltime for waiting timeout
* remove miss commited comment out
* tf::Matrix3x3 causes compilation error in electric. use btMatrix3x3 instead
* do not publish tf which contains nan values
* chnage: detecting num of /clock publisher before starting and stoping subscribe /clock
* update footstep interface for abc
* add print message in starting
* add vel ratio
* add cmd vel mode
* check hrp::link parent existence because joint state with no name is published at previous version ;; This is OpenHRP3 'WAIST' joint specification. I should check OpenHRP3 model loading
* select a timestamp of force sensor messages according to use_sim_time
* get root-link name in :state :imucoords.
* add default-double-support-ratio setting
* add robothardware conf setting
* add macros for export_collada_option and conf_file_option
* add stabilizer methods
* comment out target_name and base_name and support default zmp offset
* add optional argument to collada2eus
* add stabilizer ros bridge
* enable to specify calib offset
* rename make pose function ;; support nodisplay usage
* add force sensor calib program
* add load-forcemoment-offset-param method
* add interface to specify sensor offset
* enable to access force-sensor by limb name
* make models directory in compile model macros when it doesn't exist
* set RobotHardware0 by default for SIMULATOR_NAME
* use SIULATOR_NAME instead of ROBOT_NAME in hrpsys_ros_bridge.launch ;; ROBOT_NAME argument in hrpsys_ros_bridge.launch is still remained for compatibility but please do not use ROBOT_NAME
* support limb name usage
* fix typos and add virtual force sensor stop method
* add connection for virtual force sensor
* fix keyword of set-servo-gain-percentage
* fix naming rule of virtual force sensor and fix quaternion -> axisangle
* add CONF_FILE_ARG and specify vitual_force_sensor by using conf file
* specify rtc manager name
* specify nameserver name and CORBA_init args
* rename value -> default
* use WAIST if base-name is nil ;; Is this correct?
* remove depends to pr2_dashboard for groovy, wait for fix
* change a name of imu tf and add a method to get imu coords in rtm-ros-robot-interface
* fix :set-base-pose in rtm-ros-robot-interface.l
* fix for publishing warning message
* add connection from state_holder(qOut) to rosbridge(mcangle)
* comment out port for RobotHardwareService because Simulator does not have RobotHardwareService
* add comment for groovy
* remove rtconnects between RobotHardware and KalmanFilter; this is achieved by hrpsys.py instead
* comment the IMU related rtconnects out for non IMU robots
* add basePos and baseRpy to HrpsysSeqStateROSBridge and connect in hrpsys_ros_bridge.launch
* modify rtm-ros-robot-interface.l: add set-servo-error-limit and set-soft-error-limit
* modify hrpsys_ros_bridge.launch to execute RobotHardwareServiceROSBridge and SoftErrorLimiterServiceROSBridge
* add default rate to hrpsys_ros_bridge
* check sensor existence for HRP4C
* use regiterInport for gyrometer and gsensor
* fix walking command and example
* use automatically generated methods
* add define-all-ROSBridge-srv-methods to define srv methods from srv and idl files ;; currently comment out
* add comments to hrpsys.py
* add getSensors and use it
* fix for hrpsys-base's r693 commit ;; fix for acc and rate naming rules
* make class to configure hrpsys RTCs and data ports
* add StateHolder rosbridge
* set subscription type new for real robot
* set subscription type new for real robot
* change name of a method for logger (Issue 74)
* add method for logger, Flxed Issue 74
* added rtconnect of torque in hrpsys_ros_bridge.launch
* remove autobalancer methods from sample
* add target2foot_offset_pos and rot
* add methods for get and set parameters for ABC
* enable multiple --conf-file-option
* add AutoBalancer for hrpsys.py and launch files
* add methods for auto balancer RTC
* add method to dump pattern file for Seq
* update check of virtual force sensor impedance control and add seq loadPattern
* enable to specify conf file setting for _gen_project.launch
* support virtual force sensor usage
* add sensor_ros_bridge_connect to connect sensor data ports based on ModelLoader's sensor information
* re-organize code, use IDL style method name for each service/idl and use euslisp style method name for wrapper of them
* enable to get force-vector and moment-vector
* add removeforcesensor offset interface
* add getImpedanceControllerParam and enable to set parameter individualy
* fix potision of argument in :start-impedance-impl
* fix typo ;; start-impedance in stop-impedance
* fix condision of limb for start and stop impedance
* add yaml depend in hrpsys_ros_bridge
* use :force-sensor method
* use root-link's parent by default
* fix bug of send*
* add methods for impedance controller
* fix name : use upper case for argument, (Fixes issue 65)
* add launch_ros_model argument to hrpsys_ros_bridge.launch
* fix number of clock publisher ;; publisher ==1 means no clock publisher other than HrpsysSeqRosBridge, which means HrpsSeqRosBridge should publish /clock
* add ImpedanceCOntroller service ROS Bridge
* check use_sim_time
* do not publish /clock when othre node pulishes
* rm config, no longer used?
* fix : move hrpsys_ros_bridge exmaple codes to hrpsys_ros_bridge_tutorials
* move hrpsys_ros_bridge exmaple codes to hrpsys_ros_bridge_tutorials
* add CollisionDetectionService interface to rtm-ros-robotics-interface.l
* add CollisionDetectorService
* add import time for sleep
  Use one depth queue for avoid sending multiple servo off message, Issue 32
  Status: Fixed
* add setServoGainPercentage which is euslisp interface for idl2srv of RobotHardware RTC
* add controller configuration file to hironx_ros_bridge.launch
* add HIRONX_controller_config.yaml
* Added Log button to hrpsys_dashboard to call DataLoggerService/save
* add comment to why we need r3202
* add hrp4c sample for using joint group
* fix for using joint group
* add HrpsysJointTrajectoryBridge for using joint group
* fix directory name
* fix directory name
* mv euslisp programs to euslisp directory
* revert CMakeLists.txt
* add test code for calling seq group
* add comments
* add debug message to set-interpolation
* add :setBasePose method to rtm-ros-robot-interface.l
* add sequenceplayserservicerosbridge node
* fix by iori
* remove glc_cpature from rviz node
* fix for hrpsys 3.1.4
* support control_msgs/FollowJointTrajectoryAction
* update SIMULATOR NAME, set use_sim_time true
* remove debug code
* publich /clock from rsangle when /use_sim_time is true
* add euslisp interface for CollisionDetectorService.idl #37
* Comment out camera port connection in the simulator because it lacks lack versatility
* Changed StateHolder name from StateHolder to sh.
  Modified rviz argument from --sync to -sync. In electric, rviz may not work correctly.
* add emergence_stop.py
* adopted hrpsys_dashboard to fuerte.
* fix virtual force sensor setup
* check High Temperature
* mv SensorInfo to Impl, add virtual force sensor information
* support virtual_force_sensor option
* split diagnostics.py into diagnostics.py and  motor_states.py
* add calib done, power on, servo alarm message to diagnostics
* add motor section
* write motor status to diagnostic
* add power indicator
* send servo alarm
* fix : update for hrpsys-base update
* update for hrpsys-base update
* fix hrpsys_profile, clean up codes
* fix gyrometer and gsensor to use TimedAcceleratoin3D and TimedAngularVelocity3D
* add darwin-vrml-file-path to reduce webots dependent path description
* use ros::resolve-ros-path instead of defined function get-rospack-find-str
* fix only indent
* fix file-name probing committed at r3127 ;; probe-file webots original vrml file instead of generated openhrp3 path
* ignore if DARwInOp.proto is not found
* set default rate to 10 ; this process consumes cpu power so much
* pre calculate sensor tf information
* remove old features, diagnositcs and JointTempleture, now we can you hrpsys_profile.py and servoState
* add extra_data for hrpsys-base revision 417
* read force sensor size and name from model file
* fix due to hrpsys update
* clean up code, add error handling code
* add diagnostics.py, diagnostics_aggregator and hrpsys_profile
* add hrpsys_profile.py
* set red icon if servo off
* fix servoState data type from TimedLongSeq -> TimedLongSeqSeq
* fix servoState data type from TimedLongSeq -> TimedLongSeqSeq
* specify simulator and robothardware names by using SIMULATOR_NAME
* fix usage of StateHolder ;; use qIn and qOut as reference angle-vector value ;; fix related launch files
* round time value ;; we assume time stamp in walk2m.pos is integer
* use rootlink_name, since calling bodyinfo->links() in onExecute consumes memory?
* call goActual before servo on
* serv off is not error
* add Mode
* add diagnostics.py to publish /Mode/Operating Mode/{Servo On,Servo Off}
* pusblish odom->bodyinfo->links()[0].segments[0].name
* fix HRP4C walk2m.pos data path
* fix HRP4C model path
* add servoState
* add servoState
* fix typo hrpsys_dashboarad -> hrpsys_dashboard
* fix servo on/servo off
* use OpenRTM timer for publish ROS message
* fix link_name -> name for new openhrp3 3.1.3
* use args for realtime_cpu_monitor.py
* fix params robot_monitor
* add robot_monitor and diagnostic.yaml
* add hrpsys_dashboarad
* add depends to dashboard/robot_monitor
* add diagnostics_pub
* use hrpEC
* start up DataLoggerService in hrpsys ros bridge launch
* add save log method
* add KILL_SERVERS arguments to samplerobot_startup.launch
* support KILL_SERVERS argument, set false as default
* subscribe motor_states in initializatoin ;; add :temperature-vector
* define rtm-ros-robot-interface to use hrpsys_ros_bridge dependent codes
* add MotorStates.msg and publish it in HrpsysSeqRosBridge
* remove webots from depends and add add_custom_command to download the sourcecode
* reverted to r2951 for my mistake
* check warning only for non-eistence movie or image
* fixed index.rst and conf.py conflict problem
* use baseTform instead of pose to obtain base link transformation from StateHolder
* add JointTemperature data port ;; this data port does not work if Joint Temperature is input
* update index.rst,conf.py by Jenkins
* add manipulator for Yaskawa robot
* fix: upport manipulator definition when export-collada, HRP4C is the example
* support manipulator definition when export-collada, HRP4C is the example
* make build/images for rostest
* comment out torque reading printing by default
* add data port for actual torques ;; rename original out data port for torque as mctorque ;; publish joint_states.effort based on rstorque
* publish /fullbody_controller/state according to mcangle data port
* rename compile_collada_robot->compile_collada_model
* move compile_openhrp_model,compile_collada_robot to cmake/compile_robot_model.cmake
* remove unused m_angle data port
* increase max-time for gr001
* fix typo hr001->gr001
* increase time-limit
* increse time-limit
* increase time-limit
* update samplerobot in house example to use grxui
* fix video tag for rviz
* fix for old cmake version issue #22
* fix typo, errors
* add to generate pr2.xml and pr2.conf
* generate conf file from wrl for openhrp3 model
* fix dependency
* mv smartpal5.yaml yaskawasmartpal5.yaml
* update CMakeLists.txt in order to run compile_robot sequentially, since this target use rostest and we could not run multiple rostest at the same time
* fix xml
* update launch script for eclipse-free hrpsys simulation
* update make file to use _gen_project.launch to generate project file from model file
* update documents
* use copy for YasukawaSmartPal5
* enable Camera of darwin.vcg avoid segfo in lucid
* fix makefile for YasukawaSmartPal5
* update launch documents
* rtm-naming-restart now do not ru omniNaming in background, so that rosmake can easily kill rtm-naming
* fix typo ;; openhrp -> openhrp3
* use openhrp3_PACKAGE_PATH : try to call _stop_rtmnaming.launch at the end of the make test, to stop omniNames that confuses rosmake system
* try to call _stop_rtmnaming.launch at the end of the make test, to stop omniNames that confuses rosmake system
* revert to previous revision ;; ros name server must initialize before RTCsvn diff src/
* fix order of member variables initialization
* fix: rosbuild_download_data use download_checkmd5, wrong previous commit -> use download_checkmd5 instad of rosbuild_donwnload_data to try to download multiple times when fail
* use download_checkmd5 instad of rosbuild_donwnload_data to try to download multiple times when fail
* update test script
* use video directive
* add dependency on pr2eus because euslisp codes and msgs of pr2eus are used in examples
* update launch script for new test-grxui.py
* update time-limit
* fix test setup for max-time
* fix max-time for new jenkins machine
* fix max-time
* add off-pose
* change reset-pose, in order not to collide with table
* change service play pattern
* remove waitforClockGenerator
* update time-limit for gr001
* update gr001.launch
* update time-limit
* update gr001 sample program
* add gr001.launch to test
* add GR001 model from choreonoid
* set time-limit to 600
* add pr2.launch to rostest
* set ORBgiopMaxMsgSize to 2M to 20M
* do not error when length of rsangle is larger then that of body->joints
* remove retry option from test tag
* fix dependency
* fix dependency when downloading yaskawa model
* path of robot-interface.l has changed
* fix dependency
* remove debug code
* remove pr2 from test
* add dirty patch
* fix wait for ModelLoader code
* fix check-tf
* add debug message
* add lfsensor and rfsensor
* wait until ModelLoader is up
* remove debug code
* fix debug code
* remove debug code
* add pr2.launch for rostset
* clean up code
* fix camera position
* fix path of webots robot directory ;; projects -> resources/projects
* remove webots nodes if openhrp-node-name is not specified ;; does not export Receiver and Emitter to openhrp vrml
* capitalize bottom, side, and top fields of cylinder
* remove subdivision field from Cylinder Node in webots vrml because subdivision field does not exist in default VRML specification
* add definition of Receiver PROTO and Emitter PROTO refering to reference.pdf in webots_simulator
* fix path of darwin proto
* rename webots -> webots_simulator because original directories or webots-ros-pkg are updated
* use --nosim mode for pr2_startup
* rename webots -> webots_simulator because original directories or webots-ros-pkg are updated
* fix typo
* add darwin-walk-data and update darwin sample ;; init -> walk -> pickup -> arm-up pose
* remove unnecessary timestep argument ;; darwin simulation does not require this parameter by default at recent version
* add angle-vector-for-seq-debug version instead of angle-vector-sequence version for hrp4c-walk debug
* fix linear mode check ;; use :linear instead of linear
* fix linear mode check ;; use :linear instead of linear
* add raw service call for seq's waitInterpolation and setJointAngles
* add hrp4c-walk in hrp4c-pickup sample script
* setPreempted on Finalize
* remove debug code
* support angle-vector-sequence and wait-interpolation
* lengthen max-time for hrp4c test
* speed up darwin demo
* add fullbody pickup sample for hrp4c robot ;; use hrp4c-fullbody-pickup by default
* update hrp4c end-coords definitions
* update darwin simulation
* update for new visualization layout
* add config for rviz
* fix euslisp code for better visualization
* update hrp4c.vcg
* remove debug code
* set timestep 0.005 for darwin
* add pr2 simulation on openhrp3
* add joint_state.velocity
* update max-time
* update documents for smartpal5.launch
* add smartpal5.launch to rostest
* support any size of input image
* set max-time to 40 for dynamics simulated robot, 10 for no-sim robot
* fix debug message
* update darwin script
* add arg OUTPUT
* update launch files
* add dependency to image_view2
* fix euslisp model location
* add smartpal5.yaml
* add smartpal5 launch files
* add to download yasukawa model
* update webotsvrml2openhrp3vrml according to changing directory
* move webotsvrml2openhrp3vrml from models to scrips
* add scripts/webotsvrml2openhrp3vrml.l
* move darwin.yaml from scripts to models
* move model files from hrpsys to hrpsys_ros_bridge
* add pa10, darwin, damplerobot launch example, rename file name based on description written in manifest.xml
* add pa10, darwin, damplerobot launch example, rename file name based on description written in manifest.xml
* wait until model loader is available
* fix document
* fix xml syntax
* add hrp4c-nosim-pickup.launch and update hrp4c-pickup.l
* remove hrp4c-example.launch
* update launch description
* fix max-time to 20
* enable to specify service-name
* update debug message
* update hrp4c-\*.launch files
* rename file name from test-hrp4c-\*.launch to hrp4c-\*.launch
* fix xml syntax error
* fix documnets
* add test files
* rename hrp4crobot-pickup -> hrp4c-pickup
* remove test directory use rosdoc and launchfiles
* add rtm-ros-robot-interface including ros::service-call and use it in roseus samples ;; rtm-ros-robot-interface will support idl<->srv conversion
* use require instead of load for model loading
* add sendmsg service to Hrpsysseqstaterosbridge using dynamic_reconfigure srv
* move darwin's hrpsys.launch file to hrpsys_ros_bridge
* fix hrp4c.test and CMakeList.txt
* add darwin sample launch files and script
* add test program for hrpsys_ros_bridge
* add rostest hrp4c.test and generate-document.test
* update hrpsys_ros_bridge.launch : remove SimulationEC from openrtm_args and rename rtmlaunch to hrplaunch_hrpsys_ros_bridge
* revert previsou commit
* update hrp4c.vcg
* changed nodes in hrpsys_ros_bridge to not link SimulationEC, these nodes are not
  real-time
* current good connection script for hrp4r
* add connection script for hrpsys_ros_bridge on real robot
* unit of Update Intealval is [sec]
* changed robot super class, from ros-interface to robot-interface
* add --nosim launch file for hrp4c
* add pickup demo for hrp4c
* fix : forget to call spinOnce
* use Update Interval 1 for Robot Model and TF
* update darwin_ros_bridge.launch to use hrpsys_ros_bridge.launch
* fix compile warning
* fix wrong commit
* clean up HrpsysSeqStateROSBridge
* pusblish tf and others at the hrpsys cycle rate
* add --sync
* add SCHEDULER_ARGS argument support
* fix to pass SCHEDULER_ARGS argument
* fix debug message
* clean up launch files, hrpsys/launch contains example for OpenHRP3.1 and hrpsys_ros_bridge/launch contains example for openhrp-scedular and ros bridge
* move openhrp + ros integration example under hrpsys_ros_bridge
* rename main_withHands to HiroNX
* add hiro-nx example
* add debug message to show @Initialize
* change samplerobot_hrpsys_ros_bridge to use hrpsys.launch
* add rviz, image_proc, image_view, camshiftdemo in samplerobot_ros_bridge.launch
* remove openhrp3 from manifest, this is writtin in hrpsys/manifest.xml
* fix ROBOT_NAME from HRP-4C to HRP4
* fix samplerobot_ros_bridge.launch to include hrpsys_ros_bridge.launch
* add debug message
* update .launch files, .dae files goes to hrpsys/scripts, AbsTransformToPosRpy is launched from hrpsys.launch, update opertm_args to use SimulationEC
* update skip paramater
* link SimulationEC
* add depend package rviz to manifest.xml
* add hrpsys_ros_bridge
* add launch/darwin_ros_bridge.launch
* add ImageSensorROSBRidge to hrp4 simulation
* add rviz configuration file for samplerobot
* fix to avoid segfualt error when grxui does not publish data
* add frame_id, and fill dummy camera_info param, publish effort and use wait-interpolation
* publish sensor pose through tf::TransformBroadcaster
* add ImageSensorROSBridge
* add samplerobot_ros_bridge.launch
* update HrpsysSeqStateROSBridge read pose and publish as tf
* update use /odom as root frame
* add rviz configuration files for pa10 and hrp4c
* add pa10_ros_bridge.launch
* update to HrpsysSeqStateROSBridge
* update hrpsysStatePublihser, to work with robot_state_publisher
* rename hrprtc -> hrpsys
* rename hrprtc_ros_bridge -> hrpsys_ros_bridge
* Contributors: Atsushi.tsuda@gmail.com, Isao Isaac Saito, Kei Okada, Ryohei Ueda, chen.jsk@gmail.com, garaemon, garaemon@gmail.com, gm130s@gmail.com, ik0313, ik0313@gmail.com, kei.okada, kei.okada@gmail.com, manabu1118, manabu1118@gmail.com, murooka@jsk.imi.i.u-tokyo.ac.jp, nakaokat@gmail.com, notheworld, notheworld@gmail.com, nozawa, rtmrosrobotics.testing@gmail.com, tnaka, yk.at.jsk.@gmail.com, yk.at.jsk@gmail.com, youhei, youhei@jsk.imi.i.u-tokyo.ac.jp
