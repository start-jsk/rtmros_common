^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2017-12-22)
------------------
* add test to check `#1036 <https://github.com/start-jsk/rtmros_common/issues/1036>`_ situation (`#1038 <https://github.com/start-jsk/rtmros_common/issues/1038>`_)
* duration == 0.0 results unexpected behavior, so force set 0.001 when duration == 0 s set (`#1037 <https://github.com/start-jsk/rtmros_common/issues/1037>`_ )
* Contributors: Kei Okada

1.4.1 (2017-10-25)
------------------
* Update of sensor_ros_bridge_connect.py
  * Connect ReferenceForceUpdater's ref_force (`#1033 <https://github.com/start-jsk/rtmros_common/pull/1033>`_)
    * [hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py] connect reference force port to ReferenceForceUpdater or EmergencyStopper if exists.
  * Add wait for rmfo and vs (`#1029 <https://github.com/start-jsk/rtmros_common/pull/1029>`_)
    * [hrpsys_ros_bridge, sensor_ros_bridge_connect.py] add wait for initalizinig all components

* HrpsysSeqStateROSBridge (`#1027 <https://github.com/start-jsk/rtmros_common/pull/1027>`_)
  * [HrpsysSeqStateROSBridge] fix for using virtual force sensor

* Update rtm-ros-robot-interface.l (`#1030 <https://github.com/start-jsk/rtmros_common/pull/1030>`_)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add documentation string for new arguments (set-ref-force-linear-p, return-value-mode)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add argument to set linear interpolation during increasing force
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Enable to set return value mode
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Enable to get fric coefficient wrench add update print messages
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update otd checking loop for moment. Exit from do-until-key loop immediately when otd detection. Fix valiables for set ref force
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Use euslisp symbol as detector total wrench parameter
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Enable to access detector total wrench as euslisp symbol.
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update otd checking loop. Exit from do-until-key loop immediately when otd detection.

* Contributors: MasakiMurooka, Shunichi Nozawa, YoheiKakiuchi

1.4.0 (2017-08-09)
------------------
* display error message when body->sensor(j, i) is null (`#1014 <https://github.com/start-jsk/rtmros_common/issues/1014>`_)
* sensor_ros_bridge_connect.py : add more debug message (`#1015 <https://github.com/start-jsk/rtmros_common/issues/1015>`_)
* Update AutoBalancer and ReferenceForceUpdater euslisp method symbols (`#1026 <https://github.com/start-jsk/rtmros_common/issues/1026>`_)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Supprot :FootOriginExtMoment for RFU methods.
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Support UseForceMode setting as euslisp symbol in :set-auto-balancer-param.

* fix code to run both Indido and Kinetic (`#1025 <https://github.com/start-jsk/rtmros_common/issues/1025>`_)
  * /test/test-samplerobot-hcf.launch: increase time-limit to 600
  * test/{test-samplerobot.py,test-pa10.py} support both pr2_controllers_msgs and controllr_msgs
  * src/hrpsys_ros_bridge/hrpsys_dashboard.py: fix for qt5
  * add USE_PR2_CONTROLLERS_MSGS definition
  * CMakeLists.txt : we do not use pr2_msgs on build time
  * CMakeLists.txt : remove code to download wet pr2_controllers_msgs for groovy

* Support stride parameter with different length (`#1022 <https://github.com/start-jsk/rtmros_common/issues/1022>`_)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Support stride_parameter of different length
  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-auto-balancer.l] Add test to check stride parameter 4 and 6

* [HrpsysSeqStateROSBridge] fix stamp of joint_state. add stamp after reading m_rsangle (`#1019 <https://github.com/start-jsk/rtmros_common/issues/1019>`_)
  * [HrpsysSeqStateROSBridge] fix stamp of joint_state. add stamp after reading m_rsangle

* [HrpsysSeqStateROSBridge] remove subtraction magic number (`#1013 <https://github.com/start-jsk/rtmros_common/issues/1013>`_)
  * [HrpsysSeqStateROSBridge] remove subtraction magic number

* Support argumen t for setting duration of calibration. For :remove-xx methods, 8.0 by default[s]. For :reset-xx methods, 0.1[s] by default for compatibility (`#1011 <https://github.com/start-jsk/rtmros_common/issues/1011>`_)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Support argument for setting duration of calibration. For :remove-xx methods, 8.0 by default[s]. For :reset-xx methods, 0.1[s] by default for compatibility

* Add new methods for :remove-force-sensor-offset using RMFO (`#1010 <https://github.com/start-jsk/rtmros_common/issues/1010>`_)
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add new methods for :remove-force-sensor-offset using RMFO. Add warning message for deprecated remove-force methods.

* Update euslisp hrpsysbase samples (`#1009 <https://github.com/start-jsk/rtmros_common/issues/1009>`_)
  * [hrpsys_ros_bridge/test/hrpsys-samples] Add README for Euslisp hrpsys example basically copied from ros wiki (http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp)
  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-carry-object.l] Update carry demo euslisp sample.
  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-stabilizer.l] Update stabilizer euslisp sample according to hrpsys-base stabilizer sample update.

* Fix bug of sample4leg robot end-coords setting for arms. (`#1008 <https://github.com/start-jsk/rtmros_common/issues/1008>`_)
  * [hrpsys_ros_bridge/models/sample4legrobot.yaml] Fix bug of sample4legrobot end-coords setting for arms.

* Update project generator and refforce (`#1007 <https://github.com/start-jsk/rtmros_common/issues/1007>`_)
  * [hrpsys_ros_bridge/euslisp] Update README for openhrp-project-generator
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Fix orientation for openhrp-project-generator
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update for object turnaround detection to use initial ref forces
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update dump function for openhrp project generator. Use openhrp-project-generator in openhrp3 package instead of _gen_project.launch

* Add forgotten update for robot.launch (https://github.com/start-jsk/rtmros_common/pull/1004) (`#1005 <https://github.com/start-jsk/rtmros_common/issues/1005>`_)
  * [hrpsys_ros_bridge/scripts/default_robot.launch.in] Add forgotten update in https://github.com/start-jsk/rtmros_common/pull/1004. Add USE_XXX of under-development RTCs for robot.launch such as samplerobot.launch to pass arguments to robot_ros_bridge.launch

* set USE_UNSTABLE_RTC as not all unstable rtc defiend in hrpsys_config.py, just for basic walking test, other latest development rtc is passed by arg (`#1004 <https://github.com/start-jsk/rtmros_common/issues/1004>`_)
  * set USE_UNSTABLE_RTC as not getUnstableRTC defined in hrpsys_config.py, it is for a basic walking test, other latest development rtc is passed by arg from test_samplerobot_euslisp_unittests.launch
  * [hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch,hrpsys_ros_bridge/scripts/default_robot_ros_bridge.launch.in,hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l,hrpsys_tools/launch/hrpsys.launch] Update for ObjectTurnaroundDetector RTC. add USE_REFERENCEFORCEUDPATER and USE_OBJECTCONTACTTURNAROUNDDETECTOR.

* Contributors: Kei Okada, Shunichi Nozawa, Yohei Kakiuchi

1.3.3 (2017-02-12)
------------------

* HrpsysSeqStateROSBridge.cpp
  * HrpsysSeqStateROSBridge occurs error if number of force sensor is different with size of contact states. fix number of force sensor (`#982 <https://github.com/start-jsk/rtmros_common/issues/982>`_)
  * Revert "[HrpsysSeqStateROSBridge] fix number of force sensor" `#984 <https://github.com/start-jsk/rtmros_common/issues/984>`_
  * Separate offset-removed force sensor data ports from rsforce, which is actual and non-offset-removed data port. Currently rsforce ports -> rsforce ports + offforce ports. (`#964 <https://github.com/start-jsk/rtmros_common/issues/964>`_)
  * Fixed mistake of sensor_link_name, when robot has only virturl force sonsors (`#945 <https://github.com/start-jsk/rtmros_common/issues/945>`_)

* HrpsysJointTrajectoryBridge.cpp
  * publish feedback for joint_trajectory_actions (`#966 <https://github.com/start-jsk/rtmros_common/issues/966>`_)

* hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch
  * Add argument to change buffer length of rtmlaunch connection. Set 8 by default (default setting in openrtm). (`#980 <https://github.com/start-jsk/rtmros_common/issues/980>`_)
  * fix model filename. Enable cache in ModelLoader, filename should be same as in .conf file. (`#978 <https://github.com/start-jsk/rtmros_common/issues/978>`_)

* euslisp
  * [rtm-ros-robot-interface.l] refactor torque-controller interface (`#994 <https://github.com/start-jsk/rtmros_common/issues/994>`_)
  * Update sequencefull unit system and arguments (`#988 <https://github.com/start-jsk/rtmros_common/issues/988>`_)
    * changed arguments of angle-vector-sequence-full see `#985 <https://github.com/start-jsk/rtmros_common/issues/985>`_
    * [hrpsys-samples/samplerobot-sequence-player.l] Add test for angle-vector-sequence-full (check validity)
    * [rtm-ros-robot-interface.l] Use root-coords and zmp in the abs frame and convert them inside the :angle-vector-sequence-full
    * [rtm-ros-robot-interface.l] Introduce fsensor length for wrench and optionals
    * [rtm-ros-robot-interface.l] Use [mm] and [deg/s] instead of [m] and [rad/s] for vel, pos, and zmp.
    * [rtm-ros-robot-interface.l] Use [deg] instead of [rad] for jpos argument and remove jvs arg.
    * [rtm-ros-robot-interface.l] Use tm as [ms], previously [s].

  * [tm-ros-robot-interface.l] Update :reset-force-moment-offset to use :off force/moment vector (`#983 <https://github.com/start-jsk/rtmros_common/issues/983>`_)
  * [tm-ros-robot-interface.l] Added a missing keyword in def-set-get-method for TorqueControllerService. (`#972 <https://github.com/start-jsk/rtmros_common/issues/972>`_)
  * [tm-ros-robot-interface.l] Add euslisp interface for torque controller (`#965 <https://github.com/start-jsk/rtmros_common/issues/965>`_)
  * [datalogger-log-parser.l] Modify unit of cogvel [m] -> [mm] for euslisp (`#960 <https://github.com/start-jsk/rtmros_common/issues/960>`_)
  * [rtm-ros-robot-interface.l] Add get-go-pos-footsteps-sequence in rtm-ros-robot-interface.l and add test (`#965 <https://github.com/start-jsk/rtmros_common/issues/965>`_)
  * [rtm-ros-robot-interface.l] Update frame of wrench. Previously, wrench frame is mismatch with documentation string. Previous : local -> new : world. (`#963 <https://github.com/start-jsk/rtmros_common/issues/963>`_)
  * [rtm-ros-robot-interface.l] Add eefm force moment distribution weight parameter. (`#958 <https://github.com/start-jsk/rtmros_common/issues/958>`_)
  * [rtm-ros-robot-interface.l] add Eus interface of SequencePlayserService_setJointAnglesSequenceFull and add test in test/hrpsys-samples/samplerobot-sequence-player.l (`#954 <https://github.com/start-jsk/rtmros_common/issues/954>`_)
  * [datalogger-log-parser.l] Add sh reference force and support both rpy and 3x3 matrix for WAIST log(`#956 <https://github.com/start-jsk/rtmros_common/issues/956>`_)
  * [rtm-ros-robot-interface.l] update rfu interface for idl changed in https://github.com/fkanehiro/hrpsys-base/pull/1005 (`#952 <https://github.com/start-jsk/rtmros_common/issues/952>`_)
  * [datalogger-log-parser.l] Use joint-list length for datalogger joint angle conversion. (`#953 <https://github.com/start-jsk/rtmros_common/issues/953>`_)
    * Add simulation actual root coords reading and check for existance of RobotHardware0_servoState in datalogger. Fix setting of simulators' data parser names.
    *  Use joint-list length for datalogger joint angle conversion.
  * [rtm-ros-robot-interface.l] make directory for log files (`#951 <https://github.com/start-jsk/rtmros_common/issues/951>`_)

* [scripts/default_robot_ros_bridge.launch.in] Add nameserver argument for ros_bridge.launch (`#950 <https://github.com/start-jsk/rtmros_common/issues/950>`_)
* [test/hrpsys-samples/samplerobot-stabilizer.l] Update loadPattern sample because add-optional-data-from-rs-list is included in dump-seq-pattern-file (`#959 <https://github.com/start-jsk/rtmros_common/issues/959>`_)

* Contributors: Ryo Koyama, Kei Okada, Noriaki Takasugi, Ryo Terasawa, Shunichi Nozawa, Yohei Kakiuchi, Yoshimaru Tanaka, Iori Kumagai, Iori Yanokura, Juntaro Tamura

1.3.2 (2016-04-26)
------------------

* [fix] getFootStepParam interface according to upstream change. Remove rleg_coords and lleg_coords.
* [fix] HrpsysSeqStateROSBridgeImpl.cpp: sensor->localR is world coords
* [fix] Modified HrpsysSeqStateROSBridgeImpl to enable virtual force
* [fix] [rtm-ros-robot-interface.l] add time constant prameter according to upstream idl update
* [fix] [hrpsys_ros_bridge] Fix genjava problem by adding message_generation as build_depend
* [fix] getFootStepParam interface according to upstream change. Remove rleg_coords and lleg_coords.
* [fix] [hrpsys_ros_bridge]change D if D from hrpsys is empty
* [feat] [hrpsys_ros_bridge/test/test-samplerobot.py] add test programs to check frame_id of off_xxsensor and ref_xxsensor (#940)
* [feat] Set /robot/type param according to lower-case robot name such as samplerobot.
* [feat] [hrpsys_ros_bridge/test/test-samplerobot.*] add a test program for init of hcf
* [feat] fix collision visualization, color of sphere will be purple if collision occur
* [feat] .travis.yml : add test when old hrpsys-ros-bridge exists (#929)
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add check for param existence for def-set-get-param-method to neglect idl mismatch error. Fix indent. (#933)
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add reference force updater euslisp methods.
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add object turnaround detector moment version interface.l
* [feat] Add define controller `#887 <https://github.com/start-jsk/rtmros_common/issues/887>`_
* [feat] Add leg limb controller setting.
* [feat] Add method to define joint trajectory controller by default setting.
* [feat] Add gopos overwrite and graspless manip mode to all demos
* [feat] Add example for graspless manip mode and gopos overwrite.
* [feat] Add euslisp interface for graspless manip mode.
* [feat] add a set-default-step-time-with-the-same-swing-time method
* [feat] add time constant prameter according to upstream idl update
* [feat] Add HrpsysSeqStateROSBridge tf rate test
* [feat] enable virtual force in HrpsysSeqStateROSBridgeImpl
* [feat] Enable to set subscription_type for DataPorts from hrpsys_ros_bridge.launch argument. Use new by default to keep backward compatibility.
* [feat] Add loading of optionalData from seq pattern file.
* [improve] add more mesage to prevent confusion
* [improve] Add setting for ReferenceForceUpdater
* [improve] Reset object turnaround detector's detector total wrench mode as TOTAL_FORCE in force estimation.
* [improve] Enable to set push_policy and push_rate for DataPorts from hrpsys_ros_bridge.launch argument. Use all and 50.0 by default to keep backward compatibility.
* [improve] Rename tf extract script for test and add comments for that
* [improve] add arguments(SIMULATOR_NAME\_[ANGLE,VELOCITY,TORQUE]) to hrpsys_ros_bridge.launch, for connecting rtc components other than RobotHardware
* [improve] add time constant prameter according to u… `#910 <https://github.com/start-jsk/rtmros_common/issues/910>`_
* [improve] add argument to set periodic time for object turning detection.
* [improve] define method to set/get emergency-stopper-paramMerge pull request `#865 <https://github.com/start-jsk/rtmros_common/issues/865>`_
* [improve] Suppress /tf publishing rate by tf_rate using Timer callback in ros.
* [improve] Separate updating odometry and imu into functions
* [improve] Update st param `#894 <https://github.com/start-jsk/rtmros_common/issues/894>`_
* [improve] add eefm_swing_rot_spring_gain / eefm_swing_pos_spring_gain as st param
* [improve] Update joint group and add test for limb controller
* [doc] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] In-code documentation improvement
* [doc] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Set default documentation string for def-set-get-param-method setter and getter.
* [doc] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add argument documentation for :set-xxx methods.
* [doc] [hrpsys_ros_bridge/euslisp/README.md,hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update readme and euslisp documentation strings
* [doc] default documentation string for def-set-get-param-method setter and getter.
* [doc] In-code documentation improvement
* Contributors: Eisoku Kuroiwa, Kei Okada, Kentaro Wada, Masaki Murooka, Shunichi Nozawa, Yohei Kakiuchi, Yu Ohara, Iori Yanokura

1.3.1 (2015-12-16)
------------------
* deb release only targeting to indigo

* Fix for travis testing

  * [hrpsys_ros_bridge/test/test-samplerobot.test] Use joint_states instead of odom to check tf because joint_states is more related with tf and [Hz] printing. than odom `#880 <https://github.com/start-jsk/rtmros_common/pull/880>`_
  * [hrpsys_ros_bridge/test/test-samplerobot.test] Increase hzerror according to https://github.com/start-jsk/rtmros_common/issues/877#issuecomment-164669534. Current worst travis hz seem to be more than 300[Hz], so set 200[Hz] error. `#879 <https://github.com/start-jsk/rtmros_common/pull/879>`_
  * catkin.cmake: use ccache only for CI environment `#872 <https://github.com/start-jsk/rtmros_common/pull/872>`_
  * add depends from AutoBalancerService.hh to StabilizerService.hh `#872 <https://github.com/start-jsk/rtmros_common/pull/872>`_
  * [hrpsys_ros_bridge/test/hrpsys-samples/test_samplerobot_euslisp_unittests.launch] Increase time-limit for autobalancer euslisp test `#879 <https://github.com/start-jsk/rtmros_common/pull/879>`_

* Fasten script excution

  * [hrpsys_ros_bridge/scripts/sensor_ros_bridge_connect.py] Reduce unnecessary waiting for sensor port rosbridge connection. `#879 <https://github.com/start-jsk/rtmros_common/pull/879>`_

* Bug fix

  * [hrpsys_ros_bridge/src/HrpsysSeqStateROSBridge.cpp] Initialize prev_odom_acquired flag. `#879 <https://github.com/start-jsk/rtmros_common/pull/879>`_

* Add euslisp new example and update for example conf setting

  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-stabilizer.l] Add Stabilizer + loadPattern example for euslisp interface. `#875 <https://github.com/start-jsk/rtmros_common/pull/875>`_
  * [hrpsys_ros_bridge/catkin.cmake] Add Sequencer's optionalData setting for sample conf files. `#875 <https://github.com/start-jsk/rtmros_common/pull/875>`_

* Contributors: Kei Okada, Shunichi Nozawa

1.3.0 (2015-12-09)
------------------
* deb release only targeting to indigo

* [.travis.yml, hrpsys_ros_bridge/catkin.cmake] Enable travis euslisp test using USE_DEB=source and reducing make jobs and loads `#806 <https://github.com/start-jsk/rtmros_common/issues/806>`_

* Bug Fixes

  * [hrpsys_ros_bridge/euslisp/datalogger-log-parser.l] modify string conversion in RobotHardware0 (for robot name containing '_') `#848 <https://github.com/start-jsk/rtmros_common/issues/848>`_
  * [hrpsys_ros_bridge/euslisp/datalogger-log-parser.l] Add RobotHardware0_servoState data to robot state `#853 <https://github.com/start-jsk/rtmros_common/issues/853>`_
  * [hrpsys_ros_bridge/euslisp/datalogger-log-parser.l] Update unitsysetm conversion of angle-vector (rad->deg, m->mm) `#845 <https://github.com/start-jsk/rtmros_common/issues/845>`_
  * [hrpsys_ros_bridge/catkin.cmake] Fix typo for hrpsys_PREFIX `#843 <https://github.com/start-jsk/rtmros_common/issues/843>`_
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Return detection mode. `#841 <https://github.com/start-jsk/rtmros_common/issues/841>`_
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Update set force method. Add reset force. `#840 <https://github.com/start-jsk/rtmros_common/issues/840>`_
  * [hrpsys_ros_bridge/catkin.cmake] Update PDgain conf property name `#816 <https://github.com/start-jsk/rtmros_common/issues/816>`_
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Fix for pos and rot ik thre `#803 <https://github.com/start-jsk/rtmros_common/issues/803>`_
  * [cmake/compile_robot_model.cmake] move location of set variable (compile_all_target) `#779 <https://github.com/start-jsk/rtmros_common/issues/779>`_
  * [src/hrpsys_ros_bridge/hrpsys_dashboard.py] Run event loop for 1 sec after showing splashwindow to force to load image immediately `#762 <https://github.com/start-jsk/rtmros_common/issues/762>`_
  * [euslisp/datalogger-log-parser.l] Warn message if wrong path is specified `#767 <https://github.com/start-jsk/rtmros_common/issues/767>`_
  * [euslisp/datalogger-log-parser.l]  Add limb-list first, Add st and abc properties to datalogger log parser `#761 <https://github.com/start-jsk/rtmros_common/issues/761>`_

* /off_xxsensor and /xxsensor

  * [hrpsys_ros_bridge/src/HrpsysSeqStateROSBridge.cpp] fix  /off_xxsensor frame_id `#867 <https://github.com/start-jsk/rtmros_common/issues/867>`_
  * [hrpsys_ros_bridge/src/HrpsysSeqStateROSBridge.cpp] fix frame_id of off_xxsensor `#855 <https://github.com/start-jsk/rtmros_common/issues/855>`_

* euslisp/rtm-ros-robot-interface.l : add/modify interfaces for new/updated rtc

  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-auto-balancer.l] Use 4limbs in startAutoBalancer when Groups has rarm and larm and  update sample. `#866 <https://github.com/start-jsk/rtmros_common/issues/866>`_
  * add reference remaining_time in /ref_contact_states `#857 <https://github.com/start-jsk/rtmros_common/issues/857>`_
  * Add method of servo-state `#853 <https://github.com/start-jsk/rtmros_common/issues/853>`_
  * Add calc velocity from velocity offset `#842 <https://github.com/start-jsk/rtmros_common/issues/842>`_
  * Add euslisp style argument for leg_default_translate_pos `#836 <https://github.com/start-jsk/rtmros_common/issues/836>`_
  * Add grasp controller interface methods.l `#831 <https://github.com/start-jsk/rtmros_common/issues/831>`_
  * Enable to set optional-data from robot and contact-state `#831 <https://github.com/start-jsk/rtmros_common/issues/831>`_
  * Add optionalData from euslisp rs-list `#822 <https://github.com/start-jsk/rtmros_common/issues/822>`_
  * add stabilizer-end-coords-list `#820 <https://github.com/start-jsk/rtmros_common/issues/820>`_
  * Update setter methods `#818 <https://github.com/start-jsk/rtmros_common/issues/818>`_
  * Add ":set-foot-steps-with-param-and-base-height" method in order to specify footstep with params and base height `#811 <https://github.com/start-jsk/rtmros_common/issues/811>`_
  * update default-zmp-offsets interface according to hrpsys-base change `#808 <https://github.com/start-jsk/rtmros_common/issues/808>`_
  * add an interface for multi legs footsteps `#795 <https://github.com/start-jsk/rtmros_common/issues/795>`_
  * get ref-capture-point from Stabilizer rtco `#794 <https://github.com/start-jsk/rtmros_common/issues/794>`_
  * update AutoBalancerParam according to https://github.com/fkanehiro/hrpsys-base/pull/750 `#786 <https://github.com/start-jsk/rtmros_common/issues/786>`_
  * get cp from Stabilizer rtc `#781 <https://github.com/start-jsk/rtmros_common/issues/781>`_
  * Update footstep methods to enable overwriting `#775 <https://github.com/start-jsk/rtmros_common/issues/775>`_
  * get emergency-mode from robot-interface `#772 <https://github.com/start-jsk/rtmros_common/issues/772>`_
  * Add calculation of sole edge `#759 <https://github.com/start-jsk/rtmros_common/issues/759>`_

* Add PointCloudROSBridge and RangeSensorROSBridge, and modify ImageSensorROSBridge `#785 <https://github.com/start-jsk/rtmros_common/issues/785>`_

  * [src/RangeSensorROSBridge.cpp] add intensity parameter for adding intensities `#802 <https://github.com/start-jsk/rtmros_common/issues/802>`_
  * [src/ImageSensorROSBridge.cpp] fix parsing camera parameter `#798 <https://github.com/start-jsk/rtmros_common/issues/798>`_
  * [src/ImageSensorROSBridge.cpp] fix bug / reset debug counter `#797 <https://github.com/start-jsk/rtmros_common/issues/797>`_
  * [hrpsys_ros_bridge/cmake/compile_robot_model.cmake] Use collada_urdf_jsk_patch in order to convert dae into urdf which can be used in gazebo `#793 <https://github.com/start-jsk/rtmros_common/issues/793>`_
  * [src/ImageSensorROSBridge.cpp] add camera param setter by ros parameter `#791 <https://github.com/start-jsk/rtmros_common/issues/791>`_
  * [src/PointCloudROSBridge.cpp] fix PointCloudROSBridge, color and frame `#790 <https://github.com/start-jsk/rtmros_common/issues/790>`_
  * [src/RangeSensorROSBridge.cpp] fix RangeSensorROSBridge / time_increment is zero `#788 <https://github.com/start-jsk/rtmros_common/issues/788>`_
  * [src/RangeSensorROSBridge.cpp] fix frame of RangeSensor `#789 <https://github.com/start-jsk/rtmros_common/issues/789>`_

* Add rsvel port (`#807 <https://github.com/start-jsk/rtmros_common/issues/807>`_)

  * [src/HrpsysSeqStateROSBridge.cpp, launch/hrpsys_ros_bridge.launch] add rsvel port to HrpSeqStateROSBridge read dq in order to add velocity values to joint_states
  * [hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch] add USE_VELOCITY_OUTPUT to arguments of rtmlaunch `#821 <https://github.com/start-jsk/rtmros_common/issues/821>`_

* Add ObjectTurnaroundDetector methods `#838 <https://github.com/start-jsk/rtmros_common/issues/838>`_

  * [src/hrpsys_ros_bridge/samplerobot_hrpsys_config.py] Update latest st parameter for torque control
  * [euslisp/rtm-ros-robot-interface.l] Add set-ref-force time `#838 <https://github.com/start-jsk/rtmros_common/issues/838>`_
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add ObjectTurnaroundDetector methods
  * [test/hrpsys-samples/samplerobot-carry-object.l] Add carry object euslisp sample `#838 <https://github.com/start-jsk/rtmros_common/issues/838>`_

* SLAM support

  * [hrpsys_ros_bridge/src/hrpsys_ros_bridge/hrpsys_dashboard.py] Add MapButton for SLAM and odometry `#869 <https://github.com/start-jsk/rtmros_common/issues/869>`_
  * [src/HrpsysSeqStateROSBridge.cpp] Transform twist from local to global in pose covarance calculation because pose is in global `#830 <https://github.com/start-jsk/rtmros_common/issues/830>`_
  * [src/HrpsysSeqStateROSBridge.cpp] Set root_link as child_frame_id and describe twist in child_frame_id coordinate `#828 <https://github.com/start-jsk/rtmros_common/issues/828>`_
  * [src/HrpsysSeqStateROSBridge.cpp] Fix covariance calculation in HrpsysSeqStateROSBridge: previous verision calculation only can be applied in nonhoronomic robot `#812 <https://github.com/start-jsk/rtmros_common/issues/812>`_
  * [src/HrpsysSeqStateROSBridge.cpp] Fix coordinate representation: omega in twist is global
  * [src/HrpsysSeqStateROSBridge.cpp] Calculate covariance of odometry in HrpsysSeqStateROSBridge
  * [src/HrpsysSeqStateROSBridge.cpp] update prev_odom only when odom is successfully updated in dt > 0
  * [src/HrpsysSeqStateROSBridge.cpp] Calculate angular twist

* Special joint support `#837 <https://github.com/start-jsk/rtmros_common/issues/837>`_

  * [models/samplespecialjointrobot.yaml, catkin.cmake] Add cmake setting and euscollada yaml for special joint robot

* add ContactState.msg, ContactStateStamped.msg, ContactStatesStamped.msg `#834 <https://github.com/start-jsk/rtmros_common/issues/834>`_

  * [msg/ContactState.msg] Add frame_id to contact states messages
  * [launch/hrpsys_ros_bridge.launch, src/HrpsysSeqStateROSBridge{,cpp,.h,Impl.cpp,Impl.h}] separate ContactStates.msg into ContactState.msg, ContactStateStamped.msg and ContactStatesStamped.msg
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add euslisp interface to get contact states
  * [hrpsys_ros_bridge/src/HrpsysSeqStateROSBridge.cpp] add contact states publisher of AutoBalancer and Stabilizer
  * [msg/ContactState.msg] add ContactStates.msg for legged robots

* Sample codes

  * [src/hrpsys_ros_bridge/sample4legrobot_hrpsys_config.py, src/hrpsys_ros_bridge/sample4legrobot_hrpsys_config.py] set ThermoLimiter parameters to decrease debug messages `#849 <https://github.com/start-jsk/rtmros_common/issues/849>`_
  * [src/hrpsys_ros_bridge/sample4legrobot_hrpsys_config.py] update sample4legrobot st and abc parameters, set st and abc parameters `#847 <https://github.com/start-jsk/rtmros_common/issues/847>`_
  * [hrpsys_ros_bridge/src/hrpsys_ros_bridge/samplerobot_hrpsys_config.py] Update st vertices setting `#819 <https://github.com/start-jsk/rtmros_common/issues/819>`_
  * [src/hrpsys_ros_bridge/sample4legrobot_hrpsys_config.py,  models/sample4legrobot.yaml, euslisp/sample4legrobot-interface.l,  catkin.cmake] Add sample4legrobot generation if exists `#817 <https://github.com/start-jsk/rtmros_common/issues/817>`_
  * [euslisp/sample4legrobot-interface.l] Update path hrpsys_ros_bridge_tutorials -> hrpsys_ros_bridge `#817 <https://github.com/start-jsk/rtmros_common/issues/817>`_
  * [catkin.cmake] Add check for sample_4leg_robot existence `#817 <https://github.com/start-jsk/rtmros_common/issues/817>`_
  * [hrpsys_ros_bridge/src/hrpsys_ros_bridge/samplerobot_hrpsys_config.py] Update st damping parameters `#809 <https://github.com/start-jsk/rtmros_common/issues/809>`_
  * [hrpsys_ros_bridge/catkin.cmake, src/hrpsys_ros_bridge/samplerobot_hrpsys_config.py, test/hrpsys-samples/test_samplerobot_euslisp_unittests.launch]  Enable euslisp unittest for unstable rtcs `#806 <https://github.com/start-jsk/rtmros_common/issues/806>`_
  * [scripts/default*.launch.in,,cmake/compile_robot_model.cmake] Use arguments to specify whether unstable or not `#806 <https://github.com/start-jsk/rtmros_common/issues/806>`_
  * [hrpsys_ros_bridge/test/hrpsys-samples/samplerobot-unittest.l, samplerobot-auto-balancer.l] Update auto-balancer test functions and update unittest order `#806 <https://github.com/start-jsk/rtmros_common/issues/806>`_
  * [hrpsys_ros_bridge/test/hrpsys-samples/test_samplerobot_euslisp_unittests.launch] Add collision detector test `#805 <https://github.com/start-jsk/rtmros_common/issues/805>`_

* Move euslisp sample code from roseus_tutorials `#800 <https://github.com/start-jsk/rtmros_common/issues/800>`_

  * [hrpsys_ros_bridge/test/hrpsys-samples/test_samplerobot_euslisp_unittests.launch] Add rostest launch file for euslisp unittest and use .launch instead of .test.
  * [hrpsys_ros_bridge/test/hrpsys-samples, hrpsys_ros_bridge/euslisp/samplerobot-interface.l] Fix package name for sample robot euslisp files (hrpsys_ros_bridge_tutorials -> hrpsys_ros_bridge)

* Use emergency stopper by default for unstable rtc robots  `#760 <https://github.com/start-jsk/rtmros_common/issues/760>`_

  * [hrpsys_ros_bridge/cmake/compile_robot_model.cmake] Use emergency stopper by default for unstable rtc robots
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add method for emergency stop methods
  * [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Enable to set rosbridge name for idl srv methods.
  * [hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch] Add emergency stopper ROSBridges
  * [scripts/diagnostics.py] publish diagnostic of emergency_mode from diagnostics.py `#780 <https://github.com/start-jsk/rtmros_common/issues/780>`_
  * [src/HrpsysSeqStateROSBridge{,cpp,.h,Impl.cpp,Impl.h}] publish emergency_mode from EmergencyStopper rtc `#763 <https://github.com/start-jsk/rtmros_common/issues/763>`_

* Contributors: Eisoku Kuroiwa, Kei Okada, Masaki Murooka, Ryohei Ueda, Shunichi Nozawa, Tatsuya Ishikawa, Yohei Kakiuchi, Yuta Kojio, Iori Kumagai

1.2.14 (2015-06-23)
-------------------
* [hrpsys_ros_bridge] Add splash screen to hrpsys dashboard
* Enable to set step time for each foot steps and update documentation
* Add emergency walking stop and update documentations
* [hrpsys_ros_bridge] set position of imu_floor zero
* Add method to get remaining foot steps and displaying method
* [cmake_compile_robot_model.cmake] get_filename_component DIRECTORY is only available > cmake 2.8.12
* Publish cop position in end link frame
* Publish COP for each end effectors. COPInfo is provided by Stabilizer.
* Add --use-robot-hrpsys-config argument to compile robot old (added to compile robot in 54e64bf3c4131fc907c6b7c0a34d728f82948e76)
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa

1.2.13 (2015-06-11)
-------------------
* [launch/hrpsys_ros_bridge.launch] Add USE_ROBOT_POSE_EKF
* [euslisp/rtm-ros-robot-interface.l] fix typo
* [euslisp/rtm-ros-robot-interface.l] Add :adjust-foot-steps-roll-pitch to reset current foot steps
* [euslisp/rtm-ros-robot-interface.l] Add toe heel angle for :set-foot-steps-with-param-no-wait
* [euslisp/rtm-ros-robot-interface.l] Add argument for transition times
* [euslisp/rtm-ros-robot-interface.l] Use current foot midcoords of rleg and lleg in roll pitch method
* [euslisp/rtm-ros-robot-interface.l] Add adjust footsteps euslisp interface
* [euslisp/datalogger-log-parser.l] Add function to dump seq pattern file from datalogger log
* [euslisp/rtm-ros-robot-interface.l,uslisp/rtm-ros-robot-interface.l] Add force moment param zerosettin method and use it in calib program
* [euslisp/rtm-ros-robot-interface.l] raise error when bridged service call failed
* [euslisp/rtm-ros-robot-interface.l] Make interpolation-time in sync-controller shorter
* [euslisp/rtm-ros-robot-interface.l] Add :set-foot-steps-with-base-height
* [euslisp/rtm-ros-robot-interface.l] Fix typo in start method. Start auto balancer by legs.
* [euslisp/rtm-ros-robot-interface.l] Add euslisp methods for default unstable controllers
* [src/HrpsysSeqStateROSBridge.cpp] imu_floor again
* [src/hrpsys_ros_bridge/hrpsys_dashboard.py] add st/abc/ic start at once button
* [hrpsys_ros_bridge/cmake/compile_robot_model.cmake] Enable to specify robot specific hrpsys_config.py
* [launch/hrpsys_ros_bridge.launch] Add argument to specify periodic rate of seq ros bridge
* [euslisp/rtm-ros-robot-interface.l] add hours, minutes and seconds for :save-log
* [.travis.yaml, hrpsys_ros_bridge/catkin.cmake] add eusdoc
* [euslisp/rtm-ros-robot-interface.l] Support t or nil in setter arguments
* Contributors: Eisoku Kuroiwa, Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Iori Kumagai

1.2.12 (2015-04-27)
-------------------
* [euslisp/rtm-ros-robot-interface.l] Update function to calculate toe heel offset. Support both pos and zmp and add documentation
* [hrpsys_ros_bridge/cmake/compile_robot_model.cmake] fix for --conf option with comment
* Contributors: Kei Okada, Shunichi Nozawa

1.2.11 (2015-04-25)
-------------------

* [catkin.cmake] add depend to pr2_controllers_msgs in catpkin_package
* [cmake/compile_robot_model.cmake] fix dependency add_custom_depend need to DEPEND to other target
* [euslisp/rtm-ros-robot-interface.l] Add calculate-toe-heel-pos-offsets and set-foot-steps-with-param
* Contributors: Kei Okada, Shunichi Nozawa

1.2.10 (2015-04-24)
-------------------
* add rewrited version of compile_robot_model.cmake

  * [compile_robot_model.cmake] generate controller_config even if yaml is not found
  * [compile_robot_model.cmake] use add_custom_target/command for eusif and launch, set PROJECT_PKG_NAME
  * [compile_robot_model.cmake] rewrite everything from scratch

* euslisp

  * [rtm-ros-robot-interface.l] Add method to align footsteps    with roll or pitch angle
  * [datalogger-log-parser.l] change max-line count method
  * [rtm-ros-robot-interface.l] Add sync-controller method, which preserve limb-controller angle before remove-joint-group is called.
  * [rtm-ros-robot-interface.l] Enable to set gravitational acceleration for calculating st parameter

* [HrpsysSeqStateROSBridgeImpl.{cpp,h}] display more debug info for diagnostics
* [cmake/compile_robot_model.cmake] Revert "compile_robot contains output files, not targets"
* [hrpsys_ros_bridge/package.xml] Limits dependent pkg version to avoid critical error in downstream (tork-a/rtmros_nextage/#160)
* Contributors: Eisoku Kuroiwa, Isaac IY Saito, Kei Okada, Shunichi Nozawa, Iori Kumagai

1.2.9 (2015-04-11)
------------------
* [package.xml] remove pr2_controllers, add pr2_controller_msgs, pr2_msgs, control_msgs package.xml

* [collision_state] fix to work collision state

  * [collision_state.py] fix minor bug of collision_state.py CORBA.OBJECT_NOT_EXIST -> omniORB.CORBA.OBJECT_NOT_EXIST
  * [collision_state.py] check isActive() to avoid raise error during servo on phase

* [hrpsys_ros_bridge/catkin.cmake]

  * move rtmlaunch/rtmtest from hrpsys_ros_bridge to openrtm_tools, add envhook for catkin users

* [hrpsys_tools/hrpsys.launch] support DEBUG_HRPSYS argument to run rtcd with gdb

* [hrpsys_ros_bridge/cmake/compile_robot_model.cmake]

  * Add dependency from the files generated by  export_collada to compile_robots in order to prevent parallel execution  of export_collada
  * Check current if scope has parent scope when set  variable in PARENT_SCOPE in compile_robot_model.cmake to supress warning messages
  * Fix serious typo: daefile -> _daefile in compile_robot_model.cmake
  * fix warning in if/endif macro
  * compile_robtos should be list of all generated lisp file, not targets

* [euslisp/rtm-ros-robotics-interface.l]

  * Add impedance controller mode getter method
  * Return Euslisp coordinates from :get-foot-step-param
  * Add methods to get Euslisp style parameters from IDL enum type
  * Add comments for st methods
  * Add methods to tune st parameter
  * Add functions to calculate eefm st parameters
  * Fix order of ee setting
  * Pass arguments for joint-properties to ProjectGenerator

* Contributors: JAXON, Kei Okada, Ryohei Ueda, Shunichi Nozawa

1.2.8 (2015-03-09)
------------------
* [hrpsys_ros_bridge] Do not run collada_to_urdf parallel. In order to
  avoid parallel execution of collada_to_urdf, add tricky dependency
* Add graspless manip mode euslisp interface
* [test-*.py] test name could not contain minus? any changet to underscore (http://answers.ros.org/question/197377/rostest-not-finding-the-actual-test/)
* [test-samplerobot.py] fix syntax error on loadPattern
* update for legged robot
* Sample.pos is not installed via deb package, see https://github.com/fkanehiro/openhrp3/issues/46
* Fix rmfo sensor argument
* copy rtmlaunch,rtmtest to global_bin when compile
* Add method to calculate go-velocity param from velocity center offset
* Add menus for unstable rtcs (not used by default)
* Add setting for HrpsysConfigurator in hrpsys dashboard and apply it to servoOn/Off menu (disabled by default).
* Remove unused initial leg offsetting because this is implemented AutoBalancer's setFootSteps
* [hrpsys_ros_bridge] Refactor compile_robot_model.cmake
* Use ee name for impedance methods
* [hrpsys_ros_bridge] collision_state.py:  need to wait for activate
* [hrpsys_ros_bridge] sensor_ros_bridge_connect.py: wait for sh, sometimes we can not find them
* Update impedance start/stop methods
* [hrpsys_ros_bridge] Do not call export_collada in parallel
* Update :reset-force-moment-offset funcs and add documents
* Update documentation strings for ImpedanceController and Ref forces
* [hrpsys_ros_bridge] Fix path for catkin build
* pass :rarm instead of 'rhsensor' or 'rasensor' to :set-forcemoment-offset-param
* Add seq base pos and rpy methods
* Remove duplicated method and fix argument passing for imp methods
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Eisoku Kuroiwa

1.2.7 (2015-01-06)
------------------
* Add :reference-root-coords method for log
* Update datalogger parser time parsing and robothardware naming
* Add euslisp methods for seq wait interpolation and co get status
* Enable virtual force setting. Read conf file in HrpsysSeqStateROSBridge and connect vfs ports if exists
* Add set ref force and moment methods. Enable vforcesensors
* change robot-init function to pass arguments to initializer
* Set default name as co
* Use segment name of body_info as collision detection frame_id
* Add argument to select collision_state.py activation because collision_state.py can be launched in hrpsys_ros_bridge.launch
* Enable to configure collision detector instance name
* Use subscription_type as new to avoid influence on realtime process
* (rtm-ros-robot-interface) : Add optionaldata for dump-seq-pattern-file
* (rtm-ros-robot-interface) : Add function to load seq pattern file and euslisp list from it.
* (rtm-ros-robot-interface) : Add documentation string for dump-seq-pattern-file
* Fix frame of loadPattern
* Add euslisp interface method for reference wrench
* Publish reference wrench from StateHolder wrench data ports. Check existence of wrench ports because hrpsys-base 315.1.9 does not have wrench ports in Seq and Sh.
* Add RMFO calib readme
* Update RMFO calibration sequence. Use parameter file
* Add load and dump rmfo param methods
* Separate robot date string as method and fix :save-log default argument
* on indigo, hrpsys is not ros-catkin package
* fix for new packaging openrtm/hrpsys (compile package directoly from original repository)
* Fix typo of hrpsys_ros_bridge.launch
* Add USE_HRPSYS_PROFILE to toggle whether running hrpsys_profile
* Add parameter ~publish_sensor_tf to HrpsysSeqStateROSBridge to toggle
  publishing tf frames of the sensors
* Fix unitsystem for documentation
* Add directory to generate Euslisp documentation
* Enable to set integer enum values
* (rtm-ros-robot-interface) : Fix typo in save-log
* (rtm-ros-robot-interface) : Add date string and robot name to Datalogger log name by default.
* (rtm-ros-robot-interface) : Update seq methods. Revise set-interpolation-mode using defconst param and remove deprecated dump-seq parttern
* Merge remote-tracking branch 'origin/master' into tmp_update_impedance_controller_methods
* (rtm-ros-robot-interface) : Update impedance controller setter and getter
* (rtm-ros-robot-interface) : Remove deprecated euslisp interface and functions.
* Update rtmbuild_init to specify dependency of generate_messages and
  add new service interface to HrpsysSeqStateROSBridge to specify transformation
  for each sensor
* Merge pull request #583 from eisoku9618/update-print-end-effector-parameter-conf-from-robot
  Merged.
* do not print a leading colon of :rarm
* (rtm-ros-robot-interface) : Enable to set st algorithm as Euslisp symbol.
* remove unnecessary white spaces
* rename total time of 1 cycle from RobotHardware0 to Summary
* add eps_of_rh variable in order to reduce unnecessary for loop
* (test-samplerobot) : Import all srv files in test-samplerobot.py discussed in https://github.com/start-jsk/rtmros_common/issues/558
* (default_robot*.launch.in) : Add argument for precreate controller
* (rtm-ros-robot-interface) : Update :set-gait-generator-param method to use defconstant enum values and add printing method
* (datalogger-log-parser) : Add abc baseTform reading
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Eisoku Kuroiwa

1.2.6 (2014-10-10)
------------------
* set time-limit to 300
* (test-samplerobot.py) fix test case, since /clock is sync with hrpsys time, so we can use more strict settings
* fix test code for changing 0.002
* (rtm-ros-robot-interface) : Add documentation strings for state methods and rearrange it. Add logger documentation.
* Contributors: Kei Okada, Shunichi Nozawa

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
