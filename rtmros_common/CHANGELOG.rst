^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtmros_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2017-12-22)
------------------

1.4.1 (2017-10-25)
------------------

1.4.0 (2017-08-09)
------------------

1.3.3 (2017-02-12)
------------------

1.3.2 (2016-04-26)
------------------

* [fix][openrtm_ros_bridge] Fix genjava problem build_depend message_generation
* [fix][hrpsys_ros_bridge] HrpsysSeqStateROSBridgeImpl.cpp: sensor->localR is world coords
* [fix][rtm-ros-robot-interface.l] add time constant prameter according to upstream idl update
* [fix] getFootStepParam interface according to upstream change. Remove rleg_coords and lleg_coords.
* [feat] [hrpsys_ros_bridge/test/test-samplerobot.py] add test programs to check frame_id of off_xxsensor and ref_xxsensor (#940)
* [feat] Set /robot/type param according to lower-case robot name such as samplerobot.
* [feat] [hrpsys_ros_bridge/test/test-samplerobot.*] add a test program for init of hcf
* [feat] fix collision visualization, color of sphere will be purple if collision occur
* [feat] .travis.yml : add test when old hrpsys-ros-bridge exists (#929)
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add check for param existence for def-set-get-param-method to neglect idl mismatch error. Fix indent. (#933)
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add reference force updater euslisp methods.
* [feat] [hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l] Add object turnaround detector moment version interface.l
* [feat] Add more CORBA exception information. Add BAD_PARAM, MARSHAL, and OBJECT_NOT_EXIST checking. Add minor code printing and possible case assumption.
* [feat][openrtm_tools] Enable to set push_policy and push_rate for DataPorts from hrpsys_ros_bridge.launch argument. Use all and 50.0 by default to keep backward compatibility.
* [feat][openrtm_tools] Enable to set subscription_type for DataPorts from hrpsys_ros_bridge.launch argument. Use new by default to keep backward compatibility.
* [feat][hrpsys_tools] add ReferenceForceUpdater
* [feat] enable virtual force in HrpsysSeqStateROSBridgeImpl
* [feat] Enable to set subscription_type for DataPorts from hrpsys_ros_bridge.launch argument. Use new by default to keep backward compatibility.
* [feat] Add loading of optionalData from seq pattern file.
* [feat] add time constant prameter according to upstream idl update
* [feat] Add HrpsysSeqStateROSBridge tf rate test
* [feat] Add define controller `#887 <https://github.com/start-jsk/rtmros_common/issues/887>`_
* [feat] Add leg limb controller setting.
* [feat] Add method to define joint trajectory controller by default setting.
* [feat] Add gopos overwrite and graspless manip mode to all demos
* [feat] Add example for graspless manip mode and gopos overwrite.
* [feat] Add euslisp interface for graspless manip mode.
* [feat] add a set-default-step-time-with-the-same-swing-time method
* [doc] default documentation string for def-set-get-param-method setter and getter.
* [doc] In-code documentation improvement

* Contributors: Eisoku Kuroiwa, Kei Okada, Kentaro Wada, Masaki Murooka, Shunichi Nozawa, Koyama Ryo, Yohei Kakiuchi, Yu Ohara, Iori Kuroiwa, Isaac I.Y. Saito


1.3.1 (2015-12-16)
------------------
* deb release only targeting to indigo

1.3.0 (2015-12-09)
------------------
* deb release only targeting to indigo

1.2.14 (2015-06-23)
-------------------

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

1.2.8 (2015-03-09)
------------------

1.2.7 (2015-01-06)
------------------

1.2.6 (2014-10-10)
------------------

1.2.5 (2014-10-04)
------------------
* Improvement on rtm-ros-robot-interface, datalogger-log-parser, samplerobot
* Fix stamp of odom and imu
* Improve dependency: robot_pose_ekf.launch, robot_pose_ekf
* Add depend on std_msgs
* Contributors: Kei Okada, Shunichi Nozawa, Yohei Kakiuchi

1.2.4 (2014-09-08)
------------------

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
* (hrpsys.launch) : Add config_file setting for HGcontroller
* Contributors: Kei Okada, Kunio Kojima, Shunichi Nozawa, Masaki Murooka, Isaac IY Saito

1.2.1 (2014-06-23)
------------------
* (package.xml files) Update repository URLs
* Contributors: Isaac Isao Saito

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
* (See each contained package for the changelog)

1.0.7 (2014-03-20)
------------------

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
* add hrpsys_ros_bridge to run build
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Add a ROS metapackage rtmros_common.
* Contributors: gm130s@gmail.com, kei.okada
