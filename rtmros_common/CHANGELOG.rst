^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtmros_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
