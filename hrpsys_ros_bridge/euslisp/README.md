# Robot Interface
  By loading [``hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l) from Euslisp program, 
  you can use ROSBridges. 

  Currently example codes are included in [start-jsk/rtmros_tutorials/hrpsys_ros_bridge_tutorials] (https://github.com/start-jsk/rtmros_tutorials/tree/master/hrpsys_ros_bridge_tutorials)

  Euslisp interface documentation is [here] (http://euslisp-docs.readthedocs.org/en/latest/hrpsys_ros_bridge/rtm-ros-robot-interface)
  
# Utilities
## datalogger-log-parser.l
  [``hrpsys_ros_bridge/euslisp/datalogger-log-parser.l``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/datalogger-log-parser.l)
  is Euslisp program to read DataLogger.rtc's log files. 

1. Store log files and put them on your PC 
For example:
```
$ ls /path/to/your/own/logfile
  LogfileBasename.sh_qOut
  LogfileBasename.sh_tqOut
  ...
```
2. Use from Euslisp
```
$ rosrun roseus roseus
  ;; load Robot Interface file for your own robots
$ (load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
  ;; load datalogger-log-parser
$ (load "package://hrpsys_ros_bridge/euslisp/datalogger-log-parser.l")
  ;; create *log* to specify log full path and your own robot
$ (setq *log* (instance datalogger-log-parser-controller :init "/path/to/your/own/logfile/LogfileBasename" :robot samplerobot-robot))
  ;; Use. 
  ;;   Reading states are similar to *ri*'s :state method
$ (send *log* :state) ;; read 1 line from log files
$ (send *log* :reference-vector)
$ (send *log* :potentio-vector)
$ (send *log* :force-vector)
          ...
```

## calib-force-sensor-params.l
  [``hrpsys_ros_bridge/euslisp/calib-force-sensor-params.l``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/calib-force-sensor-params.l)
  is Euslisp program to calibrate RemoveForceSensorOffset RTC parameters. 
  This program rotates end-links and gathers force sensor values. 
  Then this program identifies parameters. 

1. Make calibration pose  
  Make calibration poses, which should include different force sensor orientations.  
  ``(make-default-ForceCalibPoses *robot*)`` and ``(make-default-ForceCalibPosesLegs *robot*)`` functions are example.  
  **Please be careful of robot's self collision.**

2. Execute calibration pose, gather sensor values, and identify parameters  
  Please execute ``(Forcecalib-For-Limbs *robot*)``.  
  Then the real robot rotates end-links and gathers force sensor values.  
  After all poses are executed, this program identifies parameters.  

  If ``(Forcecalib-For-Limbs *robot* :fname "/tmp/test-calib")`` are executed, 
  this program writes calibration results into "/tmp/test-calib-[robot name]-[date]".

  If ``(Forcecalib-For-Limbs *robot* :limbs '(:rleg :lleg) :poses (make-default-ForceCalibPosesLegs *robot*))`` are executed, 
  you can calibrate legs force sensors.
  
  This file can be read from loadForceMomentSensorOffset function, i.e., ``(send *ri* :load-forcemoment-offset-params "xxx")``.

## Dump openhrp project file based on Euslisp world(rtm-ros-robot-interface.l)

1. Initialize euslisp models
    ```
    (load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")
    (setq *robot* (samplerobot))
    (setq *box* (let ((obj (instance cascaded-link :init)) (lk (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 200 800 200)))))
                  (send obj :assoc lk)
                  (setq (obj . links) (list lk))
                  (send obj :init-ending)
                  obj))
    (send *robot* :fix-leg-to-coords (make-coords :pos #f(200 300 0) :rpy (list (deg2rad 45) 0 0)))
    (send *box* :newcoords (make-coords :pos #f(450 550 0) :rpy (list (deg2rad 45) 0 0)))
    (objects (list *robot* *box*))
    ```

2. Generate openhrp3 project file
    ```
    ;; For Torque Mode
    (dump-project-file-by-cloning-euslisp-models
     *robot*
     (ros::resolve-ros-path (format nil "package://openhrp3/share/OpenHRP-3.1/sample/model/sample1_bush.wrl"))
     :object-models nil
     :object-models-file-path nil
     :timestep 0.001 :dt 0.002 :use-highgain-mode nil :method :runge-kutta
     :output-fname (format nil "/tmp/~A_ResetPose_TorqueMode" (send *robot* :name)) :debug t)
    ;; For Torque Mode + Object
    (dump-project-file-by-cloning-euslisp-models
     *robot*
     (ros::resolve-ros-path (format nil "package://openhrp3/share/OpenHRP-3.1/sample/model/sample1_bush.wrl"))
     :object-models (list *box*)
     :object-models-file-path (list (ros::resolve-ros-path (format nil "package://openhrp3/share/OpenHRP-3.1/sample/model/box.wrl")))
     :timestep 0.001 :dt 0.002 :use-highgain-mode nil :method :runge-kutta
     :output-fname (format nil "/tmp/~A_ResetPose_TorqueMode_Box" (send *robot* :name)) :debug t)
    ;; For HighGain Mode
    (dump-project-file-by-cloning-euslisp-models
     *robot*
     (ros::resolve-ros-path (format nil "package://openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl"))
     :object-models nil
     :object-models-file-path nil
     :timestep 0.001 :dt 0.002 :use-highgain-mode t :method :runge-kutta
     :output-fname (format nil "/tmp/~A_ResetPose_HighGainMode" (send *robot* :name)) :debug t)
    ```

3. Launch hrpsys-simulator
    ```
    # For Torque Mode
    export BUSH_CUSTOMIZER_CONFIG_PATH=`rospack find openhrp3`/share/OpenHRP-3.1/customizer/sample1_bush_customizer_param.conf
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=/tmp/SampleRobot_ResetPose_TorqueMode.xml RUN_RVIZ:=false
    # For Torque Mode + Object
    export BUSH_CUSTOMIZER_CONFIG_PATH=`rospack find openhrp3`/share/OpenHRP-3.1/customizer/sample1_bush_customizer_param.conf
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=/tmp/SampleRobot_ResetPose_TorqueMode_Box.xml RUN_RVIZ:=false
    # For HighGain Mode
    rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=/tmp/SampleRobot_ResetPose_HighGainMode.xml RUN_RVIZ:=false
    ```