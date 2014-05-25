# Robot Interface
  By loading [``hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l) from Euslisp program, 
  you can use ROSBridges. 

  Currently example codes are included in [start-jsk/rtmros_tutorials/hrpsys_ros_bridge_tutorials] (https://github.com/start-jsk/rtmros_tutorials/tree/master/hrpsys_ros_bridge_tutorials)
  
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
