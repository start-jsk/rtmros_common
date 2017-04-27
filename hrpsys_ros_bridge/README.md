# hrpsys\_ros\_bridge
## Introduction
hrpsys\_ros\_bridge is a package to connect `hrpsys <http://wiki.ros.org/hrpsys>`_ (OpenRTM-based controller) system and ROS system.

This package contains ROSBridge to `hrpsys`, cmake_libraries, launch files, Euslisp interface, and examples.

### ROSBridge
  ROSBridge here is a ROS node to bridge `hrpsys` system and ROS system.

    You might want to check other "bridging" software in ROS too such as:
    
    - `rosbridge_suite <http://wiki.ros.org/rosbridge_suite>`_ provides a JSON API to ROS functionality for non-ROS programs.
    - `rtmbuild <http://wiki.ros.org/rtmbuild?distro=hydro>`_ Provides a ROS-like compile and launch script for `OpenRTM`.

  You can use `OpenRTM` resources from ROS interface, e.g., 
  use OpenRTM ``Data Ports`` through ROS ``Topic`` and OpenRTM ``Servie Ports`` through ROS ``Service``. The following table shows the corresponding concepts for the 2 worlds.

| \ | ROS | OpenRTM |
| --- | --- | --- |
| Process | ROS Node | RTC |
| Data connection | Topic | Data Port |
| RPC connection | Service | Service Port |

- HrpsysSeqStateROSBridge :  
  Publish ROS topics by reading RTC's data ports. 

  Send ROS JointTrajectory service to ``SequencePlayer.rtc``  by its service ports.

2. Generated ROSBridges:

   `openrtm_ros_bridge` is the ROS bridge for example code in `openrtm_aist` and `hrpsys_ros_bridge` provides ROS bridge for `hrpsys` package.

   These bridges ROS service to OpenRTM service port.

   For example all idl files in `hrpsys` is coped to ``hrpsys_ros_bridge/idl/*.idl``, and it generates cpp codes in ``hrpsys_ros_bridge/src_gen/*.cpp`` and built binaries in ``hrpsys_ros_bridge/[bin,lib]``. 
   
### cmake libraries
   [``hrpsys_ros_bridge/cmake/compile_robot_model.cmake``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/cmake/compile_robot_model.cmake) is a CMake file to convert robot model file format.

### launch files
   [``hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch) is a launch file to invoke ROSBridges and connect them with RTCs.

### Euslisp interface
   [Euslisp Interface] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/README.md)

### Examples
   ``hrpsys_ros_bridge/test/test-samplerobot.test``

### Euslisp Examples
[Euslisp example](https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/test/hrpsys-samples/README.md)
