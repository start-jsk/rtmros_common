# hrpsys\_ros\_bridge
## Introduction
hrpsys\_ros\_bridge is a package to connect `hrpsys <http://wiki.ros.org/hrpsys>`_ (OpenRTM-based controller) system and ROS system.

This package contains ROSBridge to `hrpsys`, cmake_libraries, launch files, Euslisp interface, and examples.

### ROSBridge
  ROSBridge here is a ROS node to bridge `hrpsys` system and ROS system.

    You might want to check other "bridging" software in ROS too such as:
    
    - `rosbridge_suite <http://wiki.ros.org/rosbridge_suite>`_ provides a JSON API to ROS functionality for non-ROS programs.
    - `openrtm_ros_bridge <http://wiki.ros.org/openrtm_ros_bridge?distro=hydro>`_ Provides a ROS node between `OpenRTM`.

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

   By default, ROS Bridge are generated in hrpsys\_ros\_bridge make.

   These bridges ROS service to OpenRTM service port.

   Based on ``hrpsys_ros_bridge/idl/*.idl``, generated codes are in ``hrpsys_ros_bridge/src_gen/*.cpp`` and built binaries are in ``hrpsys_ros_bridge/[bin,lib]``. 
   
### cmake libraries
   [``hrpsys_ros_bridge/cmake/compile_robot_model.cmake``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/cmake/compile_robot_model.cmake) is a CMake file to convert robot model file format.

### launch files
   [``hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch``] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/launch/hrpsys_ros_bridge.launch) is a launch file to invoke ROSBridges and connect them with RTCs.

### Euslisp interface
   [Euslisp Interface] (https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_ros_bridge/euslisp/README.md)

### Examples
   ``hrpsys_ros_bridge/test/test-samplerobot.test``
