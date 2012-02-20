fmk_ros_bridge ROS Launch Files
===============================

**Description:** fmk

  
  
       fmk
  
    

**License:** BSD

start_fmk.launch
----------------

.. code-block:: bash

  roslaunch fmk_ros_bridge start_fmk.launch

start_fmk_ros.launch
--------------------

.. code-block:: bash

  roslaunch fmk_ros_bridge start_fmk_ros.launch


  <initial setting>
  1)We assumed that rtmros_common and eclipse is already build up.
  2)Put local PC's IP address as 192.168.128.1 .
  3)Unzip VehicleUnitRTC.zip, and move RTC directory under /(root).
  4)Sudo apt-get install ftp server.
  $ sudo apt-get install wu-ftpd
  $ sudo useradd fmk
  $ sudo passwd fmk
  Edit /etc/passwd like below.
  Example)fmk:x:1001:1001::/:/bin/sh

  <normal startup>
  1)Connect local PC and fmk with LAN.
  2)Switch on fmk.
  3)Wait about 30 seconds.
  ($ sudo tail -f /var/log/wu-ftpd/xferlog ;;Check until Mobility.out,Interface.out,VehicleServiceProviderComp.outare send to fmk.)
  4) <new terminal> $ rosrun fmk run-vehicle-gui.sh
  5) <new terminal> $ rosrun openhrp3 eclipse.sh
  Connect provider and consumer, and activate them.
  6) Push GUI's PowerOn,ServoOn button.
  7) <new terminal> $ rosrun fmk start_fmk_ros.launch
  activate rosbridge on eclipse.
  

Contents
########

.. code-block:: xml

  <launch>
    <arg default="192.168.128.14" name="nameserver" />
    <env name="RTCTREE_NAMESERVERS" value="$(arg nameserver)" />
    <node name="rtm_naming" output="screen" pkg="openrtm" type="rtm-naming-restart" />
    <anode name="starteclipse" output="screen" pkg="openhrp3" type="eclipse.sh" />
    <node args="-f $(find fmk)/rtc.conf" name="fmkrosbridge" output="screen" pkg="fmk" type="FmkRobotROSBridgeComp">
      <rosparam>
        max_vx: 0.3
        max_vw: 0.5
        max_ax: 0.3
        max_aw: 0.5
      </rosparam>
    </node>
    <node name="ftp_log_1" output="screen" pkg="fmk" type="ftp-log-1.sh" />
    <node name="ftp_log_2" output="screen" pkg="fmk" type="ftp-log-2.sh" />
    <node args="$(find fmk)/launch/start_fmk_ros.launch" name="rtmlaunch" output="screen" pkg="openrtm" type="rtmlaunch.py" />
    <rtconnect from="FmkRobotROSBridge0.rtc:VehicleService" to="VehicleServiceProvider0.rtc:VehicleService" />
    <rtactivate component="VehicleServiceProvider0.rtc" />
    <rtactivate component="FmkRobotROSBridge0.rtc" />
  
    <machine address="localhost" default="true" name="localhost" ros-package-path="$(env ROS_PACKAGE_PATH)" ros-root="$(env ROS_ROOT)" user="root" />
    <node machine="localhost" name="ps3joy" output="screen" pkg="ps3joy" type="ps3joy.py" />
    <node name="joy_node" output="screen" pkg="joy" type="joy_node" />
    <node name="joy_teleop" output="screen" pkg="mrobot_ros_bridge" type="joy_teleop.py" />
  </launch>

start_move_base.launch
----------------------

.. code-block:: bash

  roslaunch fmk_ros_bridge start_move_base.launch


  8) <new terminal> $ rosrun fmk start_move_base.launch
  Map_server, hokuyo_node (for laser) starts, and begin navigation.
  

Contents
########

.. code-block:: xml

  <launch>
    <node name="fmk_hokuyo" output="screen" pkg="hokuyo_node" respawn="false" type="hokuyo_node">
      <param name="calibrate_time" type="bool" value="false" />
      <param name="port" type="string" value="/dev/ttyACM0" />
      <param name="intensity" type="bool" value="false" />
      <param name="frame_id" type="string" value="fmk_laser_link" />
      
      
  
    </node>
    <node args="0.26 0.0 0.205 0 0 -3.14159 /base_footprint /fmk_laser_link 100" name="base_to_laser" pkg="tf" type="static_transform_publisher" />
  
    
    <node args="$(find jsk_maps)/raw_maps/eng2-7f-0.05.yaml" name="map_server" pkg="map_server" type="map_server">
  	
      <param name="frame_id" value="/map" />
    </node>
  
    
    <include file="$(find fmk)/launch/amcl_omni.xml" />
    
    <node name="move_base" output="screen" pkg="move_base" respawn="false" type="move_base">
      <rosparam command="load" file="$(find fmk)/launch/yaml/costmap_common_params.yaml" ns="global_costmap" />
      <rosparam command="load" file="$(find fmk)/launch/yaml/costmap_common_params.yaml" ns="local_costmap" />
      <rosparam command="load" file="$(find fmk)/launch/yaml/local_costmap_params.yaml" />
      <rosparam command="load" file="$(find fmk)/launch/yaml/global_costmap_params.yaml" />
      <rosparam command="load" file="$(find fmk)/launch/yaml/base_local_planner_params.yaml" />
    </node>
  </launch>

