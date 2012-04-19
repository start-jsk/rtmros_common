openhrp3 ROS Launch Files
=========================

**Description:** openhrp3

  
  
     OpenHRP3 : Robot software platform
  
  http://www.openrtp.jp/openhrp3/en/index.html
  
  http://code.google.com/p/openhrp-aist-grx/
  
    

**License:** EPL

falling-box.launch
------------------

.. code-block:: bash

  roslaunch openhrp3 falling-box.launch


This scripts starts FallingBoxes.xml project file, select StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/FallingBoxes
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/FallingBoxes.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=30" cwd="node" pkg="openhrp3" test-name="FallingBoxes" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/FallingBoxes.ogv" pkg="jsk_tools" test-name="z_FallingBoxes" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

joystick_simulator.launch
-------------------------

.. code-block:: bash

  roslaunch openhrp3 joystick_simulator.launch

pa10.launch
-----------

.. code-block:: bash

  roslaunch openhrp3 pa10.launch


This scripts starts PA10Sample.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/PA10Sample
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/PA10Sample.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=100" cwd="node" pkg="openhrp3" test-name="PA10Sample" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/PA10Sample.ogv" pkg="jsk_tools" test-name="z_PA10Sample" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

sample-rangesensor.launch
-------------------------

.. code-block:: bash

  roslaunch openhrp3 sample-rangesensor.launch


This scripts starts SampleSV_RangeSensor.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/SampleSV_RangeSensor
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/SampleSV_RangeSensor.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=100" cwd="node" pkg="openhrp3" test-name="SampleSV_RangeSensor" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/SampleSV_RangeSensor.ogv" pkg="jsk_tools" test-name="z_SampleSV_RangeSensor" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

sample-vehicle.launch
---------------------

.. code-block:: bash

  roslaunch openhrp3 sample-vehicle.launch


This scripts starts SampleSV.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/SampleSV
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/SampleSV.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=100" cwd="node" pkg="openhrp3" test-name="SampleSV" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/SampleSV.ogv" pkg="jsk_tools" test-name="z_SampleSV" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

sample.launch
-------------

.. code-block:: bash

  roslaunch openhrp3 sample.launch


This scripts starts Sample.xml project file, select StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/Sample
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/Sample.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=10" cwd="node" pkg="openhrp3" test-name="Sample" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/Sample.ogv" pkg="jsk_tools" test-name="z_Sample" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

samplerobot-inhouse.launch
--------------------------

.. code-block:: bash

  roslaunch openhrp3 samplerobot-inhouse.launch


This scripts starts SampleLF.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/SampleRobot_inHouse
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=250" cwd="node" pkg="openhrp3" test-name="SampleRobot_inHouse" time-limit="500" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/SampleRobot_inHouse.ogv" pkg="jsk_tools" test-name="z_SampleRobot_inHouse" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

samplerobot-pickupbox.launch
----------------------------

.. code-block:: bash

  roslaunch openhrp3 samplerobot-pickupbox.launch


This scripts starts SampleLF.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/SampleLF
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/SampleLF.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=100" cwd="node" pkg="openhrp3" test-name="SampleLF" time-limit="300" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/SampleLF.ogv" pkg="jsk_tools" test-name="z_SampleLF" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

samplerobot-walk.launch
-----------------------

.. code-block:: bash

  roslaunch openhrp3 samplerobot-walk.launch



This scripts starts SamplePD.xml OpenHRP3 sample project file, the robot picks up the yellow box from the ground by selecting StartSimulation sub-menu from GrxUI menu, or clock simulation start button.

.. video:: build/images/SamplePD
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="LANG" value="C" />
    <arg name="MODEL_FILE" value="$(find openhrp3)/share/OpenHRP-3.1/sample/project/SamplePD.xml" />
  
    <node name="rtm_naming" pkg="openrtm" type="rtm-naming-restart" />
    <node name="hrpsys_kill_all" pkg="openhrp3" type="openhrp-shutdown-servers" />
    <node args="$(arg MODEL_FILE)" name="grxui" pkg="openhrp3" type="grxui.sh" />
  
    <test args="--start-simulation --target-directory=$(find openhrp3)/build/images --max-time=134" cwd="node" pkg="openhrp3" test-name="SamplePD" time-limit="400" type="test-grxui.py" />
  
    <test args="$(find openhrp3)/build/images/SamplePD.ogv" pkg="jsk_tools" test-name="z_SamplePD" time-limit="1000" type="ogv_encode.sh" />
  
  </launch>

