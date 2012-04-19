hrpsys_ros_bridge ROS Launch Files
==================================

**Description:** hrpsys and ROS bridge software, launch script and examples

  
  
  Summary: If you use real robot, start [robot]_ros_bridge.launch. For simulation environment, start [robot].launch. This script usually contains test code with the sample client program.
  
  [robot]_ros_bridge.launch : This file launches ros bridge software for specifi robot type. When you use real robot, launch this script after the robot started up (usually the script is named [robot]_startup.launch under the robot specific package), when you use the simulation, you do not have to call this sript, but use the [robot].launch file.
  
  [robot].launch : This file start the simulated robot in OpenHRP3 and the ROS bridge software. This script contains test tag to start sammple (EusLisp) client program
  
  [robot]_nosim.launch : This file is non dynamics version of [robot].launch
  
  Following scripts are only for internal usage:
  
  [robot]_startup.launch : This file start simulation robot in OpenHRP3 software.
  
  [robot]_startup_nosim.launch : This file start simulated robot in OpenHRP3 software with out forward dynamics calculation.
  
  hrpsys_ros_bridge.launch : hrpsys and ROS bridge software core launch script, this script is usually included from other script.
  
  
  
    

**License:** BSD

darwin.launch
-------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge darwin.launch



.. code-block:: bash

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/darwin-pickup.l

This launch file shows an example of ros bridge for darwin robot and sample client program `darwin-pickup.l`

.. video:: build/images/Darwin_PickUp
  :width: 600

.. video:: build/images/darwin_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/darwin_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/darwin_ros_bridge.launch" />
  
    <test args="--max-time=30   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/DBODY_LINK /DHEAD_LINK'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/darwin-pickup.l __log\:=dummy'" pkg="openhrp3" test-name="Darwin_PickUp" time-limit="600" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/Darwin_PickUp.ogv" pkg="jsk_tools" test-name="z_Darwin_PickUp" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/darwin_rviz.glc" pkg="jsk_tools" test-name="z_darwin_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

darwin_ros_bridge.launch
------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge darwin_ros_bridge.launch

darwin_startup.launch
---------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge darwin_startup.launch

gr001.launch
------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge gr001.launch



.. code-block:: bash

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/darwin-pickup.l

This launch file shows an example of ros bridge for darwin robot and sample client program `darwin-pickup.l`

.. video:: build/images/GR001_Motion
  :width: 600

.. video:: build/images/gr001_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/gr001_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/gr001_ros_bridge.launch" />
  
    <test args="--max-time=30   --target-directory=$(find hrpsys_ros_bridge)/build/images   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --check-tf='/WAIST_LINK /NECK_Y_LINK'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/gr001-motion.l __log\:=dummy'" pkg="openhrp3" test-name="GR001_Motion" time-limit="500" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/GR001_Motion.ogv" pkg="jsk_tools" test-name="z_GR001_Motion" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/gr001_rviz.glc" pkg="jsk_tools" test-name="z_gr001_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

gr001_ros_bridge.launch
-----------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge gr001_ros_bridge.launch

gr001_startup.launch
--------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge gr001_startup.launch

hironx.launch
-------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hironx.launch



.. code-block:: bash

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/kawada-hironx-example.l

This launch file shows an example of ros bridge for open kawada hironx robot, `hironx.launch` file launches folloiwng two program

1. `roslaunch hrpsys_ros_bridge hironx_startup.launch` : launch simulated robot environment, if you use real robot, you don't need this line

2. `roslaunch hrpsys_ros_bridge hironx_ros_bridge.launch` : launch rtm ros bridge

After that, we need to invoke the client program. Following program is the example of euslisp language

3. `rosrun roseus roseus kawada-hironx-example.l` : sample euslisp script to control the robot.

.. video:: build/images/HIRONX_Example
  :width: 600

.. video:: build/images/hironx_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/hironx_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/hironx_ros_bridge.launch" />
  
    <test args="--max-time=10   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/WAIST /HEAD_JOINT1_Link'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/kawada-hironx-example.l __log:=dummy'" pkg="openhrp3" test-name="HIRONX_Example" time-limit="300" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/HIRONX_Example.ogv" pkg="jsk_tools" test-name="z_HIRONX_Example" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/hironx_rviz.glc" pkg="jsk_tools" test-name="z_hironx_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

hironx_ros_bridge.launch
------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hironx_ros_bridge.launch

hironx_startup.launch
---------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hironx_startup.launch

hrp4c.launch
------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrp4c.launch



.. code-block:: bash

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/hrp4c-pickup.l

This launch file shows an example of ros bridge for open hrp4 robot, `hrp4c.launch` file launches folloiwng two program

1. `roslaunch hrpsys_ros_bridge hrp4c_startup.launch` : launch simulated robot environment, if you use real robot, you don't need this line

2. `roslaunch hrpsys_ros_bridge hrp4c_ros_bridge.launch` : launch rtm ros bridge

After that, we need to invoke the client program. Following program is the example of euslisp language

3. `rosrun roseus roseus hrp4c-pickup.l` : sample euslisp script to control the robot.

.. video:: build/images/HRP4C_PickUp
  :width: 600

.. video:: build/images/hrp4c_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/hrp4c_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/hrp4c_ros_bridge.launch" />
  
    <test args="--max-time=30   --target-directory=$(find hrpsys_ros_bridge)/build/images   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --check-tf='/BODY /R_HAND_J1_LINK'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/hrp4c-pickup.l __log:=dummy'" pkg="openhrp3" test-name="HRP4C_PickUp" time-limit="600" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/HRP4C_PickUp.ogv" pkg="jsk_tools" test-name="z_HRP4C_PickUp" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/hrp4c_rviz.glc" pkg="jsk_tools" test-name="z_hrp4c_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

hrp4c_nosim.launch
------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrp4c_nosim.launch



This script is "no-sim" version of `hrp4.launch`, launch `hrp4c_startup_nosim.launch` instead of `hrp4c_startup.launch` to run OpenHRP3 simulator without forward dynamics calculation.

.. video:: build/images/HRP4C_NoSimPickUp
  :width: 600

.. video:: build/images/hrp4c_nosim_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/hrp4c_startup_nosim.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/hrp4c_ros_bridge.launch" />
  
    <test args="--max-time=10   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/BODY /R_HAND_J1_LINK'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/hrp4c-pickup.l __log:=dummy'" pkg="openhrp3" test-name="HRP4C_NoSimPickUp" time-limit="600" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/HRP4C_NoSimPickUp.ogv" pkg="jsk_tools" test-name="z_HRP4C_NoSimPickUp" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/hrp4c_rviz.glc -o $(find hrpsys_ros_bridge)/build/images/hrp4c_nosim_rviz" pkg="jsk_tools" test-name="z_hrp4c_nosim_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

hrp4c_ros_bridge.launch
-----------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrp4c_ros_bridge.launch

hrp4c_startup.launch
--------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrp4c_startup.launch

hrp4c_startup_nosim.launch
--------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrp4c_startup_nosim.launch

hrpsys_ros_bridge.launch
------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge hrpsys_ros_bridge.launch

pa10.launch
-----------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pa10.launch



This launch file shows an example of ros bridge for pa10 robot

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/pa10_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/pa10_ros_bridge.launch" />
  
    </launch>

pa10_ros_bridge.launch
----------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pa10_ros_bridge.launch

pa10_startup.launch
-------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pa10_startup.launch

pr2.launch
----------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pr2.launch



This launch file shows an example of ros bridge for pr2 robot

.. video:: build/images/PR2_Example
  :width: 600

.. video:: build/images/pr2_rviz
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <env name="ORBgiopMaxMsgSize" value="42147483648" />
    <include file="$(find hrpsys_ros_bridge)/launch/pr2_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/pr2_ros_bridge.launch" />
  
    <test args="--max-time=10   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/base_footprint /head_plate_frame'   " pkg="openhrp3" test-name="PR2_Example" time-limit="600" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/PR2_Example.ogv" pkg="jsk_tools" test-name="z_PR2_Example" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/pr2_rviz.glc" pkg="jsk_tools" test-name="z_pr2_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

pr2_ros_bridge.launch
---------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pr2_ros_bridge.launch

pr2_startup.launch
------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge pr2_startup.launch

samplerobot.launch
------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge samplerobot.launch



.. code-block:: bash

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/samplerobot-pickup.l

This launch file shows an example of ros bridge for sample robot and client program `samplerobot-pickup.l`

.. video:: build/images/SampleRobot_PickUp
  :width: 600

.. video:: build/images/samplerobot_rviz-1
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/samplerobot_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/samplerobot_ros_bridge.launch" />
  
    <test args="--max-time=30   --viewer-name='' #   --no-start-simulation   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/WAIST_LINK0 /VISION_SENSOR1'   --script='rosrun roseus roseus $(find hrpsys_ros_bridge)/scripts/samplerobot-pickup.l __log:=dummy'" pkg="openhrp3" test-name="SampleRobot_PickUp" time-limit="1200" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/SampleRobot_PickUp.ogv" pkg="jsk_tools" test-name="z_SampleRobot_PickUp" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/samplerobot_rviz.glc" pkg="jsk_tools" test-name="z_samplerobot_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

samplerobot_ros_bridge.launch
-----------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge samplerobot_ros_bridge.launch

samplerobot_startup.launch
--------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge samplerobot_startup.launch

smartpal5.launch
----------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge smartpal5.launch




This launch file shows an example of ros bridge for yaskawa samartpal5 robot.

.. video:: build/images/SmartPal5_PickUp
  :width: 600

.. video:: build/images/smartpal5_rviz
  :width: 600


  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find hrpsys_ros_bridge)/launch/smartpal5_startup.launch" />
    <include file="$(find hrpsys_ros_bridge)/launch/smartpal5_ros_bridge.launch" />
  
    <test args="--max-time=10   --viewer-name='' #   --no-start-simulation   --capture-window='Irrlicht'   --target-directory=$(find hrpsys_ros_bridge)/build/images   --check-tf='/WAIST_LINK0 /VISION_SENSOR1'   " pkg="openhrp3" test-name="SmartPal5_PickUp" time-limit="600" type="test-grxui.py" /> 
    <test args="$(find hrpsys_ros_bridge)/build/images/SmartPal5_PickUp.ogv" pkg="jsk_tools" test-name="z_SmartPal5_PickUp" time-limit="1000" type="ogv_encode.sh" />
    <test args="$(find hrpsys_ros_bridge)/build/images/smartpal5_rviz.glc" pkg="jsk_tools" test-name="z_smartpal5_rviz" time-limit="2000" type="glc_encode.sh" />
  </launch>

smartpal5_ros_bridge.launch
---------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge smartpal5_ros_bridge.launch

smartpal5_startup.launch
------------------------

.. code-block:: bash

  roslaunch hrpsys_ros_bridge smartpal5_startup.launch

