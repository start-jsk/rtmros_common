#!/bin/bash

if [ ! -e hrpsys ]; then
##
    mkdir meshes
    rosrun collada_tools collada_to_urdf $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae -G -A --mesh_output_dir `pwd`/meshes --mesh_prefix 'package://SampleRobot/meshes'
##
    sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>SampleRobot</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' SampleRobot.urdf
# continuous joint not working in GAZEBO
    sed -i -e 's@continuous@revolute@g' SampleRobot.urdf

    mkdir hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.RobotHardware.conf hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot.dae hrpsys
    ln -s  $(rospack find hrpsys_ros_bridge_tutorials)/models/SampleRobot_nosim.xml hrpsys
fi
