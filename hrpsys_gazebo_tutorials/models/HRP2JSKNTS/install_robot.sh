#!/bin/bash

if [ ! -e hrpsys ]; then
##
    mkdir meshes
    rosrun collada_tools collada_to_urdf $(rospack find jsk_hrpsys_ros_bridge)/models/HRP2JSKNTS.dae -G -A --mesh_output_dir `pwd`/meshes --mesh_prefix 'package://HRP2JSKNTS/meshes'
##
    sed -i -e 's@</robot>@  <gazebo>\n    <plugin filename="libIOBPlugin.so" name="hrpsys_gazebo_plugin" >\n      <robotname>HRP2JSKNTS</robotname>\n      <controller>hrpsys_gazebo_configuration</controller>\n    </plugin>\n  </gazebo>\n</robot>@g' HRP2JSKNTS.urdf
# continuous joint not working in GAZEBO
    sed -i -e 's@continuous@revolute@g' HRP2JSKNTS.urdf

    mkdir hrpsys
    ln -s  $(rospack find jsk_hrpsys_ros_bridge)/models/HRP2JSKNTS.conf hrpsys
    ln -s  $(rospack find jsk_hrpsys_ros_bridge)/models/HRP2JSKNTS.RobotHardware.conf hrpsys
    ln -s  $(rospack find jsk_hrpsys_ros_bridge)/models/HRP2JSKNTS.dae hrpsys
    ln -s  $(rospack find jsk_hrpsys_ros_bridge)/models/HRP2JSKNTS.xml hrpsys
fi

