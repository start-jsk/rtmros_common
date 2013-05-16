#!/usr/bin/env python

import roslib; roslib.load_manifest('hrpsys_gazebo')
import rospy, math
import sys
from math import *

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from atlas_msgs.msg import AtlasSimInterfaceCommand

def move(x, y, z, w=0):
    # Initialize the node
    rospy.init_node('move_atlas')

    # Setup the publishers
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    sim_interface = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
    set_pose= rospy.Publisher('/atlas/set_pose', Pose, None, False, True, None)

    while set_pose.get_num_connections() == 0:
          rospy.sleep(0.1)
    while rospy.get_time() < 5.0:
          rospy.sleep(0.1)

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = 0
    p.orientation.y = 0
    p.orientation.z = sin(w/2.0)
    p.orientation.w = cos(w/2.0)

    com = AtlasSimInterfaceCommand()

    rospy.sleep(0.1)
    mode.publish("harnessed")
    com.behavior = com.FREEZE
    sim_interface.publish(com)
    com.behavior = com.STAND_PREP
    sim_interface.publish(com)
    rospy.sleep(2.0)
    mode.publish("nominal")
    rospy.sleep(0.05)
    set_pose.publish(p)
    rospy.sleep(0.2)
    com.behavior = com.STAND
    sim_interface.publish(com)

if __name__ == '__main__':
    argvs = sys.argv
    argc = len(argvs)

    try:
        move(float(argvs[1]), float(argvs[2]), float(argvs[3]), float(argvs[4])*pi/180)
    except rospy.ROSInterruptException: pass

