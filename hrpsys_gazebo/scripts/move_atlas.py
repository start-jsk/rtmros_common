#!/usr/bin/env python

import roslib; roslib.load_manifest('hrpsys_gazebo')
import rospy, math
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Pose

def move(x, y, z):
    # Initialize the node
    rospy.init_node('move_atlas')

    # Setup the publishers
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    control_mode = rospy.Publisher('/atlas/control_mode', String, None, False, True, None)
    set_pose= rospy.Publisher('/atlas/set_pose', Pose, None, False, True, None)

    while set_pose.get_num_connections() == 0:
          rospy.sleep(0.1)
    while rospy.get_time() < 5.0:
          rospy.sleep(0.1)

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1
    set_pose.publish(p)
    rospy.sleep(0.1)
    mode.publish("harnessed")
    control_mode.publish("StandPrep")
    rospy.sleep(3.0)
    mode.publish("nominal")
    rospy.sleep(0.3)
    control_mode.publish("Stand")

if __name__ == '__main__':
    argvs = sys.argv
    argc = len(argvs)

    try:
        move(float(argvs[1]), float(argvs[2]), float(argvs[3]))
    except rospy.ROSInterruptException: pass

