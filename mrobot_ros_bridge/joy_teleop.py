#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('joy')
import rospy
import geometry_msgs.msg
from joy.msg import Joy
import math

def joy_cb(msg):
    tws = geometry_msgs.msg.Twist()
    but = msg.buttons
    ax = msg.axes
    if but[10] > 0:
        if but[11] > 0:
            sp = 1.0
        else:
            sp = 0.5
        tws.linear = geometry_msgs.msg.Vector3(ax[3] * sp,ax[2] * -1.0 * sp,0)
        tws.angular = geometry_msgs.msg.Vector3(0,0,ax[0] * math.pi * 0.3 *sp)
    else:
        tws.linear = geometry_msgs.msg.Vector3(0,0,0)
        tws.angular = geometry_msgs.msg.Vector3(0,0,0)
    pub.publish(tws)

def beego_teleop():
    global pub
    rospy.init_node('beego_teleop',anonymous=True)
    rospy.Subscriber("/joy",Joy,joy_cb)
    pub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist)
    rospy.spin()


if __name__ == '__main__':
    beego_teleop()
