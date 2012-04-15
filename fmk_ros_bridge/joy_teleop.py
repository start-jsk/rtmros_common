#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('joy')
roslib.load_manifest('nav_msgs')
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import math

def joy_cb(msg):
    tws = geometry_msgs.msg.Twist()
    but = msg.buttons
    ax = msg.axes
    if but[10] > 0:
        if but[11] > 0:
            sp = 0.5
        else:
            sp = 0.25
        tws.linear = geometry_msgs.msg.Vector3(ax[3] * sp,0,0)
        if tws.linear.x < 0:
            tws.angular = geometry_msgs.msg.Vector3(0,0,-1.0 * (ax[0] + ax[2]) * math.pi * 0.9 * sp)
        else:
            tws.angular = geometry_msgs.msg.Vector3(0,0,(ax[0] + ax[2]) * math.pi * 0.9 * sp)
    elif len(but) == 11 and but[3] > 0: # wii
        tws.linear  =  geometry_msgs.msg.Vector3((but[8]-but[9]),0,0)
        tws.angular = geometry_msgs.msg.Vector3(0,0,(but[6]-but[7]) * math.pi * 0.4)
    elif len(but) == 11: #wii
        return
    else:
        tws.linear = geometry_msgs.msg.Vector3(0,0,0)
        tws.angular = geometry_msgs.msg.Vector3(0,0,0)
    pub.publish(tws)

def fmk_teleop():
    global pub
    rospy.init_node('fmk_teleop',anonymous=True)
    rospy.Subscriber("/joy",Joy,joy_cb)
    pub = rospy.Publisher('/teleop/cmd_vel',geometry_msgs.msg.Twist)
    rospy.spin()

if __name__ == '__main__':
    fmk_teleop()
