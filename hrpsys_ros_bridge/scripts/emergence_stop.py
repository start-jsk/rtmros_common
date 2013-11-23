#!/usr/bin/env python

try: # catkin does not requires load_manifest
    import hrpsys
except:
    import roslib; roslib.load_manifest('hrpsys_ros_bridge')

import time

import rospy
from sensor_msgs.msg import Joy

import hrpsys_config
from hrpsys.srv import *

def joystick_callback(msg) :
    if msg.buttons[3] == 1: # push start
        rospy.loginfo("push start button")
        servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
        power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
        try:
            servo(OpenHRP_RobotHardwareService_servoRequest("all",1));
            time.sleep(1)
            power(OpenHRP_RobotHardwareService_powerRequest("all",1))
        except rospy.ServiceException, e:
            rospy.logerr("Failed to put the hrpsys in serv off mode: service call failed with error: %s"%(e))

if __name__ == '__main__':
    try:
        rospy.init_node('emergence_stop')
        sub = rospy.Subscriber('/joy', Joy, joystick_callback, None, 1);
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()
    except rospy.ROSInterruptException: pass


