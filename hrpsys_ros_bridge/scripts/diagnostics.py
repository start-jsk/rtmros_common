#!/usr/bin/env python
import roslib; roslib.load_manifest('hrpsys_ros_bridge')
import rospy
import time
from std_msgs.msg import String
from diagnostic_msgs.msg import *
from hrpsys_ros_bridge.msg import MotorStates

def states_cb(msg):
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = msg.header.stamp
    status = DiagnosticStatus(name = 'Operating Mode', level = DiagnosticStatus.OK, message = "Servo On")

    for i in range(len(msg.name)) :
        if ( msg.servo_alarm[i] > 0 ) :
            status.message = "Servo Off"

    diagnostic.status.append(status)
    pub.publish(diagnostic)

if __name__ == '__main__':
    try:
        global initial_flag, time_latest
        rospy.init_node('hrpsys_diagnostics')
        sub = rospy.Subscriber('motor_states', MotorStates, states_cb)
        pub = rospy.Publisher('diagnostics', DiagnosticArray)

        rospy.spin()
    except rospy.ROSInterruptException: pass
