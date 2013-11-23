#!/usr/bin/env python

try: # catkin does not requires load_manifest
    import hrpsys
except:
    import roslib; roslib.load_manifest('hrpsys_ros_bridge')

import rospy
import time
import copy
from std_msgs.msg import String
from diagnostic_msgs.msg import *
from hrpsys_ros_bridge.msg import MotorStates

motor_states = None
def states_cb(msg) :
    global motor_states
    motor_states = msg

publish_motor_id = 0
def publish_motor_states_diagnostics(msg) :
    global publish_motor_id
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = msg.header.stamp

    # motor
    i = publish_motor_id % len(msg.name)
    status = DiagnosticStatus(name = 'Motor ('+str(i)+":"+msg.name[i]+')', level = DiagnosticStatus.OK, message = "OK")
    if msg.servo_alarm[i] != 0 :
        status.message = "NG"
        status.level   = DiagnosticStatus.WARN
    if msg.driver_temp[i] > 55 :
        status.message = "High Temperature (" + str(msg.driver_temp[i])  + ")"
        status.level   = DiagnosticStatus.WARN
    status.values.append(KeyValue(key = "Calib State", value = str(msg.calib_state[i])))
    status.values.append(KeyValue(key = "Servo State", value = str(msg.servo_state[i])))
    status.values.append(KeyValue(key = "Power State", value = str(msg.power_state[i])))
    status.values.append(KeyValue(key = "Servo Alarm", value = str(msg.servo_alarm[i])))
    status.values.append(KeyValue(key = "Driver Temprature", value = str(msg.driver_temp[i])))
    diagnostic.status.append(status)

    pub.publish(diagnostic)

    publish_motor_id += 1

if __name__ == '__main__':
    try:
        last_message_stamp = 0
        rospy.init_node('motor_state_diagnostics')
        sub = rospy.Subscriber('motor_states', MotorStates, states_cb)
        pub = rospy.Publisher('diagnostics', DiagnosticArray)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if motor_states and motor_states.header.stamp != last_message_stamp :
                last_message_stamp = copy.copy(motor_states.header.stamp)
                publish_motor_states_diagnostics(motor_states)
            r.sleep()
    except rospy.ROSInterruptException: pass
