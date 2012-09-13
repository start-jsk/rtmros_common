#!/usr/bin/env python
import roslib; roslib.load_manifest('hrpsys_ros_bridge')
import rospy
import time
from std_msgs.msg import String
from diagnostic_msgs.msg import *
from hrpsys_ros_bridge.msg import MotorStates

servo_alarm = {
    0x001 : 'SS_OVER_VOLTAGE',
    0x002 : 'SS_OVER_LOAD',
    0x004 : 'SS_OVER_VELOCITY',
    0x008 : 'SS_OVER_CURRENT',
    0x010 : 'SS_OVER_HEAT',
    0x020 : 'SS_TORQUE_LIMIT',
    0x040 : 'SS_VELOCITY_LIMIT',
    0x080 : 'SS_FORWARD_LIMIT',
    0x100 : 'SS_REVERSE_LIMIT',
    0x200 : 'SS_POSITION_ERROR',
    0x300 : 'SS_ENCODER_ERROR',
    0x800 : 'SS_OTHER'
}

def states_cb(msg):
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = msg.header.stamp
    status = DiagnosticStatus(name = 'Operating Mode', level = DiagnosticStatus.OK, message = "Servo On")

    alarm_mesasge = ''
    for i in range(len(msg.name)) :
        if ( status.level == DiagnosticStatus.OK and
             ( msg.servo_alarm[i] > 0 or msg.servo_state[i] == False ) ) :
            status.message = "Servo Off"
            status.level   = DiagnosticStatus.WARN
        if ( msg.servo_alarm[i] > 0 ):
            status.message = "Servo Error Alarm"
            status.level   = DiagnosticStatus.ERROR
            status.values.append(KeyValue(key = msg.name[i], value = servo_alarm.get(msg.servo_alarm[i], str(msg.servo_alarm[i]))))

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
