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

    # servo on
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

    # calib done
    status = DiagnosticStatus(name = 'Calibration Mode', level = DiagnosticStatus.OK, message = "Done")
    for i in range(len(msg.name)) :
        if ( msg.calib_state[i] == False ) :
            if ( status.level == DiagnosticStatus.OK ) :
                status.message = "Not Calibrated"
                status.level   = DiagnosticStatus.WARN
            status.message += ", " + msg.name
    diagnostic.status.append(status)

    # power on
    status = DiagnosticStatus(name = 'Power Mode', level = DiagnosticStatus.OK, message = "Power On")
    for i in range(len(msg.name)) :
        if ( msg.power_state[i] == False ) :
            status.message = "Power Off"
            status.level   = DiagnosticStatus.WARN
    diagnostic.status.append(status)


    # error
    for i in range(len(msg.name)) :
        status = DiagnosticStatus(name = 'Motor ('+msg.name[i]+')', level = DiagnosticStatus.OK, message = "OK")
        if ( msg.servo_alarm[i] != 0 ) :
            status.message = "NG"
            status.level   = DiagnosticStatus.WARN
        status.values.append(KeyValue(key = "Calib State", value = str(msg.calib_state[i])))
        status.values.append(KeyValue(key = "Servo State", value = str(msg.servo_state[i])))
        status.values.append(KeyValue(key = "Power State", value = str(msg.power_state[i])))
        status.values.append(KeyValue(key = "Servo Alarm", value = str(msg.servo_alarm[i])))
        status.values.append(KeyValue(key = "Driver Temprature", value = str(msg.driver_temp[i])))
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
