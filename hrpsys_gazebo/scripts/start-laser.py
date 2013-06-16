#!/usr/bin/env python
import rospy, sys
from std_msgs.msg import Float64

#rostopic pub /multisense_sl/set_spindle_speed std_msgs/Float64 1.2

def start_laser(speed):
    pub = rospy.Publisher('/multisense_sl/set_spindle_speed', Float64)
    rospy.init_node('start_laser_node')

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    pub.publish(speed)

if __name__ == '__main__':
    try:
        argvs = sys.argv
        argc = len(argvs)
        speed = 1.2
        if argc > 1:
            speed = float(argvs[1])
        start_laser(speed)
    except rospy.ROSInterruptException:
        pass
