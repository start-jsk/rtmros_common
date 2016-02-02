#! /usr/bin/env python

# This script extracts tf from HrpsysSeqStateROSBridge to test tf rate of HrpsysSeqStateROSBridge in rostest.

import rospy
import tf
from tf.msg import tfMessage

class HrpsysSeqStateROSBridgeTFRelay:
    def __init__(self):
        rospy.init_node("HrpsysSeqStateROSBridgeTFRelay", anonymous=True)
        rospy.Subscriber('/tf', tfMessage, self.tf_callback)
        self.broadcaster = rospy.Publisher("/hrpsys_ros_bridge/tf", tfMessage, queue_size = 100)

    def execute(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def tf_callback(self, msg):
        if msg._connection_header['callerid'] == '/HrpsysSeqStateROSBridge':
            self.broadcaster.publish(msg)
                
if __name__ == '__main__':
    try:
        node = HrpsysSeqStateROSBridgeTFRelay()
        node.execute()
    except rospy.ROSInterruptException: pass
