#!/usr/bin/env python

PKG = 'openrtm_ros_bridge'
import roslib; roslib.load_manifest(PKG)
import rospy

import unittest
import openrtm_ros_bridge.srv
#from openrtm_ros_bridge.srv import *

class TestMyServiceRosBridge(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rospy.init_node('test_myservice_rosbridge')

    def testEcho(self):
        rospy.wait_for_service("/bridge/echo")
        echo = rospy.ServiceProxy("/bridge/echo", openrtm_ros_bridge.srv.SimpleService_MyService_echo)
        req = openrtm_ros_bridge.srv.SimpleService_MyService_echo
        msg = "this is test data 123"
        rospy.loginfo("send request > " + msg)
        res = echo(msg)
        rospy.loginfo("check return < " + res.operation_return)
        self.assertTrue(res.operation_return == msg, "SimpleService.echo returns incorrect string")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_myservice_rosbridge', TestMyServiceRosBridge)
