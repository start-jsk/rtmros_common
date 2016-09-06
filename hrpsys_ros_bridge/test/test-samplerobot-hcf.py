#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test_samplerobot_hcf'

import hrpsys
from hrpsys.hrpsys_config import *
import OpenHRP

import samplerobot_hrpsys_config

import rospy
import unittest, rostest, os, rospkg, sys
from geometry_msgs.msg import WrenchStamped

class TestSampleRobotHcf(unittest.TestCase):
    off_lfsensor = None
    ref_lfsensor = None

    def off_lfsensor_cb(self, sensor):
        self.off_lfsensor = sensor
    def ref_lfsensor_cb(self, sensor):
        self.ref_lfsensor = sensor

    def setUp(self):
        rospy.init_node('TestSampleRobot')
        rospy.Subscriber('/off_lfsensor', WrenchStamped, self.off_lfsensor_cb)
        rospy.Subscriber('/ref_lfsensor', WrenchStamped, self.ref_lfsensor_cb)
        pass

    def test_off_force_sensor(self):
        while self.off_lfsensor == None:
            time.sleep(1)
            rospy.logwarn("wait for off_lfsensor...")
        self.assertEqual(self.off_lfsensor.header.frame_id, "lfsensor")

    def test_ref_force_sensor(self):
        while self.ref_lfsensor == None:
            time.sleep(1)
            rospy.logwarn("wait for ref_lfsensor...")
        self.assertEqual(self.ref_lfsensor.header.frame_id, "lfsensor")

    def test_hcf_init(self):
        hcf = samplerobot_hrpsys_config.SampleRobotHrpsysConfigurator()
        model_url = rospkg.RosPack().get_path("openhrp3") + "/share/OpenHRP-3.1/sample/model/sample1.wrl"
        if os.path.exists(model_url):
            try:
                hcf.init("SampleRobot(Robot)0", model_url)
                assert(True)
            except AttributeError as e:
                print >> sys.stderr, "[test-samplerobot-hcf.py] catch exception", e
                assert(False)

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestSampleRobotHcf, sys.argv)
