#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test_samplerobot_hcf'

import hrpsys
from hrpsys.hrpsys_config import *
import OpenHRP

import samplerobot_hrpsys_config

import unittest, rostest, os, rospkg, sys

class TestSampleRobotHcf(unittest.TestCase):

    def setUp(self):
        pass

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
