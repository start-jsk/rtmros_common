#!/usr/bin/env python

PKG = 'hrpsys_tutorials'
import roslib; roslib.load_manifest(PKG)
import rospkg

import numpy
import unittest

import sys; sys.path.insert (0, roslib.packages.get_pkg_dir('hrpsys')+'/scripts'); ## add python
from hrpsys import HrpsysConfigurator

class TestSampleRobot(unittest.TestCase):

    def setUp(self):
        self.robot = HrpsysConfigurator()
        self.robot.init("SampleRobot(Robot)0", rospkg.RosPack().get_path("openhrp3")+"/share/OpenHRP-3.1/sample/model/sample1.wrl")


    def test_ik(self):
        self.robot.seq_svc.addJointGroup("LARM", ["LARM_SHOULDER_P", "LARM_SHOULDER_R", "LARM_SHOULDER_Y", "LARM_ELBOW", "LARM_WRIST_Y", "LARM_WRIST_P", "LARM_WRIST_R"])
        lav = [60,0,0,-90,0,0,0]
        for av in [[60, 0, 0, -80, 0, 0, 0],[30, 30, 0,-80, 0, 0, 0],
                   [40,40,20, -20, 0, 0, 0],[30, 30,20,-80,20,0, 20]]:
            self.robot.setJointAnglesOfGroup("LARM", av, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            pos1 = self.robot.getReferencePosition("LARM_WRIST_R")
            rpy1 = self.robot.getReferenceRPY("LARM_WRIST_R")
            self.robot.setJointAnglesOfGroup("LARM", lav, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            self.assertTrue(self.robot.setTargetPose("LARM", pos1, rpy1, 10))
            self.robot.waitInterpolationOfGroup("LARM")
            pos2 = self.robot.getReferencePosition("LARM_WRIST_R")
            rpy2 = self.robot.getReferenceRPY("LARM_WRIST_R")
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3)
            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<5.0e-3)
            lav = av



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_samplerobot', TestSampleRobot) 




