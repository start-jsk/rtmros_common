#!/usr/bin/env python

PKG = 'hrpsys_tutorials'
import roslib; roslib.load_manifest(PKG)
import rospkg

import numpy
import unittest

from hrpsys_config import HrpsysConfigurator

class TestHRP4C(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot = HrpsysConfigurator()
        self.robot.init("HRP4C(Robot)0", rospkg.RosPack().get_path("hrpsys")+"/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl")

    def test_walk(self):
        self.robot.loadPattern(rospkg.RosPack().get_path("hrpsys")+"/share/hrpsys/samples/HRP4C/data/walk2m", 1.0)
        self.robot.waitInterpolation()


        print self.robot.getReferencePosition("WAIST")
        print self.robot.getReferenceRotation("WAIST")
        self.assertTrue(numpy.linalg.norm(self.robot.getReferencePosition("WAIST") - numpy.array([0,0,0.78415])) < 1.0e-6)
        self.assertTrue(numpy.array_equal(self.robot.getReferenceRotation("WAIST"),numpy.identity(3)))

    def test_ik(self):
        self.robot.seq_svc.addJointGroup("LARM", ["L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P", "L_WRIST_Y", "L_WRIST_R"])
        lav = [60,0,0,-90,0,0]
        for av in [[60, 0, 0, -80, 0, 0],[30, 30, 0,-80, 0, 0],
                   [40,40,20, -20, 0, 0],[30, 30,20,-80,20,20]]:
            self.robot.setJointAnglesOfGroup("LARM", av, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            pos1 = self.robot.getReferencePosition("L_WRIST_R")
            rpy1 = self.robot.getReferenceRPY("L_WRIST_R")
            self.robot.setJointAnglesOfGroup("LARM", lav, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            self.assertTrue(self.robot.setTargetPose("LARM", pos1, rpy1, 10))
            self.robot.waitInterpolationOfGroup("LARM")
            pos2 = self.robot.getReferencePosition("L_WRIST_R")
            rpy2 = self.robot.getReferenceRPY("L_WRIST_R")
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3)
            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<5.0e-3)
            lav = av



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hrp4c', TestHRP4C) 




