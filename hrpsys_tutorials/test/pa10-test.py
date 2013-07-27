#!/usr/bin/env python

PKG = 'hrpsys_tutorials'
import roslib; roslib.load_manifest(PKG)
import rospkg

import numpy
import unittest

from hrpsys_config import HrpsysConfigurator

class TestPA10(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot = HrpsysConfigurator()
        self.robot.init("PA10Controller(Robot)0", rospkg.RosPack().get_path("openhrp3")+"/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl")

    def test_ik(self):
        self.robot.seq_svc.addJointGroup("ARM", ["J1", "J2", "J3", "J4", "J5", "J6"])
        lav = [60,0,0,-90,0,0]
        for av in [[60, 0, 0, -80, 0, 0],[30, 30, 0,-80, 0, 0],
                   [40,40,20, -20, 0, 0],[30, 30,20,-80,20,20]]:
            self.robot.setJointAnglesOfGroup("ARM", av, 5)
            self.robot.waitInterpolationOfGroup("ARM")
            pos1 = self.robot.getReferencePosition("J6")
            rpy1 = self.robot.getReferenceRPY("J6")
            self.robot.setJointAnglesOfGroup("ARM", lav, 5)
            self.robot.waitInterpolationOfGroup("ARM")
            self.assertTrue(self.robot.setTargetPose("ARM", pos1, rpy1, 10))
            self.robot.waitInterpolationOfGroup("ARM")
            pos2 = self.robot.getReferencePosition("J6")
            rpy2 = self.robot.getReferenceRPY("J6")
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3)
            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<5.0e-3)
            lav = av



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pa10', TestPA10) 




