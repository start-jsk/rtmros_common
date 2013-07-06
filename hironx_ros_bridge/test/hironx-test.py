#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'hironx_ros_bridge'
import roslib, sys; sys.path.insert (0, roslib.packages.get_pkg_dir(PKG)+'/scripts'); ## add python  path
import hironx

import numpy
import unittest

class TestHiroNX(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot = hironx.HIRONX()
        self.robot.init()

    def test_goInitial(self):
        self.robot.goInitial()

    def test_ik(self):
        lav = [60,0,-90,0,0,0]
        for av in [[60, 0,-80, 0, 0, 0],[30,  0,-80, 0, 0, 0]]:
            self.robot.setJointAnglesOfGroup("LARM", av, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            pos1 = self.robot.getReferencePosition("LARM_JOINT5")
            rpy1 = self.robot.getReferenceRPY("LARM_JOINT5")
            self.robot.setJointAnglesOfGroup("LARM", lav, 5)
            self.robot.waitInterpolationOfGroup("LARM")
            self.assertTrue(self.robot.setTargetPose("LARM", pos1, rpy1, 10))
            self.robot.waitInterpolationOfGroup("LARM")
            pos2 = self.robot.getReferencePosition("LARM_JOINT5")
            rpy2 = self.robot.getReferenceRPY("LARM_JOINT5")
            print "pos", pos1, pos2, numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))
            print "rpy", rpy1, rpy2, numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))
            self.assertTrue(numpy.linalg.norm(numpy.array(pos1)-numpy.array(pos2))<5.0e-3)
            self.assertTrue(numpy.linalg.norm(numpy.array(rpy1)-numpy.array(rpy2))<5.0e-3)
            lav = av

    def test_goOffPose(self):
        self.robot.goOffPose()



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx', TestHiroNX) 




