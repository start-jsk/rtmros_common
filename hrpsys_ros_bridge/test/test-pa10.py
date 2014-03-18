#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test-samplerobot'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import argparse,unittest,rostest, time, sys, math
from numpy import *

import rospy,rospkg, tf
from geometry_msgs.msg import WrenchStamped
from hrpsys_ros_bridge.srv import OpenHRP_SequencePlayerService_loadPattern, OpenHRP_SequencePlayerService_waitInterpolation

import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TestPA10Robot(unittest.TestCase):
    listener = None
    lfsensor = None
    rfsensor = None

    def setUp(self):
        rospy.init_node('TestPA10Robot')
        self.listener = tf.TransformListener()

    def test_tf_odom_J7_LINK(self): # need to check if map/ is published?
        try:
            self.listener.waitForTransform('/odom', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /odom to /J7_LINK")
        (trans,rot) = self.listener.lookupTransform('/odom', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /odom /J7_LINK %r %r"%(trans,rot))
        self.assertAlmostEqual(trans[2],1.0,delta=0.5)

    # send joint angles
    def test_joint_angles(self):
        larm = actionlib.SimpleActionClient("/fullbody_controller/joint_trajectory_action", JointTrajectoryAction)
        larm.wait_for_server()

        try:
            self.listener.waitForTransform('/BASE_LINK', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /BASE_LINK to /J7_LINK")
        (trans1,rot1) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans1,rot1))
        goal = JointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("J1")
        goal.trajectory.joint_names.append("J2")
        goal.trajectory.joint_names.append("J3")
        goal.trajectory.joint_names.append("J4")
        goal.trajectory.joint_names.append("J5")
        goal.trajectory.joint_names.append("J6")
        goal.trajectory.joint_names.append("J7")

        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        larm.send_goal(goal)
        larm.wait_for_result()
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertNotAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestPA10Robot, sys.argv)
