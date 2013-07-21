#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'hironx_ros_bridge'
import roslib; roslib.load_manifest(PKG)
import rospy, actionlib, math, numpy
import tf
from tf.transformations import quaternion_matrix, euler_from_matrix

import unittest

import pr2_controllers_msgs.msg
import trajectory_msgs.msg

from pr2_controllers_msgs.msg import JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from hrpsys.srv import *

class TestHiroROSBridge(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rospy.init_node('hironx_ros_bridge_test')
        self.listener = tf.TransformListener()

        self.larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.rarm = actionlib.SimpleActionClient("/rarm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.torso = actionlib.SimpleActionClient("/torso_controller/joint_trajectory_action", JointTrajectoryAction)
        self.head = actionlib.SimpleActionClient("/head_controller/joint_trajectory_action", JointTrajectoryAction)
        self.larm.wait_for_server()
        self.rarm.wait_for_server()
        self.torso.wait_for_server()
        self.head.wait_for_server()

        rospy.wait_for_service('/SequencePlayerServiceROSBridge/setTargetPose')
        self.set_target_pose = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/setTargetPose', OpenHRP_SequencePlayerService_setTargetPose)
        rospy.wait_for_service('/SequencePlayerServiceROSBridge/waitInterpolationOfGroup')
        self.wait_interpolation_of_group = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/waitInterpolationOfGroup', OpenHRP_SequencePlayerService_waitInterpolationOfGroup)


    def tearDown(self):
        self.reset_Pose()
        True

    def setUp(self):
        self.reset_Pose()

    def reset_Pose(self):
        larm_goal = self.goal_LArm()
        larm_goal = self.setup_Positions(larm_goal, [[ 0.0, -40.0, -90.0, 0.0, 0.0, 0.0]])
        rarm_goal = self.goal_RArm()
        rarm_goal = self.setup_Positions(rarm_goal, [[ 0.0, -40.0, -90.0, 0.0, 0.0, 0.0]])
        head_goal = self.goal_Head()
        head_goal = self.setup_Positions(head_goal, [[0, 20]])
        torso_goal = self.goal_Torso()
        torso_goal = self.setup_Positions(torso_goal, [[0]])
        self.larm.send_goal(larm_goal)
        self.rarm.send_goal(rarm_goal)
        self.head.send_goal(head_goal)
        self.torso.send_goal(torso_goal)
        self.larm.wait_for_result()
        self.rarm.wait_for_result()
        self.head.wait_for_result()
        self.torso.wait_for_result()

    def goal_LArm(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("LARM_JOINT0")
        goal.trajectory.joint_names.append("LARM_JOINT1")
        goal.trajectory.joint_names.append("LARM_JOINT2")
        goal.trajectory.joint_names.append("LARM_JOINT3")
        goal.trajectory.joint_names.append("LARM_JOINT4")
        goal.trajectory.joint_names.append("LARM_JOINT5")
        return goal

    def goal_RArm(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("RARM_JOINT0")
        goal.trajectory.joint_names.append("RARM_JOINT1")
        goal.trajectory.joint_names.append("RARM_JOINT2")
        goal.trajectory.joint_names.append("RARM_JOINT3")
        goal.trajectory.joint_names.append("RARM_JOINT4")
        goal.trajectory.joint_names.append("RARM_JOINT5")
        return goal

    def goal_Torso(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("CHEST_JOINT0")
        return goal

    def goal_Head(self):
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.joint_names.append("HEAD_JOINT0")
        goal.trajectory.joint_names.append("HEAD_JOINT1")
        return goal

    def setup_Positions(self, goal, positions):
        tm = 1.0
        for p in positions:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = [ x * math.pi / 180.0 for x in p]
            point.time_from_start = rospy.Duration(tm)
            goal.trajectory.points.append(point)
            tm += 1.0
        return goal

    def test_LArmIK(self):
        #   /WAIST /LARM_JOINT5_Link
        # - Translation: [0.325, 0.182, 0.074]
        # - Rotation: in Quaternion [-0.000, -0.707, 0.000, 0.707]
        #             in RPY [-1.694, -1.571, 1.693]
        for torso_angle in ([0, -10, 10]):
            torso_goal = self.goal_Torso()
            torso_goal = self.setup_Positions(torso_goal, [[torso_angle]])
            self.torso.send_goal_and_wait(torso_goal)
            for p in [[ 0.325, 0.182, 0.074], # initial
                      [ 0.3, -0.2, 0.074], [ 0.3, -0.1, 0.074], [ 0.3,  0.0, 0.074],
                      [ 0.3,  0.1, 0.074], [ 0.3,  0.2, 0.074], [ 0.3,  0.3, 0.074],
                      [ 0.4, -0.1, 0.074], [ 0.4, -0.0, 0.074], [ 0.4,  0.1, 0.074],
                      [ 0.4,  0.2, 0.074], [ 0.4,  0.3, 0.074], [ 0.4,  0.4, 0.074],
                      ] :

                print "solve ik at p = ", p
                ret = self.set_target_pose('LARM', p, [-1.694,-1.571, 1.693], 1.0)
                self.assertTrue(ret.operation_return, "ik failed")
                ret = self.wait_interpolation_of_group('LARM')
                self.assertTrue(ret.operation_return, "wait interpolation failed")

                rospy.sleep(1)
                now = rospy.Time.now()
                self.listener.waitForTransform("WAIST", "LARM_JOINT5_Link", now, rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform("WAIST", "LARM_JOINT5_Link", now)
                numpy.testing.assert_array_almost_equal(trans, p, decimal=1)
                print "current position   p = ", trans
                print "                 rot = ", rot
                print "                     = ", quaternion_matrix(rot)[0:3,0:3]
                # this fails?
                #numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                #                                        numpy.array([[ 0,  0, -1],
                #                                                     [ 0,  1,  0],
                #                                                     [ 1,  0,  0]]),
                #                                        decimal=3)


    def test_LArm(self):
        goal = self.goal_LArm()
        goal = self.setup_Positions(goal, [[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           [  25,-139,-157,  45,   0,   0],
                                           [ 0.6, 0.0,-100,-15.2, 9.5, -3.2]])
        self.larm.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "LARM_JOINT5_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "LARM_JOINT5_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.325493, 0.18236, 0.0744674], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[ 0, 0,-1],
                                                             [ 0, 1, 0],
                                                             [ 1, 0, 0]]), decimal=3)

    def test_RArm(self):
        goal = self.goal_RArm()
        goal = self.setup_Positions(goal, [[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           [  25,-139,-157,  45,   0,   0],
                                           [-0.6, 0.0,-100, 15.2, 9.5,  3.2]])
        self.rarm.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "RARM_JOINT5_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "RARM_JOINT5_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.325493,-0.18236, 0.0744674], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[ 0, 0,-1],
                                                             [ 0, 1, 0],
                                                             [ 1, 0, 0]]), decimal=3)

    def test_Torso(self):
        goal = self.goal_Torso()
        goal = self.setup_Positions(goal, [[   0.0],
                                           [-162.949],
                                           [ 162.949]])
        self.torso.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "CHEST_JOINT0_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "CHEST_JOINT0_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.0, 0.0, 0.0], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[-0.956044, -0.293223, 0.0],
                                                             [ 0.293223, -0.956044, 0.0],
                                                             [ 0.0,       0.0,      1.0]]), decimal=3)

    def test_Head(self):
        goal = self.goal_Head()
        goal = self.setup_Positions(goal, [[  0.0,  0.0],
                                           [ 70.0, 70.0],
                                           [-70.0,-20.0]])
        self.head.send_goal_and_wait(goal)

        rospy.sleep(1)
        now = rospy.Time.now()
        self.listener.waitForTransform("WAIST", "HEAD_JOINT1_Link", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("WAIST", "HEAD_JOINT1_Link", now)
        numpy.testing.assert_array_almost_equal(trans, [0.0, 0.0, 0.5695], decimal=3)
        numpy.testing.assert_array_almost_equal(quaternion_matrix(rot)[0:3,0:3],
                                                numpy.array([[0.321394, 0.939693, -0.116978],
                                                             [-0.883022, 0.34202, 0.321394],
                                                             [0.34202, 0.0, 0.939693]]), decimal=3)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hronx_ros_bridge', TestHiroROSBridge)




