#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test_pa10'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import argparse,unittest,rostest, time, sys, math, os
from copy import deepcopy
from numpy import *

import rospy,rospkg, tf
from geometry_msgs.msg import WrenchStamped
from hrpsys_ros_bridge.srv import OpenHRP_SequencePlayerService_loadPattern, OpenHRP_SequencePlayerService_waitInterpolation

import actionlib

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

class TestPA10Robot(unittest.TestCase):
    listener = None
    lfsensor = None
    rfsensor = None
    feedback = None

    def send_joint_angles(self,angles,wait=True):
        pub = rospy.Publisher("/fullbody_controller/command", JointTrajectory)
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in angles ]
        point.time_from_start = rospy.Duration(5.0)
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time().now()
        msg.joint_names = ["J1","J2","J3","J4","J5","J6","J7"]
        msg.points = [point]
        pub.publish(msg)
        if wait:
            rospy.sleep(rospy.Duration(6.0))

    def setUp(self):
        rospy.init_node('TestPA10Robot')
        self.listener = tf.TransformListener()
        self.send_joint_angles([0,0,0,0,0,0,0])

    def test_tf_base_link_J7_LINK(self): # need to check if map/ is published?
        try:
            self.listener.waitForTransform('/BASE_LINK', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /BASE_LINK to /J7_LINK")
        (trans,rot) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans,rot))
        self.assertAlmostEqual(trans[2],1.0,delta=0.5)

    # send joint angles
    def test_joint_trajectory_action(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_joint_trajectory_action(larm, JointTrajectoryGoal())

    def test_follow_joint_trajectory_action(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_joint_trajectory_action(larm, FollowJointTrajectoryGoal())

    def feedback_cb(self, msg):
        self.feedback = msg

    def impl_test_joint_trajectory_action(self, larm, goal):
        larm.wait_for_server()

        try:
            self.listener.waitForTransform('/BASE_LINK', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /BASE_LINK to /J7_LINK")
        (trans1,rot1) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans1,rot1))
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
        larm.send_goal(goal, feedback_cb = self.feedback_cb)
        rospy.sleep(2.5)
        mid_feedback = deepcopy(self.feedback)
        larm.wait_for_result()
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertNotAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)
        # time_from_start in feedback should have a meaning
        # https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/233#issuecomment-403416846
        # JointTrajectoryAction in pr2_controllers_msgs has empty feedback
        if hasattr(mid_feedback, 'desired'):
            rospy.logwarn("time_from_start of desired of midpoint feedback: %r"%mid_feedback.desired.time_from_start.to_sec())
            self.assertAlmostEqual(mid_feedback.desired.time_from_start.to_sec(), 2.5, delta=1)
            rospy.logwarn("time_from_start of actual of midpoint feedback: %r"%mid_feedback.actual.time_from_start.to_sec())
            self.assertAlmostEqual(mid_feedback.actual.time_from_start.to_sec(), 2.5, delta=1)
            rospy.logwarn("time_from_start of error of midpoint feedback: %r"%mid_feedback.error.time_from_start.to_sec())
            self.assertAlmostEqual(mid_feedback.error.time_from_start.to_sec(), 2.5, delta=1)
        # feedback shouldn't be published after motion finishes
        # https://github.com/start-jsk/rtmros_common/pull/1049#issuecomment-403780615
        fb_id1 = id(self.feedback)
        rospy.sleep(0.5)
        fb_id2 = id(self.feedback)
        rospy.logwarn("object id of feedback after motion: %r, 0.5sec after: %r"%(fb_id1, fb_id2))
        self.assertEqual(fb_id1, fb_id2)

    def test_joint_trajectory_command(self):
        try:
            self.listener.waitForTransform('/BASE_LINK', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /BASE_LINK to /J7_LINK")
        (trans1,rot1) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans1,rot1))

        self.send_joint_angles([10,20,30,40,50,60,70])

        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertNotAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

    def test_joint_states_velocity(self):
        cmd_angles = [20,20,20,20,20,20,20]
        self.send_joint_angles(cmd_angles, wait=False)
        rospy.sleep(rospy.Duration(2.5))
        jt_st = rospy.wait_for_message('/joint_states', JointState)
        rospy.sleep(rospy.Duration(2.5))
        # When acceleration and deceleration are constant and take the same time,
        # (max velocity) = 2 * (target joint angle) / (whole execution time)
        pred_vel = [2 * (x * math.pi / 180.0) / 5.0 for x in cmd_angles]
        # get velocity of joints except hand
        real_vel = [v for i, v in enumerate(jt_st.velocity) if 'HAND' not in jt_st.name[i]]
        rospy.logwarn("real velocity: %r"%(array(real_vel)))
        rospy.logwarn("difference from predicted velocity: %r"%(linalg.norm(array(real_vel)-array(pred_vel))))
        self.assertAlmostEqual(linalg.norm(array(real_vel)-array(pred_vel)), 0, delta=0.1)


# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestPA10Robot, sys.argv)
