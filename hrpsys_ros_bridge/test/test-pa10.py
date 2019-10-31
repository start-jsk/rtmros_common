#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test_pa10'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import argparse,unittest,rostest, time, sys, math, os
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

    def send_joint_angles(self,angles):
        pub = rospy.Publisher("/fullbody_controller/command", JointTrajectory)
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in angles ]
        point.time_from_start = rospy.Duration(5.0)
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time().now()
        msg.joint_names = ["J1","J2","J3","J4","J5","J6","J7"]
        msg.points = [point]
        pub.publish(msg)
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
        larm.send_goal(goal)
        larm.wait_for_result()
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertNotAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

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

    # https://github.com/start-jsk/rtmros_common/pull/765#issuecomment-120208947
    def test_jta_cancel_goal(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_jta_cancel_goal(larm, JointTrajectoryGoal())

    def test_fjta_cancel_goal(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_jta_cancel_goal(larm, FollowJointTrajectoryGoal())

    def impl_test_jta_cancel_goal(self, larm, goal):
        larm.wait_for_server()

        try:
            self.listener.waitForTransform('/BASE_LINK', '/J7_LINK', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /BASE_LINK to /J7_LINK")
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
        rospy.sleep(2.5)
        (trans1,rot1) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans1,rot1))
        larm.cancel_goal()
        rospy.sleep(2.5)
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        larm.wait_for_result()
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

    # https://github.com/start-jsk/rtmros_common/pull/765#issuecomment-392741195
    def test_jta_overwrite_goal(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_jta_overwrite_goal(larm, JointTrajectoryGoal())

    def test_fjta_overwrite_goal(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/fullbody_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_jta_overwrite_goal(larm, FollowJointTrajectoryGoal())

    def impl_test_jta_overwrite_goal(self, larm, goal):
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
        # send first goal
        larm.send_goal(goal)
        rospy.sleep(2.5)
        goal.trajectory.header.stamp = rospy.get_rostime()
        point.positions = [0,0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(2.5)
        del goal.trajectory.points[:]
        goal.trajectory.points.append(point)
        # send second goal
        larm.send_goal(goal)
        jt_pos1 = rospy.wait_for_message('/joint_states', JointState)
        rospy.logwarn("joint position just after sending second goal: %r"%(array(jt_pos1.position)))
        rospy.sleep(0.5)
        jt_pos2 = rospy.wait_for_message('/joint_states', JointState)
        rospy.logwarn("joint position 0.5 sec after sending second goal: %r"%(array(jt_pos2.position)))
        rospy.sleep(2)
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        larm.wait_for_result()
        rospy.logwarn("difference between two joint positions %r"%(array(jt_pos2.position)-array(jt_pos1.position)))
        # if robot suddenly stops when goal is overwritten, joint position immediately starts to decrease (heading for new goal (zero)).
        # joint_states includes unchanged hand joints, so using assertGreaterEqual instead of assertGreater is necessary.
        for x in array(jt_pos2.position)-array(jt_pos1.position):
            self.assertGreaterEqual(x, 0)
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestPA10Robot, sys.argv)
