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
        goal.trajectory.joint_names = ["J1","J2","J3","J4","J5","J6","J7"]

        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        # cancel one point trajectory (:angle-vector)
        point1.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point1.time_from_start = rospy.Duration(5.0)
        self.jta_cancel_goal_template(larm, goal, [point1])

        # cancel two points trajectory (:angle-vector-sequence)
        point1.positions = [ x * math.pi / 180.0 for x in [5,10,15,20,25,30,35] ]
        point1.time_from_start = rospy.Duration(2.5)
        point2.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point2.time_from_start = rospy.Duration(5.0)
        self.jta_cancel_goal_template(larm, goal, [point1, point2])

    def jta_cancel_goal_template(self, larm, goal, points):
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.points = points
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
        goal.trajectory.joint_names = ["J1","J2","J3","J4","J5","J6","J7"]

        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        point3 = JointTrajectoryPoint()
        point4 = JointTrajectoryPoint()
        # one point trajectory (:angle-vector) -> one point trajectory (:angle-vector)
        rospy.logwarn("one point trajectory -> one point trajectory")
        ## first point
        point1.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point1.time_from_start = rospy.Duration(5.0)
        ## second point
        point2.positions = [0,0,0,0,0,0,0]
        point2.time_from_start = rospy.Duration(2.5)
        ## execution
        self.jta_overwrite_goal_template(larm, goal, trans1, [point1], [point2])

        # one point trajectory (:angle-vector) -> two points trajectory (:angle-vector-sequence)
        rospy.logwarn("one point trajectory -> two points trajectory")
        ## first point
        point1.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point1.time_from_start = rospy.Duration(5.0)
        ## second point
        point2.positions = [ x * math.pi / 180.0 for x in [2.5,5,7.5,10,12.5,15,17.5] ]
        point2.time_from_start = rospy.Duration(1.25)
        ## third point
        point3.positions = [0,0,0,0,0,0,0]
        point3.time_from_start = rospy.Duration(2.5)
        ## execution
        self.jta_overwrite_goal_template(larm, goal, trans1, [point1], [point2, point3])

        # two points trajectory (:angle-vector-sequence) -> one point trajectory (:angle-vector)
        rospy.logwarn("two points trajectory -> one point trajectory")
        ## first point
        point1.positions = [ x * math.pi / 180.0 for x in [5,10,15,20,25,30,35] ]
        point1.time_from_start = rospy.Duration(2.5)
        ## second point
        point2.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point2.time_from_start = rospy.Duration(5.0)
        ## third point
        point3.positions = [0,0,0,0,0,0,0]
        point3.time_from_start = rospy.Duration(2.5)
        ## execution
        self.jta_overwrite_goal_template(larm, goal, trans1, [point1, point2], [point3])

        # two points trajectory (:angle-vector-sequence) -> two points trajectory (:angle-vector-sequence)
        rospy.logwarn("two points trajectory -> two points trajectory")
        ## first point
        point1.positions = [ x * math.pi / 180.0 for x in [5,10,15,20,25,30,35] ]
        point1.time_from_start = rospy.Duration(2.5)
        ## second point
        point2.positions = [ x * math.pi / 180.0 for x in [10,20,30,40,50,60,70] ]
        point2.time_from_start = rospy.Duration(5.0)
        ## third point
        point3.positions = [ x * math.pi / 180.0 for x in [2.5,5,7.5,10,12.5,15,17.5] ]
        point3.time_from_start = rospy.Duration(1.25)
        ## fourth point
        point4.positions = [0,0,0,0,0,0,0]
        point4.time_from_start = rospy.Duration(2.5)
        ## execution
        self.jta_overwrite_goal_template(larm, goal, trans1, [point1, point2], [point3, point4])

    def jta_overwrite_goal_template(self, larm, goal, trans1, points1, points2):
        # send first goal
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.points = points1
        larm.send_goal(goal)
        rospy.sleep(2.5)
        # send second goal
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.points = points2
        larm.send_goal(goal)
        # check robot motion after it is overwritten
        jt_pos1 = rospy.wait_for_message('/joint_states', JointState)
        rospy.logwarn("joint position just after sending second goal: %r"%(array(jt_pos1.position)))
        rospy.sleep(0.2)
        jt_pos2 = rospy.wait_for_message('/joint_states', JointState)
        rospy.logwarn("joint position 0.2 sec after sending second goal: %r"%(array(jt_pos2.position)))
        rospy.sleep(2.3)
        (trans2,rot2) = self.listener.lookupTransform('/BASE_LINK', '/J7_LINK', rospy.Time(0))
        rospy.logwarn("tf_echo /BASE_LINK /J7_LINK %r %r"%(trans2,rot2))
        larm.wait_for_result()
        rospy.logwarn("difference between two joint positions %r"%(array(jt_pos2.position)-array(jt_pos1.position)))
        ## if robot suddenly stops when goal is overwritten, joint position immediately starts to decrease (heading for new goal).
        ## joint_states includes unchanged hand joints, so using assertGreaterEqual instead of assertGreater is necessary.
        for x in array(jt_pos2.position)-array(jt_pos1.position):
            self.assertGreaterEqual(x, 0)
        rospy.logwarn("difference between two /J7_LINK %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)


# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestPA10Robot, sys.argv)
