#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test_samplerobot'

import argparse,unittest,rostest, time, sys, math, os
from numpy import *

import rospy,rospkg, tf
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from hrpsys_ros_bridge.srv import *

import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint

class TestSampleRobot(unittest.TestCase):
    listener = None
    lfsensor = None
    rfsensor = None
    odom = None
    joint_states = None

    def lfsensor_cb(self, sensor):
        self.lfsensor = sensor

    def rfsensor_cb(self, sensor):
        self.rfsensor = sensor
    def odom_cb(self, odom):
        self.odom = odom
    def jointstate_cb(self, joint_states):
        self.joint_states = joint_states

    def setUp(self):
        rospy.init_node('TestSampleRobot')
        rospy.Subscriber('/lfsensor', WrenchStamped, self.lfsensor_cb)
        rospy.Subscriber('/rfsensor', WrenchStamped, self.rfsensor_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/joint_states', JointState, self.jointstate_cb)
        self.listener = tf.TransformListener()

    def test_odom(self): # need to check if map/ is published?
        # wait odom topic
        start_time = rospy.Time.now()
        r = rospy.Rate(1)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 120:
            if self.odom:
                break
            rospy.loginfo("waiting for /odom")
            r.sleep()
        if not self.odom:
            self.assertTrue(False, "no odom topic is available")
        else:
            self.assertTrue(True,"ok")

    def test_odom_tf(self): # echeck if tf/odom is published
        # wait odom topic
        try:
            rospy.loginfo("waiting for /WAIST_LINK0 to /odom")
            self.listener.waitForTransform('/WAIST_LINK0', '/odom', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /odom")
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/odom', rospy.Time(0))

        self.assertTrue(True,"ok")

    def test_force_sensor(self):
        while self.lfsensor == None or self.rfsensor == None:
            time.sleep(1)
            rospy.logwarn("wait for sensor...")
        rospy.logwarn("sensor = %r %r"%(self.lfsensor, self.rfsensor))
        self.assertAlmostEqual(self.lfsensor.wrench.force.z+self.rfsensor.wrench.force.z, 1300, delta=200)

    # send walk motion
    def test_load_pattern(self):
        rospy.wait_for_service('/SequencePlayerServiceROSBridge/loadPattern')
        load_pattern = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/loadPattern', OpenHRP_SequencePlayerService_loadPattern)
        wait_interpolation = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/waitInterpolation', OpenHRP_SequencePlayerService_waitInterpolation)
        basename = rospkg.RosPack().get_path('openhrp3')+'/share/OpenHRP-3.1/sample/controller/SampleController/etc/Sample'
        if os.path.exists(basename + ".pos"):
            tm = 1.0
            ret = load_pattern(basename = basename, tm = tm)
            t1 = rospy.get_time()
            rospy.logwarn("loadPattern = %r %r"%(basename, tm))
            ret = wait_interpolation()
            t2 = rospy.get_time()
            rospy.logwarn("waitInterpolation %f"%(t2-t1))
            self.assertAlmostEqual(t2-t1, 11, delta=2)
            #self.assertNotAlmostEqual(trans[1],0,2)

    def impl_test_joint_angles(self, larm, goal):
        larm.wait_for_server()

        try:
            self.listener.waitForTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /LARM_LINK7")
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans1,rot1))
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("LARM_SHOULDER_P")
        goal.trajectory.joint_names.append("LARM_SHOULDER_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("LARM_ELBOW")
        goal.trajectory.joint_names.append("LARM_WRIST_Y")
        goal.trajectory.joint_names.append("LARM_WRIST_P")

        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [30,30,30,-90,-40,-30] ]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        larm.send_goal(goal)
        larm.wait_for_result()
        (trans2,rot2) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans2,rot2))
        rospy.logwarn("difference between two /LARM_LINK7 %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertNotAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

        ## move less than 1.0 sec
        goal.trajectory.header.stamp = rospy.get_rostime()
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [30,30,30, 0,-40,-30] ]
        point.time_from_start = rospy.Duration(0.9)
        goal.trajectory.points = [point]
        larm.send_goal(goal)
        start_time = rospy.get_rostime()
        larm.wait_for_result()
        stop_time  = rospy.get_rostime()
        rospy.logwarn("working time: %f"%(stop_time - start_time).to_sec())
        self.assertNotAlmostEqual((stop_time - start_time).to_sec(), 0, delta=0.1)

    # send joint angles
    def test_joint_angles(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_joint_angles(larm, JointTrajectoryGoal())

    def test_follow_joint_angles(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_joint_angles(larm, FollowJointTrajectoryGoal())

    def test_joint_angles_duration_0(self): # https://github.com/start-jsk/rtmros_common/issues/1036
        # for >= kinetic
        larm = None
        goal = None
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
            larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
            goal = JointTrajectoryGoal()
        else:
            from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
            larm = actionlib.SimpleActionClient("/larm_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
            goal = FollowJointTrajectoryGoal()

        while self.joint_states == None:
            time.sleep(1)
            rospy.logwarn("wait for joint_states..")

        ## wait for joint_states updates
        rospy.sleep(1)
        current_positions = dict(zip(self.joint_states.name, self.joint_states.position))

        # goal
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("LARM_SHOULDER_P")
        goal.trajectory.joint_names.append("LARM_SHOULDER_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("LARM_ELBOW")
        goal.trajectory.joint_names.append("LARM_WRIST_Y")
        goal.trajectory.joint_names.append("LARM_WRIST_P")

        goal.trajectory.points = []
        ## add current position
        point = JointTrajectoryPoint()
        point.positions = [ current_positions[x] for x in goal.trajectory.joint_names]
        point.time_from_start = rospy.Duration(0.0)
        goal.trajectory.points.append(point)

        ## add target position
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [15,15,15,-45,-20,-15] ]
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)

        ## send goal
        larm.send_goal(goal)
        larm.wait_for_result()

        ## wait for joint_states updates
        rospy.sleep(1)
        current_positions = dict(zip(self.joint_states.name, self.joint_states.position))

        goal.trajectory.points = []
        ## add current position
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [15,15,15,-45,-20,-15] ]
        point.time_from_start = rospy.Duration(0.0)
        goal.trajectory.points.append(point)

        ## add target position
        point = JointTrajectoryPoint()
        point.positions = [ x * math.pi / 180.0 for x in [30,30,30,-90,-40,-30] ]
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)

        ## send goal again.... (# https://github.com/start-jsk/rtmros_common/issues/1036 requires send goal twice...)
        larm.send_goal(goal)
        larm.wait_for_result()

        ## wait for update joint_states
        rospy.sleep(1)
        current_positions = dict(zip(self.joint_states.name, self.joint_states.position))
        goal_angles = [ 180.0 / math.pi * current_positions[x] for x in goal.trajectory.joint_names]
        rospy.logwarn(goal_angles)
        rospy.logwarn("difference between two angles %r %r"%(array([30,30,30,-90,-40,-30])-array(goal_angles),linalg.norm(array([30,30,30,-90,-40,-30])-array(goal_angles))))
        self.assertAlmostEqual(linalg.norm(array([30,30,30,-90,-40,-30])-array(goal_angles)), 0, delta=0.1)

    # https://github.com/start-jsk/rtmros_common/pull/765#issuecomment-120208947
    def test_jta_cancel_goal(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_jta_cancel_goal(larm, JointTrajectoryGoal())

    def test_fjta_cancel_goal(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_jta_cancel_goal(larm, FollowJointTrajectoryGoal())

    def impl_test_jta_cancel_goal(self, larm, goal):
        larm.wait_for_server()

        # initialize
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("LARM_SHOULDER_P")
        goal.trajectory.joint_names.append("LARM_SHOULDER_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("LARM_ELBOW")
        goal.trajectory.joint_names.append("LARM_WRIST_Y")
        goal.trajectory.joint_names.append("LARM_WRIST_P")
        point = JointTrajectoryPoint()
        point.positions = [0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        larm.send_goal(goal)
        larm.wait_for_result()

        try:
            self.listener.waitForTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /LARM_LINK7")
        goal.trajectory.header.stamp = rospy.get_rostime()
        point.positions = [ x * math.pi / 180.0 for x in [20,20,20,20,20,20] ]
        point.time_from_start = rospy.Duration(5.0)
        del goal.trajectory.points[:]
        goal.trajectory.points.append(point)
        larm.send_goal(goal)
        rospy.sleep(2.5)
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans1,rot1))
        larm.cancel_goal()
        rospy.sleep(2.5)
        (trans2,rot2) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans2,rot2))
        larm.wait_for_result()
        rospy.logwarn("difference between two /LARM_LINK7 %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)

    # https://github.com/start-jsk/rtmros_common/pull/765#issuecomment-392741195
    def test_jta_overwrite_goal(self):
        # for < kinetic
        if os.environ['ROS_DISTRO'] >= 'kinetic' :
            return True
        from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.impl_test_jta_overwrite_goal(larm, JointTrajectoryGoal())

    def test_fjta_overwrite_goal(self):
        # for >= kinetic
        if os.environ['ROS_DISTRO'] < 'kinetic' :
            return True
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        larm = actionlib.SimpleActionClient("/larm_controller/follow_joint_trajectory_action", FollowJointTrajectoryAction)
        self.impl_test_jta_overwrite_goal(larm, FollowJointTrajectoryGoal())

    def impl_test_jta_overwrite_goal(self, larm, goal):
        larm.wait_for_server()

        # initialize
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names.append("LARM_SHOULDER_P")
        goal.trajectory.joint_names.append("LARM_SHOULDER_R")
        goal.trajectory.joint_names.append("LARM_SHOULDER_Y")
        goal.trajectory.joint_names.append("LARM_ELBOW")
        goal.trajectory.joint_names.append("LARM_WRIST_Y")
        goal.trajectory.joint_names.append("LARM_WRIST_P")
        point = JointTrajectoryPoint()
        point.positions = [0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        larm.send_goal(goal)
        larm.wait_for_result()

        try:
            self.listener.waitForTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /LARM_LINK7")
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans1,rot1))
        goal.trajectory.header.stamp = rospy.get_rostime()
        point.positions = [ x * math.pi / 180.0 for x in [20,20,20,20,20,20] ]
        point.time_from_start = rospy.Duration(5.0)
        del goal.trajectory.points[:]
        goal.trajectory.points.append(point)
        # send first goal
        larm.send_goal(goal)
        rospy.sleep(2.5)
        goal.trajectory.header.stamp = rospy.get_rostime()
        point.positions = [0,0,0,0,0,0]
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
        (trans2,rot2) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans2,rot2))
        larm.wait_for_result()
        rospy.logwarn("difference between two joint positions %r"%(array(jt_pos2.position)-array(jt_pos1.position)))
        # if robot suddenly stops when goal is overwritten, joint position immediately starts to decrease (heading for new goal (zero)).
        # joint_states includes unchanged hand joints, so using assertGreaterEqual instead of assertGreater is necessary.
        for x in array(jt_pos2.position)-array(jt_pos1.position):
            self.assertGreaterEqual(x, 0)
        rospy.logwarn("difference between two /LARM_LINK7 %r %r"%(array(trans1)-array(trans2),linalg.norm(array(trans1)-array(trans2))))
        self.assertAlmostEqual(linalg.norm(array(trans1)-array(trans2)), 0, delta=0.1)


#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestSampleRobot, sys.argv)
