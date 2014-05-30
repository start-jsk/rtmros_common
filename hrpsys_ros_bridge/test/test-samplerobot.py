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
from nav_msgs.msg import Odometry
from hrpsys_ros_bridge.srv import OpenHRP_SequencePlayerService_loadPattern, OpenHRP_SequencePlayerService_waitInterpolation

import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TestSampleRobot(unittest.TestCase):
    listener = None
    lfsensor = None
    rfsensor = None
    odom = None
    def lfsensor_cb(self, sensor):
        self.lfsensor = sensor

    def rfsensor_cb(self, sensor):
        self.rfsensor = sensor
    def odom_cb(self, odom):
        self.odom = odom
    def setUp(self):
        rospy.init_node('TestSampleRobot')
        rospy.Subscriber('/lfsensor', WrenchStamped, self.lfsensor_cb)
        rospy.Subscriber('/rfsensor', WrenchStamped, self.rfsensor_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
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
        tm = 1.0
        ret = load_pattern(basename = basename, tm = tm)
        t1 = rospy.get_time()
        rospy.logwarn("loadPattern = %r %r"%(basename, tm))
        ret = wait_interpolation()
        t2 = rospy.get_time()
        rospy.logwarn("waitInterpolation %f"%(t2-t1))
        self.assertAlmostEqual(t2-t1, 20, delta=5)
        #self.assertNotAlmostEqual(trans[1],0,2)

    # send joint angles
    def test_joint_angles(self):
        larm = actionlib.SimpleActionClient("/larm_controller/joint_trajectory_action", JointTrajectoryAction)
        larm.wait_for_server()

        try:
            self.listener.waitForTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(), rospy.Duration(120))
        except tf.Exception:
            self.assertTrue(None, "could not found tf from /WAIST_LINK0 to /LARM_LINK7")
        (trans1,rot1) = self.listener.lookupTransform('/WAIST_LINK0', '/LARM_LINK7', rospy.Time(0))
        rospy.logwarn("tf_echo /WAIST_LINK0 /LARM_LINK7 %r %r"%(trans1,rot1))
        goal = JointTrajectoryGoal()
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

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestSampleRobot, sys.argv)
