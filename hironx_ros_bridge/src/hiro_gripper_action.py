#! /usr/bin/env python

import roslib; roslib.load_manifest('hironx_ros_bridge')
import rospy

import site
site.addsitedir(roslib.rospack.rospackexec(['find','hironx_ros_bridge']) + '/src/hironx_ros_bridge')
site.addsitedir(roslib.rospack.rospackexec(['find','hrpsys']) + '/share/hrpsys/python')

import os, sys
nameserver = os.environ['RTCTREE_NAMESERVERS']
sys.argv += ['-ORBInitRef','NameService=corbaloc:iiop:'+nameserver+':2809/NameService']

import rtm
from OpenHRP import GrasperService
import math

#
# ROS actionlib server
#

import actionlib
from pr2_controllers_msgs.msg import *

class HiroGripperCommandAction(object):
    def __init__(self, name, hand):
        self.createComps()
        # create messages that are used to publish feedback/result
        self._feedback = Pr2GripperCommandFeedback()
        self._result   = Pr2GripperCommandResult()
        self._hand = hand
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Pr2GripperCommandAction, execute_cb=self.execute_cb)
        self._as.start()

    # utility from bodyinfo.py
    def anglesFromDistance(self, gripDist):
        # gripDist is [mm]
        safetyMargin = 3

        l1 = 33
        l2 = 41.9
        l3 = 30
        l4 = l2 - safetyMargin*2
        l5 = 19

        # if gripDist < 0.0 or gripDist > (l1+l4)*2:
        if gripDist < 0.0 or gripDist > (l2+l5 - safetyMargin)*2:
            return None

        xPos   = gripDist/2.0 + safetyMargin
        # print 'xPos =', xPos
        a2Pos  = xPos - l5
        # print 'a2Pos =', a2Pos
        a1radH = math.acos(a2Pos/l2)
        # print 'a1radH = ', a1radH
        a1rad  = math.pi/2.0 - a1radH
        a2rad  = -a1rad
        # dEnd   = l2 * math.cos(a1rad) + l3

        return a1rad, a2rad, -a1rad, -a2rad #, dEnd

    def createComps(self):
        self.grsp_svc = None
        grsp = rtm.findRTC('grsp')
        if not grsp:
            return None
        grsp_obj = grsp.service('service0')
        if not grsp_obj:
            return None
        self.grsp_svc = grsp_obj._narrow(GrasperService)

    def execute_cb(self, goal):
        success = False

        pos = goal.command.position * 1000 # target position of gripper [mm]
        effort = goal.command.max_effort   # ignore

        # call Hiro RTC command
        if not self.grsp_svc:
            rospy.logwarn('grasp_svc is None')
        elif self._hand == 'RHAND' or self._hand == 'LHAND':
            angles = self.anglesFromDistance(pos)
            status, ttm = self.grsp_svc.setJointAngles(self._hand, angles, 1.0)
            rospy.logdebug('move %s gripper (%s[mm])' % (self._hand, pos))
            success = True
        else:
            rospy.logwarn('%s is invalid hand type' % self._hand)

        # set actionlib result
        self._result.position = pos
        self._result.effort = effort

        if success:
            self._result.stalled = False
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)
        else:
            self._result.stalled = False
            self._result.reached_goal = False
            self._as.set_aborted(self._result)
        return success

if __name__ == '__main__':
    rospy.init_node('hiro_gripper_action')
    hand_name = rospy.get_param('~hand_name','')
    if not (hand_name in ['LHAND', 'RHAND']):
        rospy.logerr("hand_name %s is not LHAND or RHAND, exit..." % hand_name)
        exit(0)
    HiroGripperCommandAction("gripper_action", hand_name)
    rospy.spin()
