#! /usr/bin/env python

import sys
sys.argv += ['-ORBInitRef','NameService=corbaloc:iiop:hiro014:2809/NameService']
from rtm import *
from OpenHRP import *
# import org.omg.CORBA.DoubleHolder # ??

# utility from bodyinfo.py
def anglesFromDistance(gripDist):

    safetyMargin = 3

    l1 = 33
    l2 = 41.9
    l3 = 30
    l4 = l2 - safetyMargin*2
    l5 = 19

    #if gripDist < 0.0 or gripDist > (l1+l4)*2:
    if gripDist < 0.0 or gripDist > (l2+l5 - safetyMargin)*2:
        return None

    xPos   = gripDist/2.0 + safetyMargin
    #print 'xPos =', xPos
    a2Pos  = xPos - l5
    #print 'a2Pos =', a2Pos
    a1radH = math.acos(a2Pos/l2)
    #print 'a1radH = ', a1radH
    a1rad  = math.pi/2.0 - a1radH
    a2rad  = -a1rad
    #dEnd   = l2 * math.cos(a1rad) + l3

    return a1rad, a2rad, -a1rad, -a2rad #, dEnd


def createComps():
    global grsp_svc, seq_svc

    grsp = findRTC('grsp',rootnc)
    grsp_obj = grsp.service('service0')
    grsp_svc = grsp_obj._narrow(GrasperService)

    # seq = findRTC('seq',rootnc)
    # seq_obj = seq.service('service0')
    # seq_svc = seq_obj._narrow(SequencePlayerService)

#
# ROS actionlib server
#
import roslib; roslib.load_manifest('hironx_ros_bridge')
import rospy

import actionlib
from pr2_controllers_msgs.msg import *

class HiroGripperCommandAction(object):
  # create messages that are used to publish feedback/result
  _feedback = Pr2GripperCommandFeedback()
  _result   = Pr2GripperCommandResult()

  def __init__(self, name, hand):
      self._hand = hand
      self._action_name = name
      self._as = actionlib.SimpleActionServer(self._action_name, Pr2GripperCommandAction, execute_cb=self.execute_cb)
      self._as.start()

  def execute_cb(self, goal):
      success = True

      pos = goal.position # target position of gripper
      effort = goal.max_effort

      # call Hiro RTC command
      if self._hand == 'RHAND' or self._hand == 'LHAND':
          # ttm = org.omg.CORBA.DoubleHolder()
          # grsp_svc.setJointAngles(self._hand, anglesFromDistance(pos), 3.0, ttm)
          # seq_svc.setJointAngles(anglesFromDistance(pos), 3.0, ttm)
          rospy.logdebug('move grippr %s' % self._hand)
      else:
          rospy.logwarn('%s is invalid hand type' % self._hand)

      if success:
          self._result.position = pos
          self._result.effort = effort
          self._result.stalled = False
          self._result.reached_goal = True
          rospy.loginfo('%s: Succeeded' % self._action_name)
          self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('hiro_gripper_action')
    hand_name = rospy.get_param('hand_name','')
    if not (hand_name in ['LHAND', 'RHAND']):
        rospy.logerror("hand_name %s is not LHAND or RHAND, exit..." % hand_name)
        exit(0)
    HiroGripperCommandAction(rospy.get_name(), hand_name)
    rospy.spin()
