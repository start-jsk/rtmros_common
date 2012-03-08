#!/usr/bin/env python

# Sample script for HRP4C + SequencePlayerServiceROSBridgeComp
# This will be copied to openrtm_ros_bridge package

import roslib; roslib.load_manifest('hrpsys_ros_bridge')
import sys

import rospy
from hrpsys.srv import *

# walkdata=`rospack find hrpsys`/share/hrpsys/samples/HRP-4C/data/walk2m
walkdata = roslib.rospack.rospackexec(['find','hrpsys']) + '/share/hrpsys/samples/HRP-4C/data/walk2m'
tms = 1.0

if __name__ == "__main__":
    rospy.wait_for_service('loadPattern')
    try:
        load_pattern = rospy.ServiceProxy('loadPattern', OpenHRP_SequencePlayerService_loadPattern)
        resp1 = load_pattern(walkdata, tms)
        exit
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

