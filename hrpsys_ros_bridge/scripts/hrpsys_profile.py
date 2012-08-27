#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys_ros_bridge")

import rospy
from diagnostic_msgs.msg import *

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

def rtc_init () :
    global ms, rh, eps

    initCORBA()

    ms = None;
    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager()
        print "[hrpsys_profile.py] wait for RTCmanager : ",ms

    #
    #for c in [ findRTC(x.name()) for x in ms.get_components() ] :
    #    eps = narrow(c.owned_ecs[0], "ExecutionProfileService")
    #    print c.name(), c.owned_ecs[0], eps

    rh = findRTC('RobotHardware0')
    eps = narrow(rh.owned_ecs[0], "ExecutionProfileService")

def hrpsys_profile() :
    global ms, rh, eps

    rospy.init_node('hrpsys_profile_diagnostics')
    pub = rospy.Publisher('diagnostics', DiagnosticArray)

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = rospy.Time.now()

        total_prof = eps.getProfile()
        components = [ findRTC(x.name()) for x in ms.get_components() ]

        status = DiagnosticStatus(name = 'hrpEC Profile', level = DiagnosticStatus.OK, message = "Running")
        status.values.append(KeyValue(key = "Max Period", value = str(total_prof.max_period*1000)))
        status.values.append(KeyValue(key = "Min Period", value = str(total_prof.min_period*1000)))
        status.values.append(KeyValue(key = "Average Period", value = str(total_prof.avg_period*1000)))
        status.values.append(KeyValue(key = "Max Process", value = str(total_prof.max_process*1000)))
        status.values.append(KeyValue(key = "Count", value = str(total_prof.count)))
        status.values.append(KeyValue(key = "Timeover", value = str(total_prof.timeover)))
        if ( total_prof.timeover > 0 ) :
            status.level   = DiagnosticStatus.WARN
            status.message = "Max Period : %7.3f, Average Period : %7.3f" % (total_prof.max_period*1000, total_prof.avg_period*1000);

        diagnostic.status.append(status)

        for c in components :
            try :
                prof = eps.getComponentProfile(c.ref)
                #print c.name(), prof.count, prof.max_process*1000, prof.avg_process*1000

                status = DiagnosticStatus(name =  c.name()+' hrpEC Profile', level = DiagnosticStatus.OK, message = "Running")
                status.values.append(KeyValue(key = "Max Process", value = str(prof.max_process*1000)))
                status.values.append(KeyValue(key = "Avg Process", value = str(prof.avg_process*1000)))
                status.values.append(KeyValue(key = "Count", value = str(prof.count)))
                diagnostic.status.append(status)
            except :
                #print c.name()
                None

            
            if ( total_prof.count > 100000 ) :
                print "eps.resetProfile()"
                eps.resetProfile() 
            
        pub.publish(diagnostic)
        r.sleep()

if __name__ == '__main__':
    try:
        rtc_init()
        hrpsys_profile()
    except rospy.ROSInterruptException: pass





