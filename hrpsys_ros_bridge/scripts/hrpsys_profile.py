#!/usr/bin/env python

try: # catkin does not requires load_manifest
    import hrpsys
except:
    import roslib; roslib.load_manifest("hrpsys_ros_bridge")

import OpenRTM_aist.RTM_IDL # for catkin

import rospy
from diagnostic_msgs.msg import *

import os
import hrpsys
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

def rtc_init () :
    global ms, rh, eps

    initCORBA()
    ms = rtm.findRTCmanager(rtm.nshost)
    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager(rtm.nshost)
        print "[hrpsys_profile.py] wait for RTCmanager : ",ms

def hrpsys_profile() :
    global ms, rh, eps

    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = rospy.Time.now()

    components = ms.get_components()
    for component in components :
        component_name = component.name()
        component_rtc = findRTC(component_name)
        eps = narrow(component_rtc.owned_ecs[0], "ExecutionProfileService")

        if not eps :
            continue

        total_prof = eps.getProfile()

        if total_prof.count > 0 :
            status = DiagnosticStatus(name = 'hrpEC Profile ('+component_name+')', level = DiagnosticStatus.OK)
            status.message = "Running : Average Period : %7.5f, Max Period : %7.5f" % (total_prof.avg_period*1000, total_prof.max_period*1000);
            status.values.append(KeyValue(key = "Max Period", value = str(total_prof.max_period*1000)))
            status.values.append(KeyValue(key = "Min Period", value = str(total_prof.min_period*1000)))
            status.values.append(KeyValue(key = "Average Period", value = str(total_prof.avg_period*1000)))
            status.values.append(KeyValue(key = "Max Process", value = str(total_prof.max_process*1000)))
            status.values.append(KeyValue(key = "Count", value = str(total_prof.count)))
            status.values.append(KeyValue(key = "Timeover", value = str(total_prof.timeover)))
            if ( total_prof.timeover > 0 ) :
                status.level   = DiagnosticStatus.WARN

            diagnostic.status.append(status)

        for c in components :
            try:
                prof = eps.getComponentProfile(c.ref)
                status = DiagnosticStatus(name =  'hrpEC Profile (RTC: ' + c.name() + ')', level = DiagnosticStatus.OK)
                status.message = "Running : Average Process : %7.5f, Max Process : %7.5f" % (prof.avg_process*1000, prof.max_process*1000);
                status.values.append(KeyValue(key = "Max Process", value = str(prof.max_process*1000)))
                status.values.append(KeyValue(key = "Avg Process", value = str(prof.avg_process*1000)))
                status.values.append(KeyValue(key = "Count", value = str(prof.count)))
                diagnostic.status.append(status)
            except :
                True

        if ( total_prof.count > 100000 ) :
            rospy.loginfo("eps.resetProfile()")
            eps.resetProfile()
            
    pub.publish(diagnostic)


if __name__ == '__main__':
    try:
        rtc_init()

        rospy.init_node('hrpsys_profile_diagnostics')
        pub = rospy.Publisher('diagnostics', DiagnosticArray)

        r = rospy.Rate(1) # 10hz

        while not rospy.is_shutdown():
            try :
                hrpsys_profile()
            except (omniORB.CORBA.TRANSIENT, omniORB.CORBA.BAD_PARAM, omniORB.CORBA.COMM_FAILURE), e :
                print "[hrpsys_profile.py] catch exception", e
                rtc_init()
            except Exception, e:
                pass

            r.sleep()
        
    except rospy.ROSInterruptException: pass





