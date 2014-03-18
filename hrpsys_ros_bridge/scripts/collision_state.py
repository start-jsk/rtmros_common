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
import numpy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import tf
from tf.transformations import *

def rtc_init () :
    global ms, co, co_svc, root_link_name, root_link_offset

    initCORBA()
    ms = rtm.findRTCmanager(rtm.nshost)
    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager(rtm.nshost)
        rospy.loginfo("[collision_state.py] wait for RTCmanager : ",ms)

    co = None
    count = 0
    while co  == None and count < 10:
        co = rtm.findRTC(rospy.get_param('~comp_name', 'co'))
        if co :
            break
        rospy.logwarn("Could not found CollisionDetector, waiting...")
        time.sleep(1)
        count += 1
    if co == None:
        rospy.logerr("Could not found CollisionDetector, exiting...")
        exit(0)
    co_svc = narrow(co.service("service0"), "CollisionDetectorService")

    if modelfile:
        #import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader_idl._0_OpenHRP__POA.ModelLoader)
        rospy.loginfo("  bodyinfo URL = file://"+modelfile)
        body_info = mdlldr.getBodyInfo("file://"+modelfile)
        root_link_name = body_info._get_links()[0].name

        root_link_offset = inverse_matrix(concatenate_matrices(translation_matrix(body_info._get_links()[0].translation),
                                                rotation_matrix(body_info._get_links()[0].rotation[3],
                                                                body_info._get_links()[0].rotation[0:3])))
    else:
        root_link_name = "WAIST"
        root_link_offset = identity_matrix()

        rospy.loginfo("ssetup collision_state with " + root_link_name + " " + root_link_offset)


def collision_state() :
    global ms, co_svc, root_link_name, root_link_offset, last_collision_status

    diagnostic = DiagnosticArray()
    now = rospy.Time.now()
    diagnostic.header.stamp = now

    collision_status = co_svc.getCollisionStatus()

    if (now - last_collision_status > rospy.Duration(5.0)):
        num_collision_pairs = len(collision_status[1].lines)
        rospy.loginfo("check %d collision status, %f Hz",num_collision_pairs, num_collision_pairs/(now - last_collision_status).to_sec())
        last_collision_status = now

    # check if ther are collision
    status = DiagnosticStatus(name = 'CollisionDetector', level = DiagnosticStatus.OK, message = "Ok")

    #if any(a): # this calls omniORB.any
    for collide in collision_status[1].collide:
        if collide:
            status.level   = DiagnosticStatus.ERROR
            status.message = "Robots is in collision mode"

    status.values.append(KeyValue(key = "Time", value = str(collision_status[1].time)))
    status.values.append(KeyValue(key = "Computation Time", value = str(collision_status[1].computation_time)))
    status.values.append(KeyValue(key = "Safe Posture", value = str(collision_status[1].safe_posture)))
    status.values.append(KeyValue(key = "Recover Time", value = str(collision_status[1].recover_time)))
    status.values.append(KeyValue(key = "Loop for check", value = str(collision_status[1].loop_for_check)))

    frame_id = root_link_name # root id
    markerArray = MarkerArray()
    for line in collision_status[1].lines:
        p1 = Point(*(numpy.dot(root_link_offset[3,3], line[0]) + root_link_offset[0:3,3]))
        p2 = Point(*(numpy.dot(root_link_offset[3,3], line[1]) + root_link_offset[0:3,3]))

        sphere_color = ColorRGBA(0,1,0,1)
        line_width = 0.01
        line_length = numpy.linalg.norm(numpy.array((p1.x,p1.y,p1.z))-numpy.array((p2.x,p2.y,p2.z)))
        sphere_scale = 0.02
        # color changes between 0.15(green) -> 0.05(red), under 0.05, it always red
        if (line_length < 0.15) :
            if ( line_length < 0.05) :
                sphere_color = ColorRGBA(1, 0, 0, 1)
            else:
                ratio = 1.0 - (line_length-0.05)*10 # 0.0 (0.15) -> 1.0 ( 0.05)
                sphere_scale = 0.02+ ratio*0.08
                sphere_color = ColorRGBA(ratio, 1-ratio,0,1)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.color = sphere_color
        marker.points = [p1, p2]
        marker.scale.x = line_width
        markerArray.markers.append(marker)

        sphere_scale = Vector3(sphere_scale, sphere_scale, sphere_scale)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale = sphere_scale
        marker.color = sphere_color
        marker.pose.orientation.w = 1.0
        marker.pose.position = p1
        markerArray.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale = sphere_scale
        marker.color = sphere_color
        marker.pose.orientation.w = 1.0
        marker.pose.position = p2
        markerArray.markers.append(marker)


    id = 0
    for m in markerArray.markers:
        m.lifetime = rospy.Duration(1.0)
        m.id = id
        id += 1

    pub_collision.publish(markerArray)
    diagnostic.status.append(status)
    pub_diagnostics.publish(diagnostic)


modelfile = None
if __name__ == '__main__':
    if len(sys.argv) > 1 :
        modelfile = sys.argv[1]

    try:
        rospy.init_node('collision_state_diagnostics')
        pub_diagnostics = rospy.Publisher('diagnostics', DiagnosticArray)
        pub_collision = rospy.Publisher('collision_detector_marker_array', MarkerArray)

        r = rospy.Rate(50)

        rtc_init()

        last_collision_status = rospy.Time.now()

        while not rospy.is_shutdown():
            try :
                collision_state()
            except (CORBA.OBJECT_NOT_EXIST, omniORB.CORBA.TRANSIENT, omniORB.CORBA.BAD_PARAM, omniORB.CORBA.COMM_FAILURE), e :
                print "[collision_state.py] catch exception, restart rtc_init", e
                time.sleep(2)
                rtc_init()
            except Exception as e:
                print "[collision_state.py] catch exception", e
            r.sleep()

    except rospy.ROSInterruptException: pass





