#!/usr/bin/env python

PKG = 'rtmbuild_test'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import unittest, os, sys, time, rospy

from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

import OpenRTM_aist
import RTC, OpenRTM, SDOPackage, RTM
from RTC import *

rootnc = None
orb    = None
nshost = 'localhost'
nsport = 9999

def initCORBA() :
    global rootnc, orb
    print "configuration ORB with ", nshost, ":", nsport
    os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:{0}:{1}/NameService'.format(nshost,nsport)

    try:
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
        nameserver = orb.resolve_initial_references("NameService");
        rootnc = nameserver._narrow(CosNaming.NamingContext)
    except omniORB.CORBA.ORB.InvalidName, e:
        sys.exit('[ERROR] Invalide Name (hostname={0}).\n'.format(nshost) +
                 'Make sure the hostname is correct.\n' + str(e))
    except omniORB.CORBA.TRANSIENT, e:
        sys.exit('[ERROR] Connection Failed with the Nameserver (hostname={0} port={1}).\n'.format(nshost, nsport) +
                 'Make sure the hostname is correct and the Nameserver is running.\n' + str(e))
    except Exception as e:
        print str(e)

    return None

def findObject(name) :
    global rootnc
    nc = None
    while nc == None :
        try:
            nc = rootnc.resolve([CosNaming.NameComponent(name, "rtc")])
        except:
            print >>sys.stderr, "Waiting for %s ..."%name
            time.sleep(1)
            continue
        time.sleep(1)
    print >>sys.stderr, "Found for %s (%r)"%(name,nc)
    return nc

class TestCompileIDL(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        global orb

        # run opnrtm
        print >>sys.stderr, "initCORBA"
        initCORBA()

        # wait for MyServiceROSBridge
        print >>sys.stderr, "wait for MyServiceROSBridge"
        bridge   = findObject("MyServiceROSBridge")._narrow(RTC.RTObject)
        print >>sys.stderr, "wait for MyServiceProvider"
        provider = findObject("MyServiceProvider0")._narrow(RTC.RTObject)
        # connect
        print >>sys.stderr, "connect components"
        inP = provider.get_ports()[0]
        outP = bridge.get_ports()[0]
        con_prof = RTC.ConnectorProfile("connector0", "", [outP, inP], [])
        inP.connect(con_prof)
        # activate
        print >>sys.stderr, "activate components"
        bridge.get_owned_contexts()[0].activate_component(bridge)
        provider.get_owned_contexts()[0].activate_component(provider)

        orb.shutdown(1)

        # run ros
        rospy.init_node('test_compile_idl')


    def testEcho(self):
        sys.path.append("/tmp/test_compile_idl/devel/lib/python2.7/dist-packages")
        print sys.path
        import rtmbuild_test.srv
        rospy.logwarn("wait for service")
        rospy.wait_for_service("/MyServiceROSBridge/echo")
        echo = rospy.ServiceProxy("/MyServiceROSBridge/echo", rtmbuild_test.srv.SimpleService_MyService_echo)
        req = rtmbuild_test.srv.SimpleService_MyService_echo
        msg = "this is test data 123"
        rospy.logwarn("send request > " + msg)
        res = echo(msg)
        rospy.logwarn("check return < " + res.operation_return)
        self.assertTrue(res.operation_return == msg, "SimpleService.echo returns incorrect string")

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_myservice_rosbridge', TestCompileIDL)
