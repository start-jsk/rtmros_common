#!/usr/bin/env python

import sys,time

from omniORB import CORBA
import CosNaming

import roslib
roslib.load_manifest('openhrp3')

from openhrp3 import OnlineViewer_idl

rootnc = None
def initCORBA(argv):
    global rootnc
    while rootnc == None:
        try:
            orb = CORBA.ORB_init(argv)
            nameserver = orb.string_to_object("corbaloc::localhost/NameService")
            rootnc = nameserver._narrow(CosNaming.NamingContext)
        except:
            time.sleep(1)
    return None

def findObject(name, kind):
    global rootnc
    nc = CosNaming.NameComponent(name,kind)
    path= [nc]
    try:
        obj = rootnc.resolve(path)
        return obj
    except CosNaming.NamingContext.NotFound, ex:
        return Null

def waitOnlineViewer():
    initCORBA(sys.argv)
    ret = False
    while not ret :
        time.sleep(1)
        print "wait for online viewer..."
        try:
            ret = filter(lambda x: findObject("OnlineViewer", "").getPosture(x)[0], ["floor", "longfloor", "box", "pa10"])
        except:
            print "error on findObject onlineViewer"
    return ret



