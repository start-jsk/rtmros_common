#!/usr/bin/env python

import sys

from omniORB import CORBA
import CosNaming

import roslib
roslib.load_manifest('openhrp3')
from openhrp3 import OnlineViewer_idl


rootnc = None

def initCORBA(argv):
    global rootnc
    orb = CORBA.ORB_init(argv)
    nameserver = orb.string_to_object("corbaloc::localhost/NameService")
    rootnc = nameserver._narrow(CosNaming.NamingContext)
    return None

def findObject(name, kind):
    global rootnc
    nc = CosNaming.NameComponent(name,kind)
    path= [nc]
    try:
        obj = rootnc.resolve(path)
        return obj
    except CosNaming.NamingContext.NotFound, ex:
        print ex
        raise

initCORBA(sys.argv)
try:
    obj = findObject("OnlineViewer", "")
    if len(sys.argv) > 1 :
        robotname = sys.argv[1]
    else:
        robotname = False
    if robotname:
        print ";; getPosture of ",robotname
        ret = obj.getPosture(robotname);
        print ";;               -> ",ret
        if not ret[0]:
            raise
except:
    exit(-1)

exit(0)
