hostname='hiro014' # you need to add this host/ip information to /etc/hosts

from java.lang import System
System.setProperty('NS_OPT', '-ORBInitRef NameService=corbaloc:iiop:'+hostname+':2809/NameService')

import rtm
from rtm import *
from OpenHRP import *

def createComps():
    global seq, seq_svc

    ms.load("SequencePlayer")
    print ms
    seq = findRTC("seq")
    print seq
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))
    print ms,seq,seq_svc
    print dir(seq_svc)

def init():
    global ms, hostname

    ms = rtm.findRTCmanager(hostname=hostname)

    print "creating components"
    createComps()
    print "initialized successfully"

def playPattern(tm=10.0):
    seq_svc.playPattern([[0,    0,    0,
                         -0.0,  0, -1.75,  0.26,  0.16,  0.5,
                          0.0,  0, -1.75, -0.26,  0.16, -0.5]], [], [], [tm])
    seq_svc.playPattern([[0,    0,    0,
                         -0.0,  0, -1.95,  0.26,  0.16,  0.5,
                          0.0,  0, -1.75, -0.26,  0.16, -0.5],
                         [0,    0,    0,
                         -0.0,  0, -1.75,  0.26,  0.16,  0.5,
                          0.0,  0, -1.75, -0.26,  0.16, -0.5],
                         ], [], [], [2,4])
    seq_svc.waitInterpolation()

init()
playPattern()
