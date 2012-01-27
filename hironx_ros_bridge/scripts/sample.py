from java.lang import System
System.setProperty('NS_OPT', '-ORBInitRef NameService=corbaloc:iiop:192.168.128.14:2809/NameService')

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
    global ms

    ms = rtm.findRTCmanager()

    print "creating components"
    createComps()
    print "initialized successfully"

def loadPattern(tm=10.0):
    seq_svc.playPattern([0,    0,    0,
                         -0.6,  0, -100,  15.2,  9.4,  3.2,
                          0.6,  0, -100, -15.2,  9.4, -3.2], tm)
    seq_svc.waitInterpolation()

init()
loadPattern()
