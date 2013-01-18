#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

def connectComps():
    connectPorts(rh.port("q"), [sh.port("currentQIn"), co.port("qCurrent"), el.port("qCurrent"), ic.port("qCurrent")])
    #
    connectPorts(seq.port("qRef"), sh.port("qIn"))
    #
    connectPorts(rh.port("tau"), tf.port("tauIn"))
    #
    connectPorts(seq.port("basePos"), sh.port("basePosIn"))
    connectPorts(seq.port("baseRpy"), sh.port("baseRpyIn"))
    connectPorts(seq.port("zmpRef"),  sh.port("zmpIn"))
    #
    connectPorts(sh.port("qOut"),  ic.port("qRef"))
    connectPorts(ic.port("q"),  co.port("qRef"))
    connectPorts(co.port("q"),  el.port("qRef"))
    out_port = el.port("q")
    if simulation_mode :
        connectPorts(out_port,  hgc.port("qIn"))
        connectPorts(hgc.port("qOut"), [seq.port("qInit"), rh.port("qRef")])
    else :
        connectPorts(out_port,  [seq.port("qInit"), rh.port("qRef")])
    #
    connectPorts(sh.port("basePosOut"), seq.port("basePosInit"))
    connectPorts(sh.port("baseRpyOut"), seq.port("baseRpyInit"))

def activateComps():
    rtm.serializeComponents([rh, seq, sh, tf, ic, co, el, log])
    rh.start()
    seq.start()
    sh.start()
    tf.start()
    ic.start()
    co.start()
    el.start()
    log.start()

def createComps():
    global seq, seq_svc, sh, sh_svc, tf, ic, ic_svc, co, co_svc, el, log, log_svc

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    print "[hrpsys.py] createComps -> SequencePlayer : ",seq
    seq_svc = narrow(seq.service("service0"), "SequencePlayerService")

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "sh")
    print "[hrpsys.py] createComps -> StateHolder : ",sh
    sh_svc = narrow(sh.service("service0"), "StateHolderService");
    tk_svc = narrow(sh.service("service1"), "TimeKeeperService")

    ms.load("TorqueFilter");
    tf = ms.create("TorqueFilter", "tf")
    print "[hrpsys.py] createComps -> TorqueFilter : ",tf

    ms.load("ImpedanceController");
    ic = ms.create("ImpedanceController", "ic")
    print "[hrpsys.py] createComps -> ImpedanceController : ",ic
    ic_svc = narrow(ic.service("service0"), "ImpedanceControllerService");

    ms.load("CollisionDetector");
    co = ms.create("CollisionDetector", "co")
    print "[hrpsys.py] createComps -> CollisionDetector : ",co
    co_svc = narrow(co.service("service0"), "CollisionDetectorService");

    ms.load("SoftErrorLimiter");
    el = ms.create("SoftErrorLimiter", "el")
    print "[hrpsys.py] createComps -> SoftErrorLimiter : ",el

    ms.load("DataLogger");
    log = ms.create("DataLogger", "log")
    print "[hrpsys.py] createComps -> DataLogger : ",log
    log_svc = narrow(log.service("service0"), "DataLoggerService");


# setup logger
def setupLogger(url=None):
    #
    if rh.port("q") :
        log_svc.add("TimedDoubleSeq", "q")
        connectPorts(rh.port("q"), log.port("q"))

    if rh.port("tau") :
        log_svc.add("TimedDoubleSeq", "tau")
        connectPorts(rh.port("tau"), log.port("tau"))
    # sensor logger ports
    if url :
        print "[hrpsys.py] sensor names for DataLogger"
        import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader)
        print "[hrpsys.py]   bodyinfo URL = file://"+url
        bodyInfo = mdlldr.getBodyInfo("file://"+url)
        ret = []
        for ll in bodyInfo._get_links():
            if len(ll.sensors) > 0:
                ret.extend(ll.sensors)
        for sen in map(lambda x : x.name, ret):
            if sen == "gyrometer":
                sen_type = "TimedAngularVelocity3D"
            elif sen == "gsensor":
                sen_type = "TimedAcceleration3D"
            elif sen.find("fsensor") != -1:
                sen_type = "TimedDoubleSeq"
            else:
                continue
            print "[hrpsys.py]   type =", sen_type, ",name = ", sen, ",port = ", rh.port(sen)
            if rh.port(sen) != None:
                log_svc.add(sen_type, sen)
                connectPorts(rh.port(sen), log.port(sen))

    log.owned_ecs[0].start()
    log.start(log.owned_ecs[0])

def init(robotname="Robot", url=""):
    global ms, rh, rh_svc, ep_svc, hgc, simulation_mode

    ms = rtm.findRTCmanager()
    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager()
        print "[hrpsys.py] wait for RTCmanager : ",ms

    rh = rtm.findRTC("RobotHardware0")
    if rh:
        rh_svc = narrow(rh.service("service0"), "RobotHardwareService")
        ep_svc = narrow(rh.ec, "ExecutionProfileService")
    else:
        rh = rtm.findRTC(robotname)
        hgc = findRTC("HGcontroller0")
        simulation_mode = True
    if not rh:
        print "[hrpsys.py] Could not find ", robotname
        print "[hrpsys.py] Candidates are .... ", [x.name()  for x in ms.get_components()]
        print "[hrpsys.py] Exitting.... ", robotname
        return

    print "[hrpsys.py] findComps -> RobotHardware : ",rh

    print "[hrpsys.py] creating components"
    createComps()

    print "[hrpsys.py] activating components"
    activateComps()

    print "[hrpsys.py] connecting components"
    connectComps()

    print "[hrpsys.py] initialized successfully"

    setupLogger(url)
    print "[hrpsys.py] setup logger done"


def findModelLoader():
    try:
        return rtm.findObject("ModelLoader")
    except:
        return None

if __name__ == '__main__':

    while findModelLoader() == None: # seq uses modelloader
        print "[hrpsys.py] wait for ModelLoader"
        time.sleep(3);

    print "[hrpsys.py] start hrpsys"

    if len(sys.argv) > 2 :
        init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        init(sys.argv[1])
    else :
        init()



