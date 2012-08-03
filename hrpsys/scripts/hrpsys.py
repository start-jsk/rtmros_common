#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

def connectComps():
    connectPorts(sim.port("q"), seq.port("qInit"))
    #
    connectPorts(seq.port("qRef"), sh.port("qIn"))
    connectPorts(sh.port("qOut"), hgc.port("qIn"))
    #
    connectPorts(hgc.port("qOut"), sim.port("qRef"))
    connectPorts(hgc.port("dqOut"), sim.port("dqRef"))
    connectPorts(hgc.port("ddqOut"), sim.port("ddqRef"))

def activateComps():
    rtm.serializeComponents([sim, seq, sh, log, hgc])
    sim.start()
    seq.start()
    sh.start()
    log.start()
    hgc.start()

def createComps():
    global seq, seq_svc, sh, sh_svc, hgc, log, log_svc

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")

    print "[hrpsys.py] createComps -> SequencePlayer : ",seq
    seq_svc = narrow(seq.service("service0"), "SequencePlayerService")

    ms.load("HGcontroller");
    hgc = ms.create("HGcontroller")
    print "[hrpsys.py] createComps -> HGcontroller : ",hgc

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "StateHolder0")
    print "[hrpsys.py] createComps -> StateHolder : ",sh
    sh_svc = narrow(sh.service("service0"), "StateHolderService");

    ms.load("DataLogger");
    log = ms.create("DataLogger", "log")
    print "[hrpsys.py] createComps -> DataLogger : ",log
    log_svc = narrow(log.service("service0"), "DataLoggerService");


# setup logger
def setupLogger(url=""):
    log_svc.add("TimedDoubleSeq", "q")
    #log_svc.add("TimedPoint3D", "pos")
    #log_svc.add("TimedOrientation3D", "rpy")
    connectPorts(sim.port("q"), log.port("q"))
    #connectPorts(sim.port("pos"), log.port("pos"))
    #connectPorts(sim.port("rpy"), log.port("rpy"))
    #if sim.port("tau") != None:
    #    log_svc.add("TimedDoubleSeq", "tau")
    #    connectPorts(sim.port("tau"), log.port("tau"))
    # sensor logger ports
    # import org.omg.CosNaming
    # import jp.go.aist.hrp.simulator
    # obj = rtm.rootnc.resolve([org.omg.CosNaming.NameComponent('ModelLoader', '')])
    # mdlldr = jp.go.aist.hrp.simulator.ModelLoaderHelper.narrow(obj)
    # print "[bodyinfo] URL = file://"+url
    # bodyInfo = mdlldr.getBodyInfo("file://"+url)
    # ret = []
    # for ll in bodyInfo.links():
    #     if len(ll.sensors) > 0:
    #         ret.extend(ll.sensors)
    # for sen in map(lambda x : x.name, ret):
    #     if sen == "gyrometer":
    #         sen_type = "TimedAngularVelocity3D"
    #     elif sen == "gsensor":
    #         sen_type = "TimedAcceleration3D"
    #     elif sen.find("fsensor") != -1:
    #         sen_type = "TimedDoubleSeq"
    #     else:
    #         continue
    #     print sen_type, sen
    #     log_svc.add(sen_type, sen)
    #     connectPorts(sim.port(sen), log.port(sen))


def init(simulator="Simulator", url=""):
    global ms,sim

    ms = None

    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager()
        print "[hrpsys.py] wait for RTCmanager : ",ms

    print "[hrpsys.py] createRTCmanager : ",ms

    sim = None

    while sim == None :
        sim = findRTC(simulator)

    print "[hrpsys.py] createComps -> Simulation : ",sim

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

    if len(sys.argv) > 1 :
        init(sys.argv[1], sys.argv[2])
    else :
        init()



