#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

# class for configure hrpsys RTCs and ports
class HrpsysConfigurator:
    def connectComps(self):
        connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"), self.co.port("qCurrent"), self.el.port("qCurrent"), self.ic.port("qCurrent")])
        #
        connectPorts(self.seq.port("qRef"), self.sh.port("qIn"))
        #
        connectPorts(self.rh.port("tau"), self.tf.port("tauIn"))
        # currently use first acc and rate sensors for kf
        s_acc=filter(lambda s : s.type == 'Acceleration', hcf.getSensors(self.url))[0]
        if (s_acc):
            connectPorts(self.rh.port(s_acc.name), self.kf.port("acc"))
        s_rate=filter(lambda s : s.type == 'RateGyro', hcf.getSensors(self.url))[0]
        if (s_rate):
            connectPorts(self.rh.port(s_rate.name), self.kf.port("rate"))
        connectPorts(self.seq.port("accRef"), self.kf.port("accRef"))
        #
        connectPorts(self.seq.port("basePos"), self.sh.port("basePosIn"))
        connectPorts(self.seq.port("baseRpy"), self.sh.port("baseRpyIn"))
        connectPorts(self.seq.port("zmpRef"),  self.sh.port("zmpIn"))
        #
        connectPorts(self.sh.port("qOut"),  self.ic.port("qRef"))
        connectPorts(self.ic.port("q"), self.abc.port("qRef"))
        connectPorts(self.abc.port("q"),  self.co.port("qRef"))
        connectPorts(self.co.port("q"),  self.el.port("qRef"))
        out_port = self.el.port("q")
        if self.simulation_mode :
            connectPorts(out_port,  self.hgc.port("qIn"))
            connectPorts(self.hgc.port("qOut"), [self.seq.port("qInit"), self.rh.port("qRef")])
        else :
            connectPorts(out_port,  [self.seq.port("qInit"), self.rh.port("qRef")])
        #
        connectPorts(self.sh.port("basePosOut"), self.seq.port("basePosInit"))
        connectPorts(self.sh.port("baseRpyOut"), self.seq.port("baseRpyInit"))

    def activateComps(self, rtcList):
        rtm.serializeComponents(rtcList)
        for r in rtcList:
            r.start()

    def createComps(self):

        self.ms.load("SequencePlayer")
        self.seq = self.ms.create("SequencePlayer", "seq")
        print "[hrpsys.py] createComps -> SequencePlayer : ",self.seq
        self.seq_svc = narrow(self.seq.service("service0"), "SequencePlayerService")

        self.ms.load("StateHolder");
        self.sh = self.ms.create("StateHolder", "sh")
        print "[hrpsys.py] createComps -> StateHolder : ",self.sh
        self.sh_svc = narrow(self.sh.service("service0"), "StateHolderService");
        self.tk_svc = narrow(self.sh.service("service1"), "TimeKeeperService")

        self.ms.load("TorqueFilter");
        self.tf = self.ms.create("TorqueFilter", "tf")
        print "[hrpsys.py] createComps -> TorqueFilter : ",self.tf

        self.ms.load("KalmanFilter");
        self.kf = self.ms.create("KalmanFilter", "kf")
        print "[hrpsys.py] createComps -> KalmanFilter : ",self.kf
        self.kf_svc = narrow(self.kf.service("service0"), "KalmanFilterService")

        self.ms.load("ImpedanceController");
        self.ic = self.ms.create("ImpedanceController", "ic")
        print "[hrpsys.py] createComps -> ImpednanceController : ",self.ic
        self.ic_svc = narrow(self.ic.service("service0"), "ImpedanceControllerService");

        self.ms.load("AutoBalancer");
        self.abc = self.ms.create("AutoBalancer", "abc")
        print "[hrpsys.py] createComps -> AutoBalancerController : ",self.abc
        self.abc_svc = narrow(self.abc.service("service0"), "AutoBalancerService");

        self.ms.load("CollisionDetector");
        self.co = self.ms.create("CollisionDetector", "co")
        print "[hrpsys.py] createComps -> CollisionDetector : ",self.co
        self.co_svc = narrow(self.co.service("service0"), "CollisionDetectorService");

        self.ms.load("SoftErrorLimiter");
        self.el = self.ms.create("SoftErrorLimiter", "el")
        print "[hrpsys.py] createComps -> SoftErrorLimiter : ",self.el

        self.ms.load("DataLogger");
        self.log = self.ms.create("DataLogger", "log")
        print "[hrpsys.py] createComps -> DataLogger : ",self.log
        self.log_svc = narrow(self.log.service("service0"), "DataLoggerService");

    def getRTCList(self):
        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.ic, self.abc, self.co, self.el, self.log]

    def getBodyInfo(self, url):
        import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader)
        print "[hrpsys.py]   bodyinfo URL = file://"+url
        return mdlldr.getBodyInfo("file://"+url)

    def getSensors(self, url):
        return sum(map(lambda x : x.sensors, filter(lambda x : len(x.sensors) > 0, self.getBodyInfo(url)._get_links())), [])  # sum is for list flatten

    # setup logger
    def setupLogger(self, url=None):
        #
        if self.rh.port("q") :
            self.log_svc.add("TimedDoubleSeq", "q")
            connectPorts(self.rh.port("q"), self.log.port("q"))

        if self.rh.port("tau") :
            self.log_svc.add("TimedDoubleSeq", "tau")
            connectPorts(self.rh.port("tau"), self.log.port("tau"))
        # sensor logger ports
        if url :
            print "[hrpsys.py] sensor names for DataLogger"
            for sen in hcf.getSensors(url):
                if self.rh.port(sen.name) != None:
                    sen_type = rtm.dataTypeOfPort(self.rh.port(sen.name)).split("/")[1].split(":")[0]
                    print "[hrpsys.py]   type =", sen_type, ",name = ", sen.name, ",port = ", self.rh.port(sen.name)
                    self.log_svc.add(sen_type, sen.name)
                    connectPorts(self.rh.port(sen.name), self.log.port(sen.name))

        self.log.owned_ecs[0].start()
        self.log.start(self.log.owned_ecs[0])

    def findRTCManagerAndRoboHardware(self, robotname="Robot"):

        self.ms = None
        while self.ms == None :
            time.sleep(1);
            self.ms = rtm.findRTCmanager()
            print "[hrpsys.py] wait for RTCmanager : ",self.ms

        self.rh = None
        timeout_count = 0;
        self.simulation_mode = False
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.rh == None and timeout_count < 3: # <- time out limit
            time.sleep(1);
            self.rh = rtm.findRTC("RobotHardware0")
            if self.rh:
                self.rh_svc = narrow(self.rh.service("service0"), "RobotHardwareService")
                self.ep_svc = narrow(self.rh.ec, "ExecutionProfileService")
            else:
                self.rh = rtm.findRTC(robotname)
                self.hgc = findRTC("HGcontroller0")
                self.simulation_mode = True
            print "[hrpsys.py] wait for Simulator or RobotHardware : ",self.rh, "(timeout ", timeout_count, " < 3)"
            timeout_count += 1

        if not self.rh:
            print "[hrpsys.py] Could not find ", robotname
            print "[hrpsys.py] Candidates are .... ", [x.name()  for x in self.ms.get_components()]
            print "[hrpsys.py] Exitting.... ", robotname
            return

        print "[hrpsys.py] findComps -> RobotHardware : ",self.rh

    def findModelLoader(self):
        try:
            return rtm.findObject("ModelLoader")
        except:
            return None

    def waitForModelLoader(self):
        while self.findModelLoader() == None: # seq uses modelloader
            print "[hrpsys.py] wait for ModelLoader"
            time.sleep(3);

    def init(self, robotname="Robot", url=""):
        self.url = url
        print "[hrpsys.py] waiting ModelLoader"
        self.waitForModelLoader()
        print "[hrpsys.py] start hrpsys"

        print "[hrpsys.py] finding RTCManager and RobotHardware"
        self.findRTCManagerAndRoboHardware(robotname)

        print "[hrpsys.py] creating components"
        self.createComps()

        print "[hrpsys.py] activating components"
        self.activateComps(self.getRTCList())

        print "[hrpsys.py] connecting components"
        self.connectComps()

        print "[hrpsys.py] initialized successfully"

        self.setupLogger(url)
        print "[hrpsys.py] setup logger done"


if __name__ == '__main__':
    hcf = HrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()



