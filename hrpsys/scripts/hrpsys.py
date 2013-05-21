#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time

# class for configure hrpsys RTCs and ports
#   In order to specify robot-dependent code, please inherit this HrpsysConfigurator
class HrpsysConfigurator:

    # public method
    def connectComps(self):
        connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"), self.co.port("qCurrent"), self.el.port("qCurrent"), self.vs.port("qCurrent"), self.tf.port("qCurrent"), self.ic.port("qCurrent")])
        #
        connectPorts(self.seq.port("qRef"), self.sh.port("qIn"))
        #
        connectPorts(self.rh.port("tau"), self.tf.port("tauIn"))
        connectPorts(self.tf.port("tauOut"), self.vs.port("tauIn"))
        # currently use first acc and rate sensors for kf
        s_acc=filter(lambda s : s.type == 'Acceleration', self.getSensors(self.url))
        if (len(s_acc)>0) and self.rh.port(s_acc[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            connectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
        s_rate=filter(lambda s : s.type == 'RateGyro', self.getSensors(self.url))
        if (len(s_rate)>0) and self.rh.port(s_rate[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
        connectPorts(self.seq.port("accRef"), self.kf.port("accRef"))
        # for rh
        if self.rh.port("servoState") != None:
            connectPorts(self.rh.port("servoState"), self.el.port("servoStateIn"))
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
            connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
        else :
            connectPorts(out_port,  self.rh.port("qRef"))
        #
        connectPorts(self.sh.port("basePosOut"), self.seq.port("basePosInit"))
        connectPorts(self.sh.port("baseRpyOut"), self.seq.port("baseRpyInit"))
        connectPorts(self.sh.port("qOut"), self.seq.port("qInit"))
        #
        # virtual force sensor connection
        for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, self.vs.ports.keys()):
            connectPorts(self.vs.port(vfp), self.ic.port(vfp))

    def activateComps(self):
        rtcList = self.getRTCList()
        rtm.serializeComponents(rtcList)
        for r in rtcList:
            r.start()

    def createComps(self):
        self.ms.load("SequencePlayer")
        self.seq = self.ms.create("SequencePlayer", "seq")
        print self.configurator_name, "createComps -> SequencePlayer : ",self.seq
        self.seq_svc = narrow(self.seq.service("service0"), "SequencePlayerService")

        self.ms.load("StateHolder");
        self.sh = self.ms.create("StateHolder", "sh")
        print self.configurator_name, "createComps -> StateHolder : ",self.sh
        self.sh_svc = narrow(self.sh.service("service0"), "StateHolderService");
        self.tk_svc = narrow(self.sh.service("service1"), "TimeKeeperService")

        self.ms.load("TorqueFilter");
        self.tf = self.ms.create("TorqueFilter", "tf")
        print self.configurator_name, "createComps -> TorqueFilter : ",self.tf

        self.ms.load("KalmanFilter");
        self.kf = self.ms.create("KalmanFilter", "kf")
        print self.configurator_name, "createComps -> KalmanFilter : ",self.kf
        self.kf_svc = narrow(self.kf.service("service0"), "KalmanFilterService")

        self.ms.load("VirtualForceSensor");
        self.vs = self.ms.create("VirtualForceSensor", "vs")
        print self.configurator_name, "createComps -> VirtualForceSensor : ",self.vs

        self.ms.load("ImpedanceController");
        self.ic = self.ms.create("ImpedanceController", "ic")
        print self.configurator_name, "createComps -> ImpednanceController : ",self.ic
        self.ic_svc = narrow(self.ic.service("service0"), "ImpedanceControllerService");

        self.ms.load("AutoBalancer");
        self.abc = self.ms.create("AutoBalancer", "abc")
        print self.configurator_name, "createComps -> AutoBalancerController : ",self.abc
        self.abc_svc = narrow(self.abc.service("service0"), "AutoBalancerService");

        self.ms.load("CollisionDetector");
        self.co = self.ms.create("CollisionDetector", "co")
        print self.configurator_name, "createComps -> CollisionDetector : ",self.co
        self.co_svc = narrow(self.co.service("service0"), "CollisionDetectorService");

        self.ms.load("SoftErrorLimiter");
        self.el = self.ms.create("SoftErrorLimiter", "el")
        print self.configurator_name, "createComps -> SoftErrorLimiter : ",self.el

        self.ms.load("DataLogger");
        self.log = self.ms.create("DataLogger", "log")
        print self.configurator_name, "createComps -> DataLogger : ",self.log
        self.log_svc = narrow(self.log.service("service0"), "DataLoggerService");

    # public method to configure all RTCs to be activated on rtcd
    def getRTCList(self):
        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.vs, self.ic, self.abc, self.co, self.el, self.log]

    # public method to get bodyInfo
    def getBodyInfo(self, url):
        import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader)
        print self.configurator_name, "  bodyinfo URL = file://"+url
        return mdlldr.getBodyInfo("file://"+url)

    # public method to get sensors list
    def getSensors(self, url):
        return sum(map(lambda x : x.sensors, filter(lambda x : len(x.sensors) > 0, self.getBodyInfo(url)._get_links())), [])  # sum is for list flatten

    def connectLoggerPort(self, artc, sen_name):
        if rtm.findPort(artc.ref, sen_name) != None:
            sen_type = rtm.dataTypeOfPort(artc.port(sen_name)).split("/")[1].split(":")[0]
            print self.configurator_name, "  setupLogger : type =", sen_type, ",name = ", sen_name, ",port = ", artc.port(sen_name)
            if rtm.findPort(self.log.ref, sen_name) == None:
                self.log_svc.add(sen_type, sen_name)
            connectPorts(artc.port(sen_name), self.log.port(sen_name))

    # public method to configure default logger data ports
    def setupLogger(self, url=None):
        #
        for pn in ['q', 'tau']:
            self.connectLoggerPort(self.rh, pn)
        # sensor logger ports
        if url :
            print self.configurator_name, "sensor names for DataLogger"
            for sen in self.getSensors(url):
                self.connectLoggerPort(self.rh, sen.name)
        #
        self.connectLoggerPort(self.kf, 'rpy')
        self.connectLoggerPort(self.seq, 'qRef')
        self.connectLoggerPort(self.rh, 'emergencySignal')

    def waitForRTCManagerAndRoboHardware(self, robotname="Robot", managerhost=socket.gethostname()):
        self.ms = None
        while self.ms == None :
            time.sleep(1);
            if managerhost == "localhost":
                managerhost = socket.gethostname()
            self.ms = rtm.findRTCmanager(managerhost)
            print self.configurator_name, "wait for RTCmanager : ",self.ms

        self.rh = None
        timeout_count = 0;
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.rh == None and timeout_count < 3: # <- time out limit
            time.sleep(1);
            self.rh = rtm.findRTC("RobotHardware0")
            if not self.rh:
                self.rh = rtm.findRTC(robotname)
            print self.configurator_name, "wait for Simulator or RobotHardware : ",self.rh, "(timeout ", timeout_count, " < 3)"
            timeout_count += 1

        if not self.rh:
            print self.configurator_name, "Could not find ", robotname
            print self.configurator_name, "Candidates are .... ", [x.name()  for x in self.ms.get_components()]
            print self.configurator_name, "Exitting.... ", robotname
            return

        print self.configurator_name, "findComps -> RobotHardware : ",self.rh

        # distinguish real robot from simulation by using "servoState" port
        if rtm.findPort(self.rh.ref, "servoState") == None:
            self.hgc = findRTC("HGcontroller0")
            self.simulation_mode = True
        else:
            self.simulation_mode = False
#           self.rh_svc = narrow(self.rh.service("service0"), "RobotHardwareService")
#           self.ep_svc = narrow(self.rh.ec, "ExecutionProfileService")

        print self.configurator_name, "simulation_mode : ", self.simulation_mode

    def findModelLoader(self):
        try:
            return rtm.findObject("ModelLoader")
        except:
            return None

    def waitForModelLoader(self):
        while self.findModelLoader() == None: # seq uses modelloader
            print self.configurator_name, "wait for ModelLoader"
            time.sleep(3);

    def init(self, robotname="Robot", url=""):
        self.url = url
        print self.configurator_name, "waiting ModelLoader"
        self.waitForModelLoader()
        print self.configurator_name, "start hrpsys"

        print self.configurator_name, "finding RTCManager and RobotHardware"
        self.waitForRTCManagerAndRoboHardware(robotname)

        print self.configurator_name, "creating components"
        self.createComps()

        print self.configurator_name, "connecting components"
        self.connectComps()

        print self.configurator_name, "activating components"
        self.activateComps()

        self.setupLogger(url)
        print self.configurator_name, "setup logger done"

        print self.configurator_name, "initialized successfully"

    def __init__(self, cname="[hrpsys.py] "):
        self.configurator_name = cname


if __name__ == '__main__':
    hcf = HrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()



