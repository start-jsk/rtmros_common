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
        # connection for actual joint angles
        connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"), self.co.port("qCurrent"), self.el.port("qCurrent"), self.vs.port("qCurrent"), self.tf.port("qCurrent"), self.ic.port("qCurrent")])
        # connection for reference joint angles
        tmp_contollers = [self.ic, self.abc, self.st, self.co, self.el]
        connectPorts(self.sh.port("qOut"),  tmp_contollers[0].port("qRef"))
        for i in range(len(tmp_contollers)-1):
            connectPorts(tmp_contollers[i].port("q"), tmp_contollers[i+1].port("qRef"))
        if self.simulation_mode :
            connectPorts(tmp_contollers[-1].port("q"),  self.hgc.port("qIn"))
            connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
        else :
            connectPorts(tmp_contollers[-1].port("q"),  self.rh.port("qRef"))
        connectPorts(self.seq.port("qRef"), self.sh.port("qIn"))
        # connection for actual torques
        if rtm.findPort(self.rh.ref, "tau") != None:
            connectPorts(self.rh.port("tau"), self.tf.port("tauIn"))
        connectPorts(self.tf.port("tauOut"), self.vs.port("tauIn"))

        # connection for kf
        #   currently use first acc and rate sensors for kf
        s_acc=filter(lambda s : s.type == 'Acceleration', self.getSensors(self.url))
        if (len(s_acc)>0) and self.rh.port(s_acc[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            connectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
        s_rate=filter(lambda s : s.type == 'RateGyro', self.getSensors(self.url))
        if (len(s_rate)>0) and self.rh.port(s_rate[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
        connectPorts(self.seq.port("accRef"), self.kf.port("accRef"))
        #connectPorts(self.kf.port("rpy"), self.ic.port("rpy"))

        # connection for rh
        if self.rh.port("servoState") != None:
            connectPorts(self.rh.port("servoState"), self.el.port("servoStateIn"))

        # connection for sh
        connectPorts(self.seq.port("basePos"), self.sh.port("basePosIn"))
        connectPorts(self.seq.port("baseRpy"), self.sh.port("baseRpyIn"))
        connectPorts(self.seq.port("zmpRef"),  self.sh.port("zmpIn"))
        connectPorts(self.sh.port("basePosOut"), self.seq.port("basePosInit"))
        connectPorts(self.sh.port("baseRpyOut"), self.seq.port("baseRpyInit"))
        connectPorts(self.sh.port("qOut"), self.seq.port("qInit"))

        # connection for st
        if rtm.findPort(self.rh.ref, "lfsensor") and rtm.findPort(self.rh.ref, "rfsensor"):
            connectPorts(self.rh.port("lfsensor"), self.st.port("forceL"))
            connectPorts(self.rh.port("rfsensor"), self.st.port("forceR"))
            connectPorts(self.kf.port("rpy"), self.st.port("rpy"))
            connectPorts(self.abc.port("zmpRef"), self.st.port("zmpRef"))
            connectPorts(self.abc.port("baseRpy"), self.st.port("baseRpyIn"))
            connectPorts(self.abc.port("basePos"), self.st.port("basePosIn"))

        # connection for ic
        #  actual force sensors
        for sen in filter(lambda x : x.type == "Force", self.getSensors(self.url)):
            connectPorts(self.rh.port(sen.name), self.ic.port(sen.name))
        #  virtual force sensors
        for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, self.vs.ports.keys()):
            connectPorts(self.vs.port(vfp), self.ic.port(vfp))

    def activateComps(self):
        rtcList = self.getRTCList()
        rtm.serializeComponents(rtcList)
        for r in rtcList:
            r.start()

    def createComp(self, compName, instanceName):
        self.ms.load(compName)
        comp = self.ms.create(compName, instanceName)
        print self.configurator_name, "create Comp -> ", compName, " : ", comp
        if comp == None:
            raise RuntimeError("Cannot create component: " + compName)
        return comp

    def createComps(self):
        self.seq = self.createComp("SequencePlayer", "seq")

        self.sh = self.createComp("StateHolder", "sh")

        self.tf = self.createComp("TorqueFilter", "tf")

        self.kf = self.createComp("KalmanFilter", "kf")

        self.vs = self.createComp("VirtualForceSensor", "vs")

        self.ic = self.createComp("ImpedanceController", "ic")

        self.abc = self.createComp("AutoBalancer", "abc")

        self.st = self.createComp("Stabilizer", "st")

        self.co = self.createComp("CollisionDetector", "co")

        self.el = self.createComp("SoftErrorLimiter", "el")

        self.log = self.createComp("DataLogger", "log")
        self.log_svc = narrow(self.log.service("service0"), "DataLoggerService");

    # public method to configure all RTCs to be activated on rtcd
    def getRTCList(self):
        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.vs, self.ic, self.abc, self.st, self.co, self.el, self.log]

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



