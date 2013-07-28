#!/usr/bin/env python
import roslib;
import sys; sys.path.insert (0, roslib.packages.get_pkg_dir('hrpsys')+'/scripts');
from hrpsys_config import *

class ATLASHrpsysConfigurator(HrpsysConfigurator):
    def connectComps(self):
        HrpsysConfigurator.connectComps(self)
        # connect for kf rtc dummy rpy
        s_acc=filter(lambda s : s.type == 'Acceleration', self.sensors)
        if (len(s_acc)>0) and self.rh.port(s_acc[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            disconnectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
        s_rate=filter(lambda s : s.type == 'RateGyro', self.sensors)
        if (len(s_rate)>0) and self.rh.port(s_rate[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            disconnectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
            connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rpyIn"))
        # delete co
        disconnectPorts(self.rh.port("q"), self.co.port("qCurrent"))
        disconnectPorts(self.st.port("q"), self.co.port("qRef"))
        disconnectPorts(self.co.port("q"), self.el.port("qRef"))
        connectPorts(self.st.port("q"), self.el.port("qRef"))

#    # delete co
#    def getRTCList(self):
#        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.vs, self.ic, self.abc, self.st, self.el, self.log]

#    def activateComps(self):
#        HrpsysConfigurator.activateComps(self)
#        # stop co
#        self.co.stop()

    def init(self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)

if __name__ == '__main__':
    shcf=ATLASHrpsysConfigurator()
    shcf.init(sys.argv[1], sys.argv[2])
