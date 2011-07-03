# import time
# import com.generalrobotix.ui.item.GrxSimulationItem as GrxSimulationItem
# import syncExec
# import java.lang.Runnable as Runnable

# class StartSim(Runnable):
#     def run(self):
#         sim.startSimulation(0)
#         return None

# class SetTime(Runnable):
#     def run(self):
#         sim.setDbl("totalTime", 5.0)
#         return None

# sim = uimanager.getSelectedItem(GrxSimulationItem, None)
# syncExec.exec(StartSim())

import rtm

from rtm import *
from OpenHRP import *

def connectComps():
    connectPorts(bridge.port("q"), seq.port("qInit"))
    #
    connectPorts(seq.port("qRef"), hgc.port("qIn"))
    #
    connectPorts(bridge.port("q"), sh.port("qIn"))

def activateComps():
    rtm.serializeComponents([bridge, seq, sh])
    seq.start()
    sh.start()

def createComps():
    global bridge, seq, seq_svc, sh, sh_svc, hgc

    bridge = findRTC("Darwin(Robot)0")

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "StateHolder0")

    sh_svc = StateHolderServiceHelper.narrow(sh.service("service0"));

    hgc = findRTC("HGcontroller0")

def init():
    global ms

    ms = rtm.findRTCmanager()

    print "creating components"
    createComps()
      
    print "connecting components"
    connectComps()

    print "activating components"
    activateComps()
    print "initialized successfully"

def loadPattern(basename, tm=1.0):
    print basename
    seq_svc.loadPattern(basename, tm)
    seq_svc.waitInterpolation()

init()





