import sys
import rtm
import hrp

from rtm import *
from OpenHRP import *

def connectComps():
    connectPorts(sim.port("q"), seq.port("qInit"))
    #
    connectPorts(seq.port("qRef"), hgc.port("qIn"))
    #
    connectPorts(hgc.port("qOut"), sim.port("qCmd"))
    connectPorts(hgc.port("dqOut"), sim.port("dqCmd"))
    connectPorts(hgc.port("ddqOut"), sim.port("ddqCmd"))
    #
    connectPorts(sim.port("q"), sh.port("qIn"))

def activateComps():
    rtm.serializeComponents([sim, seq, sh, hgc])
    sim.start()
    seq.start()
    sh.start()
    hgc.start()

def createComps():
    global seq, seq_svc, sh, sh_svc, hgc

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")

    print "[hrpsys.py] createComps -> SequencePlayer : ",seq
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("HGcontroller");
    hgc = ms.create("HGcontroller")
    print "[hrpsys.py] createComps -> HGcontroller : ",hgc

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "StateHolder0")
    print "[hrpsys.py] createComps -> StateHolder : ",sh
    sh_svc = StateHolderServiceHelper.narrow(sh.service("service0"));


def init(simulator="Simulator"):
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


if __name__ == '__main__':

    while hrp.findModelLoader() == None: # seq uses modelloader
        time.sleep(3);
        print "[hrpsys.py] wait for ModelLoader"

    print "[hrpsys.py] start hrpsys"

    if len(sys.argv) > 1 :
        init(sys.argv[1])
    else :
        init()



