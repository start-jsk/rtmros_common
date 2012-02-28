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
    rtm.serializeComponents([sim, seq])
    sim.start()
    seq.start()
    hgc.start()

def createComps():
    global sim, seq, seq_svc, sh, sh_svc, hgc

    sim = None
    print "[hrpsys.py] createComps -> find SimulationRTC"
    while sim == None :
        sim = findRTC("Simulator0")
        print "[hrpsys.py] createComps -> sim : ",sim

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


def init(_name):
    global ms
    global name

    name = _name
    ms = None

    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager()
        print "[hrpsys.py] wait for RTCmanager : ",ms

    print "[hrpsys.py] createRTCmanager : ",ms
    print "[hrpsys.py] creating components"
    createComps()

    print "[hrpsys.py] activating components"
    activateComps()

    print "[hrpsys.py] connecting components"
    connectComps()

    print "[hrpsys.py] initialized successfully"


if __name__ == '__main__':
    import sys,os,time
    if len(sys.argv) < 2 :
        sys.exit("Usage : "+sys.argv[0]+" [XMLFILE]")

    while hrp.findModelLoader() == None: # seq uses modelloader
        time.sleep(3);
        print "[hrpsys.py] wait for ModelLoader"

    modelname=""
    while modelname=="" :
        prog=os.popen("rospack find openhrp3").read().strip()+"/bin/extract-robotname "+sys.argv[1];
        modelname=os.popen(prog).read().strip()

    os.environ['LD_LIBRARY_PATH']=os.environ['LD_LIBRARY_PATH']+":"+os.popen("rospack find hrpsys").read().strip()+"/lib"

    print "[hrpsys.py] start hrpsys for ",modelname
    init(modelname)


