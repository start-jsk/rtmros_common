import rtm
import hrp

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

    print "[hrpsys.py] createComps -> findRTC : ",name+"(Robot)0"
    bridge = findRTC(name+"(Robot)0")
    print "[hrpsys.py] createComps -> bridge : ",bridge

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")

    print "[hrpsys.py] createComps -> SequencePlayer : ",seq
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "StateHolder0")
    print "[hrpsys.py] createComps -> StateHolder : ",sh
    sh_svc = StateHolderServiceHelper.narrow(sh.service("service0"));

    hgc = findRTC("HGcontroller0")

def init(_name):
    import time
    global ms
    global name

    name = _name
    ms = None

    while ms == None :
        time.sleep(1);
        ms = rtm.findRTCmanager()
        print "[hrpsys.py] wait for RTCmanager : ",ms

    while hrp.findModelLoader() == None: # seq uses modelloader
        time.sleep(1);
        print "[hrpsys.py] wait for ModelLoader"

    print "[hrpsys.py] createRTCmanager : ",ms
    print "[hrpsys.py] creating components"
    createComps()

    print "[hrpsys.py] connecting components"
    connectComps()

    print "[hrpsys.py] activating components"
    activateComps()
    print "[hrpsys.py] initialized successfully"


if __name__ == '__main__':
    import sys,os
    if len(sys.argv) < 2 :
        sys.exit("Usage : "+sys.argv[0]+" [ROBOTNAME]")

    os.environ['LD_LIBRARY_PATH']=os.environ['LD_LIBRARY_PATH']+":"+os.popen("rospack find hrpsys").read().rstrip()+"/lib"
    init(sys.argv[1])


