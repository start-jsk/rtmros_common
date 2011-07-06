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

    print "createComps -> findRTC : ",name+"(Robot)0"
    bridge = findRTC(name+"(Robot)0")
    print "createComps -> bridge : ",bridge

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    print "createComps -> SequencePlayer : ",seq
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("StateHolder");
    sh = ms.create("StateHolder", "StateHolder0")
    print "createComps -> StateHolder : ",sh
    sh_svc = StateHolderServiceHelper.narrow(sh.service("service0"));

    hgc = findRTC("HGcontroller0")

def init(_name):
    global ms
    global name
    name = _name

    ms = rtm.findRTCmanager()
    print "createRTCmanager : ",ms

    print "creating components"
    createComps()

    print "connecting components"
    connectComps()

    print "activating components"
    activateComps()
    print "initialized successfully"


if __name__ == '__main__':
    import sys,os
    if len(sys.argv) < 2 :
        sys.exit("Usage : "+sys.argv[0]+" [ROBOTNAME]")

    os.environ['LD_LIBRARY_PATH']=os.environ['LD_LIBRARY_PATH']+":"+os.popen("rospack find hrpsys").read().rstrip()+"/lib"
    init(sys.argv[1])


