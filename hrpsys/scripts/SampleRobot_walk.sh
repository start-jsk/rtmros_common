#!/bin/sh

WALKDATA=`rospack find openhrp3`/share/OpenHRP-3.1/sample/controller/SampleController/etc/Sample
`rospack find hrpsys`/bin/hrpsyspy <<EOF
import rtm

from rtm import *
from OpenHRP import *

def init():
    global seq, seq_svc
    seq = findRTC("seq")
    print seq
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))
    print seq_svc

def loadPattern(basename, tm=1.0):
    print basename
    seq_svc.loadPattern(basename, tm)
    seq_svc.waitInterpolation()

init()
loadPattern("$WALKDATA")

EOF


