#!/bin/sh

WALKDATA=`rospack find hrpsys`/share/hrpsys/samples/HRP4C/data/walk2m
python <<EOF
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

def init():
    global seq, seq_svc
    seq = findRTC("seq")
    print seq
    seq_svc = narrow(seq.service("service0"), "SequencePlayerService")
    print seq_svc

def loadPattern(basename, tm=1.0):
    print basename
    seq_svc.loadPattern(basename, tm)
    seq_svc.waitInterpolation()

init()
loadPattern("$WALKDATA")

EOF


