#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time
seq = rtm.findRTC("seq")
seq_svc = narrow(seq.service("service0"), "SequencePlayerService")
col = rtm.findRTC("co")
col_svc = narrow(col.service("service0"), "CollisionDetectorService")
seq_svc.setJointAngles([0,0,0,0,0,0,0,0,0],10)
seq_svc.waitInterpolation()
seq_svc.setJointAngles([0,1.0,0,2.0,0,1.5,0,0,0],10)
seq_svc.waitInterpolation()
seq_svc.setJointAngles([0,1.2,0,2.2,0,1.7,0,0,0],10)
seq_svc.waitInterpolation()
#
seq_svc.setJointAngles([0,0,0,0,0,0,0,0,0],10)
seq_svc.waitInterpolation()
col_svc.setTolerance("all",0.03)
seq_svc.setJointAngles([0,1.0,0,2.0,0,1.5,0,0,0],10)
seq_svc.waitInterpolation()
seq_svc.setJointAngles([0,1.2,0,2.2,0,1.7,0,0,0],10)
seq_svc.waitInterpolation()
