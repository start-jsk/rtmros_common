#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")

import os
import rtm

from rtm import *
from OpenHRP import *

import socket
import time


rh=rtm.findRTC("RobotHardware0")
rhr=rtm.findRTC("RobotHardwareServiceROSBridge")

connectPorts(rhr.port("RobotHardwareService"),rh.port("RobotHardwareService"))

