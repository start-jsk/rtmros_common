#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")
import OpenRTM_aist.RTM_IDL # for catkin
import sys

import hrpsys
from hrpsys.hrpsys_config import *
import OpenHRP
#from hrpsys.hrpsys_config import HrpsysConfigurator

# copy from hrpsys/lib/python2.7/dist-packages/hrpsys_config.py
if __name__ == '__main__':
    hcf = HrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()

