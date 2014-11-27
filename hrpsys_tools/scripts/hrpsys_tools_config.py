#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")
import OpenRTM_aist.RTM_IDL # for catkin
import sys

from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP

# copy from hrpsys/lib/python2.7/dist-packages/hrpsys_config.py
import argparse, code
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="hrpsys command line interpreters, try `./hrpsys_tools_config.py--host xxxx --port xxxx  -i` or `ipython ./hrpsys_tools_config.py -- --host xxxx --port xxxx` for robot debugging, we recommend to use ipython")
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('-i', help='interactive mode',  action='store_true')
    parser.add_argument('-c', help='execute command',  nargs='*')
    parser.add_argument('--use-unstable-rtc', help='use unstable rtc', action='store_true')
    args, unknown = parser.parse_known_args()

    if args.i: # interactive
        sys.argv.remove('-i')
    if args.c:
        sys.argv.remove('-c')
        [sys.argv.remove(a) for a in args.c] # remove command from sys.argv
    if args.host:
        rtm.nshost = args.host; sys.argv = [sys.argv[0]] + sys.argv[3:]
    if args.port:
        rtm.nsport = args.port; sys.argv = [sys.argv[0]] + sys.argv[3:]

    # ipython ./hrpsys_tools_config.py -i -- --port 2809
    hcf = HrpsysConfigurator()
    if args.use_unstable_rtc: # use Unstable RTC
        hcf.getRTCList = hcf.getRTCListUnstable; sys.argv = [sys.argv[0]] + sys.argv[2:]
    if args.i or '__IPYTHON__' in vars(__builtins__):
        hcf.waitForModelLoader()
        if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
            hcf.waitForRTCManagerAndRoboHardware(robotname=sys.argv[1])
            sys.argv = [sys.argv[0]] + sys.argv[2:]
        hcf.findComps()
        print >> sys.stderr, "[hrpsys.py] #\n[hrpsys.py] # use `hcf` as robot interface, for example hcf.getJointAngles()\n[hrpsys.py] #"
        while args.c != None:
            print >> sys.stderr, ">>", args.c[0]
            exec(args.c[0])
            args.c.pop(0)
        if not (args.i and '__IPYTHON__' in vars(__builtins__)):
            code.interact(local=locals()) #drop in shell if invoke from python, or ipython without -i option
    elif len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()

