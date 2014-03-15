#!/usr/bin/env python

PKG = 'hrpsys_tools'
NAME = 'test-hrpsys-config'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

from hrpsys.hrpsys_config import *
from hrpsys import rtm

import argparse,unittest,rostest

class SampleHrpsysConfigurator(HrpsysConfigurator):
    def init(self, robotname="SampleRobot(Robot)0", url=""):
         HrpsysConfigurator.init(self, robotname=robotname, url=url)

class TestHrpsysConfigurator(unittest.TestCase):
    def setUp(self):
        parser = argparse.ArgumentParser(description='hrpsys command line interpreters')
        parser.add_argument('--host', help='corba name server hostname')
        parser.add_argument('--port', help='corba name server port number')
        args, unknown = parser.parse_known_args()

        if args.host:
            rtm.nshost = args.host
        if args.port:
            rtm.nsport = args.port
        h = SampleHrpsysConfigurator()

    def test(self):
        self.assertTrue(True,"ok")


if __name__ == '__main__':
    rostest.run(PKG, NAME, TestHrpsysConfigurator, sys.argv)
