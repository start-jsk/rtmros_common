#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
NAME = 'test-import-python'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import argparse,unittest,rostest,sys

class TestImportPython(unittest.TestCase):
    def setUp(self):
        pass;

    def test_msg(self):
        import hrpsys_ros_bridge.msg
        self.assertTrue(hrpsys_ros_bridge.msg)

    def test_srv(self):
        import hrpsys_ros_bridge.srv
        self.assertTrue(hrpsys_ros_bridge.srv)

    def test_idl(self):
        import OpenRTM_aist.RTM_IDL
        import hrpsys_ros_bridge.Img_idl
        self.assertTrue(hrpsys_ros_bridge.Img_idl)

    def test_dashboard(self):
        import hrpsys_ros_bridge.hrpsys_dashboard
        self.assertTrue(hrpsys_ros_bridge.hrpsys_dashboard)

# unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestImportPython, sys.argv)
