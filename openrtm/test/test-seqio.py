#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge'
import roslib; roslib.load_manifest(PKG)

import unittest


# rtm modules
import sys, time, signal, os
import RTC
import OpenRTM_aist

global mgr

class TestComp(OpenRTM_aist.DataFlowComponentBase):
    def __init__ (self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        port = "Short"
        self.inport = OpenRTM_aist.InPort(port, RTC.TimedShort)
        self.registerInPort(port, self.inport)
        self.data_received = 0
        self.data_previous = 0

        return RTC.RTC_OK

    def onExecute(self, ec_id):
        if self.inport.isNew():
            data = self.inport.read()
            print data
            if ( self.data_received > 0 ) :
                print "prev data = ", self.data_previous, ", current data = ", data.data
                assert data.data == self.data_previous + 1
            self.data_previous=data.data
            self.data_received += 1
            print signal.SIGINT, os.getpid()
            if ( self.data_received > 10 ) :
                os.kill(os.getpid(), signal.SIGINT)

        return RTC.RTC_OK

module_spec = ["implementation_id", "test_seqio",
               "type_name",         "test_seqio",
               "description",       "Dataport ROS bridge component",
               "version",           "1.0",
               "vendor",            "Kei Okada",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]

def TestInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    print  profile
    manager.registerFactory(profile,
                            TestComp,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent("test_seqio")

class TestSeqIO(unittest.TestCase):
    def test_SeqIo(self):
        mgr = OpenRTM_aist.Manager.init(sys.argv)
        # signal.signal(signal.SIGINT, handler)
        mgr.setModuleInitProc(TestInit)
        mgr.activateManager()
        mgr.runManager()

#suite = unittest.TestSuite()
#suite.addTest(unittest.FunctionTestCase(test_seqio))
#unittest.TextTestRunner(verbosity=2).run(suite)
import rosunit
rosunit.unitrun(PKG, 'test_hronx_ros_bridge', TestSeqIO)






