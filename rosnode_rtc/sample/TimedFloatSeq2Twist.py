#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

import RTC
import OpenRTM_aist

import site
site.addsitedir('/tmp')
import RTMROSDataBridge

component_spec = ["implementation_id", "TimedFloatSeq2Twist",
                  "type_name",         "TimedFloatSeq2Twist",
                  "description",       "data convert component",
                  "version",           "1.0",
                  "vendor",            "",
                  "category",          "example",
                  "activity_type",     "DataFlowComponent",
                  "max_instance",      "10",
                  "language",          "Python",
                  "lang_type",         "script",
                  ""]

# TimedFloatSeq2Twist
class Component(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    self._in_data = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._inport = OpenRTM_aist.InPort("in", self._in_data)
    self.addInPort("in", self._inport)
    # this data definition is printed in rosnode_rtc screen
    self._out_data = RTMROSDataBridge.geometry_msgs_Twist(linear=RTMROSDataBridge.geometry_msgs_Vector3(x=0.0, y=0.0, z=0.0), angular=RTMROSDataBridge.geometry_msgs_Vector3(x=0.0, y=0.0, z=0.0))
    self._outport = OpenRTM_aist.OutPort("out", self._out_data)
    self.addOutPort("out", self._outport)
    return RTC.RTC_OK

  def onExecute(self, ec_id):
    if self._inport.isNew():
      data = self._inport.read()
      # data conversion
      x = data.data[0]/1000
      y = data.data[1]/1000
      self._out_data.linear.x = y
      self._out_data.angular.z = -x
      self._outport.write()
    return RTC.RTC_OK


def MyModuleInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=component_spec)
  manager.registerFactory(profile,
                          Component,
                          OpenRTM_aist.Delete)
  comp = manager.createComponent("TimedFloatSeq2Twist")

def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()
