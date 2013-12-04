#!/usr/bin/env python

__doc__ = """
Hrpsys_Dashboard
GUI for the hrpsys robots."""

import roslib
roslib.load_manifest('hrpsys_ros_bridge')

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import diagnostic_msgs.msg
import std_msgs.msg
import std_srvs.srv

#import hrpsys  #9/15/2013 Removed since it looks not in use.
import hrpsys_ros_bridge
from hrpsys_ros_bridge.srv import *

import rospy
from roslib import rosenv

from os import path
import threading
import time
import re

## check if pr2_dashboard is alive
try:
  import pr2_dashboard
  roslib.packages.get_pkg_dir('pr2_dashboard')
except ImportError:
  print >> sys.stderr, ";;\n;; pr2_dashboard is not found\n;; please try\n;;  >> apt-get install ros-groovy-pr2-gui\n;; "
  exit()
except roslib.packages.InvalidROSPkgException:
  sys.path.append(roslib.packages.get_pkg_dir('pr2_dashboard')+"src/")

from pr2_dashboard.status_control import StatusControl
from pr2_dashboard.power_state_control import PowerStateControl
from pr2_dashboard.diagnostics_frame import DiagnosticsFrame
from pr2_dashboard.rosout_frame import RosoutFrame


class HrpsysFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"

    def __init__(self, parent, id=wx.ID_ANY, title='Hrpsys Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP, use_diagnostics=True, use_rosout=True, use_battery=True):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)

        wx.InitAllImageHandlers()

        rospy.init_node('hrpsys_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("hrpsys_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass

        self.SetTitle('%s (%s)'%(title,rosenv.get_master_uri()))

        self._icons_path = path.join(roslib.packages.get_pkg_dir('pr2_dashboard'), "icons/")

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)

        static_poser = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagnostic"), wx.HORIZONTAL)
        sizer.Add(static_poser, 0)

        # Diagnostics
        if use_diagnostics:
          self._diagnostics_button = StatusControl(self, wx.ID_ANY, self._icons_path, "diag", True)
          self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
          static_poser.Add(self._diagnostics_button, 0)
        else:
          self._diagnostics_button = None

        # Rosout
        if use_rosout:
          self._rosout_button = StatusControl(self, wx.ID_ANY, self._icons_path, "rosout", True)
          self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
          static_poser.Add(self._rosout_button, 0)
        else:
          self._rosout_button = None

        # Log
        self._log_button = StatusControl(self, wx.ID_ANY, self._icons_path, "breakerleft", True)
        self._log_button.SetToolTip(wx.ToolTip("Log"))
        static_poser.Add(self._log_button, 0)
        if not use_diagnostics:
          self._log_button.set_ok()

        # Motors
        self._motors_button = StatusControl(self, wx.ID_ANY, self._icons_path, "motor", True)
        self._motors_button.SetToolTip(wx.ToolTip("Mode"))
        static_poser.Add(self._motors_button, 0)
        self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)
        if not use_diagnostics:
          self._motors_button.set_ok()

        # power / motor
        static_poser = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Power"), wx.HORIZONTAL)
        sizer.Add(static_poser, 0)

        self._power_button = StatusControl(self, wx.ID_ANY, self._icons_path, "runstop", False)
        self._power_button.SetToolTip(wx.ToolTip("Power"))
        static_poser.Add(self._power_button, 0)


        if use_battery:
          static_poser = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
          sizer.Add(static_poser, 0)

          # Battery State
          self._battery_state_ctrl = PowerStateControl(self, wx.ID_ANY, self._icons_path)
          self._battery_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
          static_poser.Add(self._battery_state_ctrl, 1, wx.EXPAND)
        else:
          self._battery_state_ctrl = None

        #
        self.setup_extra_layout()

        self._config = wx.Config("hrpsys_dashboard")

        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.Layout()
        self.Fit()

        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        if self._diagnostics_button:
          self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)

        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        if self._rosout_button:
          self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)

        self._log_frame = RosoutFrame(self, wx.ID_ANY, "Log")
        self._log_frame.Hide()
        self._log_frame.Center()
        self._log_button.Bind(wx.EVT_BUTTON, self.on_log_clicked)

        self.load_config()

        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)

        if use_diagnostics:
          self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

    def __del__(self):
        self._dashboard_agg_sub.unregister()


    def setup_extra_layout(self):
      return None

    def on_timer(self, evt):
      # fuerte does not have self._diagnostics_frame._diagnostics_panel
      if self._diagnostics_button:
        if self._diagnostics_frame._diagnostics_panel is not None:
          level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
        else:
          level = self._diagnostics_frame.get_top_level_state()

        if (level == -1 or level == 3):
          if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
        elif (level >= 2):
          if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
        elif (level == 1):
          if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
        else:
          if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))

      if self._rosout_button:
        self.update_rosout()

      if self._last_dashboard_message_time and (rospy.get_time() - self._last_dashboard_message_time > 5.0):
          self._motors_button.set_stale()
          ctrls = [self._motors_button]
          if self._battery_state_ctrl:
            self._battery_state_ctrl.set_stale()
            ctrls.append(self._battery_state_ctrl)

          for ctrl in ctrls:
              ctrl.SetToolTip(wx.ToolTip("No message received on dashboard_agg in the last 5 seconds"))

      if (rospy.is_shutdown()):
        self.Close()

    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()

    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()

    def on_log_clicked(self, evt):
      hrpsys_save = rospy.ServiceProxy("/DataLoggerServiceROSBridge/save", OpenHRP_DataLoggerService_save )
      try:
        name = "/tmp/rtclog-" + time.strftime("%Y%m%d%H%M%S")
        print "Writing log data to ",name
        hrpsys_save(OpenHRP_DataLoggerService_saveRequest(name))
        print "Done writing",name
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to save rtcd log: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def on_motors_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_servo_on, menu.Append(wx.ID_ANY, "Servo On"))
      menu.Bind(wx.EVT_MENU, self.on_servo_off, menu.Append(wx.ID_ANY, "Servo Off"))
      self._motors_button.toggle(True)
      self.PopupMenu(menu)
      self._motors_button.toggle(False)


    # from hrpsys/idl/RobotHardwareService.idl
    # enum SwitchStatus {SWITCH_ON, SWITCH_OFF};

    def on_servo_on(self, evt):
      servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      actual = rospy.ServiceProxy("/StateHolderServiceROSBridge/goActual", OpenHRP_StateHolderService_goActual )
      try:
        power(OpenHRP_RobotHardwareService_powerRequest("all",0))
        time.sleep(1)
        actual(OpenHRP_StateHolderService_goActualRequest())
        time.sleep(2)
        servo(OpenHRP_RobotHardwareService_servoRequest("all",0))
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the hrpsys in servo on mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def on_servo_off(self, evt):
      servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      try:
        servo(OpenHRP_RobotHardwareService_servoRequest("all",1));
        time.sleep(1)
        power(OpenHRP_RobotHardwareService_powerRequest("all",1))
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to put the hrpsys in servo off mode: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

    def dashboard_callback(self, msg):
      wx.CallAfter(self.new_dashboard_message, msg)

    def new_dashboard_message(self, msg):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()

      battery_status = {}
      op_mode = None
      pw_mode = None
      log_mode = None
      for status in msg.status:
          if status.name == "/Power System/Battery":
              for value in status.values:
                  battery_status[value.key]=value.value
          if status.name == "/Mode/Operating Mode":
              op_mode=status.message
          if status.name == "/Mode/Power Mode":
              pw_mode=status.message
          if status.name == "/Hrpsys/hrpEC Profile (log)":
              log_mode = status.message

      if self._battery_state_ctrl:
        if (battery_status):
          self._battery_state_ctrl.set_power_state(battery_status)
        else:
          self._battery_state_ctrl.set_stale()

      if (op_mode):
        if (op_mode=='Servo On'):
          if (self._motors_button.set_ok()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Servo On"))
        elif(op_mode=='Servo Off'):
          if (self._motors_button.set_error()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Servo Off"))
        else:
          if (self._motors_button.set_stale()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Stale"))
      else:
          if (self._motors_button.set_stale()):
              self._motors_button.SetToolTip(wx.ToolTip("Mode: Stale"))

      if (pw_mode):
        if (pw_mode=='Power On'):
          if (self._power_button.set_ok()):
              self._power_button.SetToolTip(wx.ToolTip("Mode: Power On"))
        elif(pw_mode=='Power Off'):
          if (self._power_button.set_error()):
              self._power_button.SetToolTip(wx.ToolTip("Mode: Power Off"))
      else:
          if (self._power_button.set_stale()):
              self._power_button.SetToolTip(wx.ToolTip("Mode: Stale"))

      if (log_mode):
        if (re.search("^Running", log_mode)):
          self._log_button.set_ok()
        else:
          self._log_button.set_error()
      else:
        self._log_button.set_stale()

    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0

      if (summary_dur < 0):
          summary_dur = 0.0

      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)

      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()


      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)

      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip

      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))

    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)

      self.SetPosition((x, y))
      self.SetSize((width, height))

    def save_config(self):
      config = self._config

      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)

      config.Flush()

    def on_close(self, event):
      self.save_config()

      self.Destroy()

