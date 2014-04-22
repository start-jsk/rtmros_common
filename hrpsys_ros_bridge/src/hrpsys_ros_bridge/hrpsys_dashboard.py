#!/usr/bin/env python

__doc__ = """
Hrpsys_Dashboard
GUI for the hrpsys robots."""

import imp
try:
    imp.find_module('hrpsys_ros_bridge')
except:
    import roslib; roslib.load_manifest('hrpsys_ros_bridge')

import diagnostic_msgs.msg

import rospy

import hrpsys_ros_bridge
from hrpsys_ros_bridge.srv import *
import time

####### rqt

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget
from rqt_robot_dashboard.widgets import MenuDashWidget

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from subprocess import check_call, Popen

# subclasses required for hrpsys_dasboard
class HrpsysLogMenu(MenuDashWidget):
  """
  a button to download log of hrpsys
  """
  def __init__(self):
    base_icon = '/usr/share/icons/Humanity/actions/32/document-new.svg'
    icons = [['bg-grey.svg', base_icon],
             ['bg-green.svg', base_icon],
             ['bg-red.svg', base_icon]]
    super(HrpsysLogMenu, self).__init__('hrpsys log', icons)
    self.update_state(0)
    self.add_action('download rtm log', self.on_download)
    self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
  def on_download(self):
    try:
      hrpsys_save = rospy.ServiceProxy("/DataLoggerServiceROSBridge/save", OpenHRP_DataLoggerService_save )
      name = "/tmp/rtclog-" + time.strftime("%Y%m%d%H%M%S")
      print "Writing log data to ",name
      hrpsys_save(OpenHRP_DataLoggerService_saveRequest(name))
      print "Done writing",name
    except rospy.ServiceException, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error",
                       "Failed to save rtcd log: service call failed with error: %s"%(e),
                       QMessageBox.Ok, self.parent())
      mb.exec_()
    except Exception, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error", str(e), QMessageBox.Ok, self.parent())
      mb.exec_()


class HrpsysRTCDMenu(MenuDashWidget):
  """
  a button to start/stop rtcd
  """
  def __init__(self, start_command, stop_command):
    icon_path = '/usr/share/icons/Humanity/actions/32/gnome-app-install-star.svg'
    icons = [['bg-grey.svg', icon_path],
             ['bg-green.svg', icon_path],
             ['bg-red.svg', icon_path]]
    super(HrpsysRTCDMenu, self).__init__('rtcd', icons)
    self.update_state(0)
    self.start_command = start_command
    self.stop_command = stop_command
    if self.start_command:
      self.add_action('start rtcd', self.on_start_rtcd)
    if self.stop_command:
      self.add_action('stop rtcd', self.on_stop_rtcd)
    self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
  def on_start_rtcd(self):
    Popen(['bash', '-c', self.start_command])
  def on_stop_rtcd(self):
    Popen(['bash', '-c', self.stop_command])


class HrpsysROSBridgeMenu(MenuDashWidget):
  """
  a button to start/stop rosbridge
  """
  def __init__(self, rosbridge_start_command, rosbridge_stop_command):
    icon_path = '/usr/share/icons/Humanity/apps/32/application-community.svg'
    icons = [['bg-grey.svg', icon_path],
             ['bg-green.svg', icon_path],
             ['bg-red.svg', icon_path]]
    super(HrpsysROSBridgeMenu, self).__init__('hrpsys_ros_bridge', icons)
    self.update_state(0)
    self.rosbridge_start_command = rosbridge_start_command
    self.rosbridge_stop_command = rosbridge_stop_command
    if self.rosbridge_start_command:
      self.add_action('start hrpsys_ros_bridge', self.on_start_ros_bridge)
    if self.rosbridge_stop_command:
      self.add_action('stop hrpsys_ros_bridge', self.on_stop_ros_bridge)
    self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
  def on_start_ros_bridge(self):
    Popen(['bash', '-c', self.rosbridge_start_command])
  def on_stop_ros_bridge(self):
    Popen(['bash', '-c', self.rosbridge_stop_command])

class HrpsysPowerMenu(MenuDashWidget):
  """
  a button to power on/off
  """
  def __init__(self):
    icons = [['bg-grey.svg', 'ic-runstop-off.svg'],
             ['bg-green.svg', 'ic-runstop-on.svg'],
             ['bg-red.svg', 'ic-runstop-off.svg']]
    super(HrpsysPowerMenu, self).__init__('power on/off', icons)
    self.update_state(0)
    self.add_action('Power On', self.on_power_on)
    self.add_action('Power Off', self.on_power_off)
    self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
  def on_power_on(self):
    try:
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      power(OpenHRP_RobotHardwareService_powerRequest("all",0))
    except Exception, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error",
                       "Failed to power on: %s"%(e),
                       QMessageBox.Ok, self.parent())
      mb.exec_()
  def on_power_off(self):
    try:
      power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
      power(OpenHRP_RobotHardwareService_powerRequest("all",1))
    except Exception, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error",
                       "Failed to power off: %s"%(e),
                       QMessageBox.Ok, self.parent())
      mb.exec_()


class HrpsysServoMenu(MenuDashWidget):
  """
  a button to servo on/off
  """
  def __init__(self, start_command, stop_command):
    icons = [['bg-grey.svg', 'ic-wireless-runstop-off.svg'],
             ['bg-green.svg', 'ic-wireless-runstop-on.svg'],
             ['bg-red.svg', 'ic-wireless-runstop-off.svg']]
    super(HrpsysServoMenu, self).__init__('servo on/off', icons)
    self.update_state(0)
    self.start_command = start_command
    self.stop_command = stop_command
    self.add_action('Servo On', self.on_servo_on)
    self.add_action('Servo Off', self.on_servo_off)
    self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
  def on_servo_on(self):
    try:
      if self.start_command:
        Popen(['bash', '-c', self.start_command])
      else:
        servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
        power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
        actual = rospy.ServiceProxy("/StateHolderServiceROSBridge/goActual", OpenHRP_StateHolderService_goActual )
        power(OpenHRP_RobotHardwareService_powerRequest("all",0))
        time.sleep(1)
        actual(OpenHRP_StateHolderService_goActualRequest())
        time.sleep(2)
        servo(OpenHRP_RobotHardwareService_servoRequest("all",0))
    except Exception, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error",
                       "Failed to servo on: %s"%(e),
                       QMessageBox.Ok, self.parent())
      mb.exec_()
  def on_servo_off(self):
    try:
      if self.stop_command:
        Popen(['bash', '-c', self.stop_command])
      else:
        power = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/power", OpenHRP_RobotHardwareService_power )
        servo = rospy.ServiceProxy("/RobotHardwareServiceROSBridge/servo", OpenHRP_RobotHardwareService_servo )
        servo(OpenHRP_RobotHardwareService_servoRequest("all", 1))
        time.sleep(1)
        power(OpenHRP_RobotHardwareService_powerRequest("all",1))
    except Exception, e:
      mb = QMessageBox(QMessageBox.NoIcon, "Error",
                       "Failed to servo off: %s"%(e),
                       QMessageBox.Ok, self.parent())
      mb.exec_()
    
class HrpsysDashboard(Dashboard):
  """
  Dashbaord for hrpsys.
  it includes:
    * diagnostics
    * rosout
    * log download
    * power
    * servo
    * rtcd
    * ros_bridge
    * battery
  """
  def setup(self, context):
    self.name = "hrpsys dashbaord"
    self._console = ConsoleDashWidget(self.context, minimal=False)
    self._monitor = MonitorDashWidget(self.context)
    self._log = HrpsysLogMenu()
    rtcd_start_command = rospy.get_param('hrpsys_rtcd_start_command', None)
    rtcd_stop_command = rospy.get_param('hrpsys_rtcd_stop_command', None)
    if rtcd_stop_command or rtcd_start_command:
      self._rtcd = HrpsysRTCDMenu(rtcd_start_command, rtcd_stop_command)
    else:
      self._rtcd = None
    ros_bridge_start_command = rospy.get_param('hrpsys_ros_bridge_start_command', None)
    ros_bridge_stop_command = rospy.get_param('hrpsys_ros_bridge_stop_command', None)
    if ros_bridge_stop_command or ros_bridge_start_command:
      self._ros_bridge = HrpsysROSBridgeMenu(ros_bridge_start_command, ros_bridge_stop_command)
    else:
      self._ros_bridge = None
    self._power = None
    #self._power = HrpsysPowerMenu()
    self._servo = HrpsysServoMenu(rospy.get_param('hrpsys_servo_start_command', None),
                                  rospy.get_param('hrpsys_servo_stop_command', None))
    self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)
  def get_widgets(self):
    widgets = []
    if self._rtcd:
      widgets.append(self._rtcd)
    if self._ros_bridge:
      widgets.append(self._ros_bridge)
    if self._power:
      widgets.append(self._power)
    if self._servo:
      widgets.append(self._servo)
    return [[self._monitor, self._console, self._log], widgets]
  def dashboard_callback(self, msg):
    servo_mode = None
    power_mode = None
    log_mode = None
    for status in msg.status:
      if status.name == "/Mode/Operating Mode":
        servo_mode = status.message
      if status.name == "/Mode/Power Mode":
        power_mode = status.message
      if status.name == "/Hrpsys/hrpEC Profile (log)":
        log_mode = status.message
    if servo_mode:
      if servo_mode == 'Servo On':
        self._servo.update_state(1)
      elif servo_mode == 'Servo Off':
        self._servo.update_state(2)
    else:
      self._servo.update_state(0)

    if self._power:
      if power_mode:
        if power_mode == 'Power On':
          self._power.update_state(1)
        elif power_mode =='Power Off':
          self._power.update_state(2)
      else:
        self._power.update_state(0)

    if log_mode:
      if re.search("^Running", log_mode):
        self._log.update_state(1)
      else:
        self._log.update_state(2)
    else:
      self._log.update_state(0)
      
          
  def shutdown_dashboard(self):
    self._dashboard_agg_sub.unregister()

  def save_settings(self, plugin_settings, instance_settings):
    self._console.save_settings(plugin_settings, instance_settings)
    self._monitor.save_settings(plugin_settings, instance_settings)

  def restore_settings(self, plugin_settings, instance_settings):
    self._console.restore_settings(plugin_settings, instance_settings)
    self._monitor.restore_settings(plugin_settings, instance_settings)

