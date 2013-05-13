#!/usr/bin/python

# This program is used to connect RosBridge sensor ports with simulator or RobotHardware sensor ports based on ModelLoader information.

# use hrpsys/scripts/hrpsys.py file
import roslib;
import sys; sys.path.insert (0, roslib.packages.get_pkg_dir('hrpsys')+'/scripts');
from hrpsys import *

program_name = '[sensor_ros_bridge_connect.py] '

def connecSensorRosBridgePort(url, rh, bridge):
    for sen in hcf.getSensors(url):
        if sen.type in ['Acceleration', 'RateGyro', 'Force']:
            if rh.port(sen.name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                print program_name, "connect ", sen.name, rh.port(sen.name), bridge.port(sen.name)
                connectPorts(rh.port(sen.name), bridge.port(sen.name), "new")
        else:
            continue

def initSensorRosBridgeConnection(url, simulator_name, rosbridge_name, managerhost):
    hcf.waitForModelLoader()
    hcf.waitForRTCManagerAndRoboHardware(simulator_name, managerhost)
    bridge = None
    while bridge == None :
        time.sleep(1);
        bridge = rtm.findRTC(rosbridge_name)
        print program_name, " wait for ", rosbridge_name, " : ",bridge
    connecSensorRosBridgePort(url, hcf.rh, bridge)

if __name__ == '__main__':
    print program_name, "start"
    hcf=HrpsysConfigurator(program_name)
    if len(sys.argv) > 3 :
        initSensorRosBridgeConnection(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    else :
        print program_name, " requires url, simulator_name, rosbridge_name"

