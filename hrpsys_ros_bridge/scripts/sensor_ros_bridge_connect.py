#!/usr/bin/python

# This program is used to connect RosBridge sensor ports with simulator or RobotHardware sensor ports based on ModelLoader information.

# use hrpsys/scripts/hrpsys.py file
import roslib;
import sys; sys.path.insert (0, roslib.packages.get_pkg_dir('hrpsys')+'/scripts');
from hrpsys import *

program_name = '[sensor_ros_bridge_connect.py] '

def connecSensorRosBridgePort(url, rh, bridge):
    for sen in hcf.getSensors(url):
        print sen.name
        if sen.type == 'Acceleration':
            print program_name, "connect ", rh.port('gsensor'), bridge.port("gsensor")
            connectPorts(rh.port('gsensor'), bridge.port("gsensor"), "new")
        elif sen.type == 'RateGyro':
            print program_name, "connect ", rh.port('gyrometer'), bridge.port('gyrometer')
            connectPorts(rh.port('gyrometer'), bridge.port('gyrometer'), "new")
        elif sen.type == 'Force':
            print program_name, "connect ", rh.port(sen.name), bridge.port(sen.name)
            connectPorts(rh.port(sen.name), bridge.port(sen.name), "new")
        else:
            continue

def initSensorRosBridgeConnection(url, simulator_name, rosbridge_name):
    hcf.waitForModelLoader()
    hcf.waitForRTCManagerAndRoboHardware(simulator_name)
    bridge = None
    while bridge == None :
        time.sleep(1);
        bridge = rtm.findRTC(rosbridge_name)
        print program_name, " wait for ", rosbridge_name, " : ",bridge
    connecSensorRosBridgePort(url, hcf.rh, bridge)

if __name__ == '__main__':
    print program_name, "start"
    hcf=HrpsysConfigurator()
    if len(sys.argv) > 3 :
        initSensorRosBridgeConnection(sys.argv[1], sys.argv[2], sys.argv[3])
    else :
        print program_name, " requires url, simulator_name, rosbridge_name"

