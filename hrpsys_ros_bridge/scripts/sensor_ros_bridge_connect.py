#!/usr/bin/python

# This program is used to connect RosBridge sensor ports with simulator or RobotHardware sensor ports based on ModelLoader information.

try: # catkin does not requires load_manifest
    import hrpsys
except:
    import roslib; roslib.load_manifest('hrpsys')
    import hrpsys

import OpenRTM_aist.RTM_IDL # for catkin

from hrpsys.hrpsys_config import *
import OpenHRP

program_name = '[sensor_ros_bridge_connect.py] '

def connecSensorRosBridgePort(url, rh, bridge, vs, afs):
    for sen in hcf.getSensors(url):
        if sen.type in ['Acceleration', 'RateGyro', 'Force']:
            if rh.port(sen.name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                print program_name, "connect ", sen.name, rh.port(sen.name).get_port_profile().name, bridge.port(sen.name).get_port_profile().name
                connectPorts(rh.port(sen.name), bridge.port(sen.name), "new")
                if sen.type == 'Force' and afs != None:
                    print program_name, "connect ", sen.name, afs.port("off_" + sen.name).get_port_profile().name, bridge.port("off_" + sen.name).get_port_profile().name
                    connectPorts(afs.port("off_" + sen.name), bridge.port("off_" + sen.name), "new") # for abs forces
        else:
            continue
    if vs != None:
        for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, vs.ports.keys()):
            print program_name, "connect ", vfp, vs.port(vfp).get_port_profile().name, bridge.port(vfp).get_port_profile().name
            connectPorts(vs.port(vfp), bridge.port(vfp), "new")

def initSensorRosBridgeConnection(url, simulator_name, rosbridge_name, managerhost):
    hcf.waitForModelLoader()
    hcf.waitForRTCManagerAndRoboHardware(simulator_name, managerhost)
    bridge = None
    while bridge == None :
        time.sleep(1);
        bridge = rtm.findRTC(rosbridge_name)
        print program_name, " wait for ", rosbridge_name, " : ",bridge
    vs=rtm.findRTC('vs')
    afs=rtm.findRTC('afs')
    connecSensorRosBridgePort(url, hcf.rh, bridge, vs, afs)

if __name__ == '__main__':
    print program_name, "start"
    hcf=HrpsysConfigurator(program_name)
    if len(sys.argv) > 3 :
        initSensorRosBridgeConnection(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    else :
        print program_name, " requires url, simulator_name, rosbridge_name"

