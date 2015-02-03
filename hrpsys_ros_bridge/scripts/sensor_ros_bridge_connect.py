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

def connecSensorRosBridgePort(url, rh, bridge, vs, rmfo, sh):
    for sen in hcf.getSensors(url):
        if sen.type in ['Acceleration', 'RateGyro', 'Force']:
            if rh.port(sen.name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                print program_name, "connect ", sen.name, rh.port(sen.name).get_port_profile().name, bridge.port(sen.name).get_port_profile().name
                connectPorts(rh.port(sen.name), bridge.port(sen.name), "new")
                if sen.type == 'Force' and rmfo != None:
                    print program_name, "connect ", sen.name, rmfo.port("off_" + sen.name).get_port_profile().name, bridge.port("off_" + sen.name).get_port_profile().name
                    connectPorts(rmfo.port("off_" + sen.name), bridge.port("off_" + sen.name), "new") # for abs forces
                if sen.type == 'Force' and sh.port(sen.name+"Out") and bridge.port("ref_" + sen.name):
                    print program_name, "connect ", sen.name, sh.port(sen.name+"Out").get_port_profile().name, bridge.port("ref_" + sen.name).get_port_profile().name
                    connectPorts(sh.port(sen.name+"Out"), bridge.port("ref_" + sen.name), "new") # for reference forces
        else:
            continue
    if vs != None:
        for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, vs.ports.keys()):
            print program_name, "connect ", vfp, vs.port(vfp).get_port_profile().name, bridge.port(vfp).get_port_profile().name
            connectPorts(vs.port(vfp), bridge.port(vfp), "new")
            print program_name, "connect ", vfp, sh.port(vfp+"Out").get_port_profile().name, bridge.port("ref_"+vfp).get_port_profile().name
            connectPorts(sh.port(vfp+"Out"), bridge.port("ref_" + vfp), "new") # for reference forces

def initSensorRosBridgeConnection(url, simulator_name, rosbridge_name, managerhost):
    hcf.waitForModelLoader()
    hcf.waitForRTCManagerAndRoboHardware(simulator_name, managerhost)
    bridge = None
    while bridge == None :
        time.sleep(1);
        bridge = rtm.findRTC(rosbridge_name)
        print program_name, " wait for ", rosbridge_name, " : ",bridge
    sh = None
    while sh == None :
        time.sleep(1);
        sh=rtm.findRTC('sh')
        print program_name, " wait for 'sh' : ",sh
    vs=rtm.findRTC('vs')
    rmfo=rtm.findRTC('rmfo')
    connecSensorRosBridgePort(url, hcf.rh, bridge, vs, rmfo, sh)

if __name__ == '__main__':
    print program_name, "start"
    hcf=HrpsysConfigurator(program_name)
    if len(sys.argv) > 3 :
        initSensorRosBridgeConnection(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    else :
        print program_name, " requires url, simulator_name, rosbridge_name"

