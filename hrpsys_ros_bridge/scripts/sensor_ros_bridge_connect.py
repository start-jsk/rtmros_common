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

def connecSensorRosBridgePort(url, rh, bridge, vs, rmfo, sh, es, rfu, subscription_type = "new", push_policy = 'all', push_rate = 50.0):
    print program_name, "connecSensorRosBridgePort(", url, ",", rh.name(), ")"
    for sen in hcf.getSensors(url):
        print program_name, "sensor(name: ", sen.name, ", type:", sen.type, ")"
        if sen.type in ['Acceleration', 'RateGyro', 'Force']:
            print program_name, "rh.port(", sen.name, ") = ", rh.port(sen.name)
            if rh.port(sen.name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                print program_name, "connect ", sen.name, rh.port(sen.name).get_port_profile().name, bridge.port(sen.name).get_port_profile().name
                connectPorts(rh.port(sen.name), bridge.port(sen.name), subscription_type, rate=push_rate, pushpolicy=push_policy)
                if sen.type == 'Force' and rmfo != None:
                    print program_name, "connect ", sen.name, rmfo.port("off_" + sen.name).get_port_profile().name, bridge.port("off_" + sen.name).get_port_profile().name
                    connectPorts(rmfo.port("off_" + sen.name), bridge.port("off_" + sen.name), subscription_type, rate=push_rate, pushpolicy=push_policy) # for abs forces
                if sen.type == 'Force' and bridge.port("ref_" + sen.name): # for reference forces
                    if rfu != None and rfu.port("ref_"+sen.name+"Out"):
                        print program_name, "connect ", sen.name, rfu.port("ref_"+sen.name+"Out").get_port_profile().name, bridge.port("ref_" + sen.name).get_port_profile().name
                        connectPorts(rfu.port("ref_"+sen.name+"Out"), bridge.port("ref_" + sen.name), subscription_type, rate=push_rate, pushpolicy=push_policy)
                    elif es != None and es.port(sen.name+"Out"):
                        print program_name, "connect ", sen.name, es.port(sen.name+"Out").get_port_profile().name, bridge.port("ref_" + sen.name).get_port_profile().name
                        connectPorts(es.port(sen.name+"Out"), bridge.port("ref_" + sen.name), subscription_type, rate=push_rate, pushpolicy=push_policy)
                    elif sh.port(sen.name+"Out"):
                        print program_name, "connect ", sen.name, sh.port(sen.name+"Out").get_port_profile().name, bridge.port("ref_" + sen.name).get_port_profile().name
                        connectPorts(sh.port(sen.name+"Out"), bridge.port("ref_" + sen.name), subscription_type, rate=push_rate, pushpolicy=push_policy)
        else:
            continue
    if vs != None:
        for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, vs.ports.keys()):
            print program_name, "connect ", vfp, vs.port(vfp).get_port_profile().name, bridge.port(vfp).get_port_profile().name
            connectPorts(vs.port(vfp), bridge.port(vfp), subscription_type, rate=push_rate, pushpolicy=push_policy)
            print program_name, "connect ", vfp, sh.port(vfp+"Out").get_port_profile().name, bridge.port("ref_"+vfp).get_port_profile().name
            connectPorts(sh.port(vfp+"Out"), bridge.port("ref_" + vfp), subscription_type, rate=push_rate, pushpolicy=push_policy) # for reference forces

def initSensorRosBridgeConnection(url, simulator_name, rosbridge_name, managerhost, subscription_type, push_policy, push_rate):
    print program_name, "initSensorRosBridgeConnection"
    hcf.waitForModelLoader()
    hcf.waitForRTCManagerAndRoboHardware(simulator_name, managerhost)
    bridge = rtm.findRTC(rosbridge_name)
    while bridge == None :
        time.sleep(1);
        bridge = rtm.findRTC(rosbridge_name)
        print program_name, " wait for ", rosbridge_name, " : ",bridge
    sh=rtm.findRTC('sh')
    while sh == None :
        time.sleep(1);
        sh=rtm.findRTC('sh')
        print program_name, " wait for 'sh' : ",sh

    while sh.isInactive(): ## wait for initalizing all components
        time.sleep(1)
        sh=rtm.findRTC('sh')

    vs=rtm.findRTC('vs')
    rmfo=rtm.findRTC('rmfo')
    es=rtm.findRTC('es')
    rfu=rtm.findRTC('rfu')
    connecSensorRosBridgePort(url, hcf.rh, bridge, vs, rmfo, sh, es, rfu, subscription_type, push_policy, push_rate)

if __name__ == '__main__':
    print program_name, "start"
    hcf=HrpsysConfigurator(program_name)
    if len(sys.argv) > 3 :
        initSensorRosBridgeConnection(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
    else :
        print program_name, " requires url, simulator_name, rosbridge_name"

