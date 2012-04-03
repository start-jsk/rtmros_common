"""
Mainly copied from "rtmros_common/hrpsys_ros_bridge/scripts/hrpsys-bridge-connect.py", due to the connection failue of rtshell and realtime loop problem

"""
import os,time
import rtm

from rtm import *
from OpenHRP import *

def myinitCORBA(nshost):
    global rootnc
    args = string.split('-ORBInitRef NameService=corbaloc:iiop:'+ nshost +':2809/NameService')
    props = System.getProperties()
    orb = ORB.init(args, props)

    nameserver = orb.resolve_initial_references("NameService")
    rootnc = NamingContextHelper.narrow(nameserver)


if __name__ == '__main__':
    myinitCORBA(os.getenv("RTCTREE_NAMESERVERS","localhost"))

    time.sleep(10)

    #who wants to parse from launch file
    conn = [['Odometry0', 'ResultLocalization',
             'MobileRobotROSBridge0', 'in'],
            ['BeegoController0', 'TargetVelocity',
             'MobileRobotROSBridge0', 'out']]

    for node1, port1, node2, port2 in conn:
        print [node1,port1,node2,port2]
        connectPorts(findRTC(node1,rootnc).port(port1),
                     findRTC(node2,rootnc).port(port2), subscription='flush')

    beego = findRTC("MobileRobotROSBridge0")
    beego.start()
