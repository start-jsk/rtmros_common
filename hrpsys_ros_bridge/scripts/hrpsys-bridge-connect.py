import os
import time
import rtm
from rtm import *
from OpenHRP import *

if os.environ.has_key('RTCTREE_NAMESERVERS'):
    nshost = os.environ.get('RTCTREE_NAMESERVERS')
else:
    nshost = localhost

def myinitCORBA():
    global rootnc, nshost, orb
    props = System.getProperties()

    args = string.split('-ORBInitRef NameService=corbaloc:iiop:'+nshost+':2809/NameService')
    orb = ORB.init(args, props)

    nameserver = orb.resolve_initial_references("NameService")
    rootnc = NamingContextHelper.narrow(nameserver)
    return None

myinitCORBA()

conn = [['RobotHardware0', 'q',
         'HrpsysSeqStateROSBridge0', 'rsangle'],
        ['VideoStream0', 'MultiCameraImages',
         'ImageSensorROSBridge0', 'MultiCameraImages']]
#        ['HrpsysSeqStateROSBridge0', 'SequencePlayerService',
#         'seq', 'SequencePlayerService'],
#        ['StateHolder0', 'basePoseOut',
#         'HrpsysSeqStateROSBridge0', 'pose']]

time.sleep(10)

for node1, port1, node2, port2 in conn:
    print [node1,port1,node2,port2]
    connectPorts(findRTC(node1,rootnc).port(port1),
                 findRTC(node2,rootnc).port(port2), subscription='new')
