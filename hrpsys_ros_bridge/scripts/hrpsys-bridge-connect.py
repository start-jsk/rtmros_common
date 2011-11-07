
import time
import rtm;
from rtm import *;
from OpenHRP import *;

conn = [['StateHolder0', 'qOut',
         'HrpsysSeqStateROSBridge0', 'rsangle'],
        ['HrpsysSeqStateROSBridge0', 'SequencePlayerService',
         'seq', 'SequencePlayerService'],
        ['StateHolder0', 'basePoseOut',
         'HrpsysSeqStateROSBridge0', 'pose']]

time.sleep(20)

for node1, port1, node2, port2 in conn :
    connectPorts(findRTC(node1).port(port1),
                 findRTC(node2).port(port2), subscription='new')

