#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

############################################################
# Usage
# . rosrun rtmbuild rtmros-data-bridge.py _input_topic:='TOPIC1 ..' _output_topic:='TOPIC2 ..' _idl_dir='/tmp'
# . rtprint -m RTMROSDataBridge -p /tmp /..../RTMROSDataBridge0.rtc:<topicname>

# python standard modules
import re
import os, sys, site
import math
from subprocess import *

# ros modules
rospackage='rosnode_rtc'
import roslib; roslib.load_manifest(rospackage)
import rospy
import rostopic
import rosnode
import socket
import xmlrpclib
import genpy

# rtm modules
import time
import RTC
import OpenRTM_aist

module_spec = ["implementation_id", "<implementation_id>",
               "type_name",         "<type_name>",
               "description",       "Dataport ROS bridge component",
               "version",           "1.0",
               "vendor",            "Manabu Saito",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]

# ROS topic type utility functions
def get_topicname_by_node(nodename):
    master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        state = rosnode._succeed(master.getSystemState(rospy.get_name()))
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    pubs = [t for t, l in state[0] if nodename in l]
    subs = [t for t, l in state[1] if nodename in l]
    return pubs, subs # in,out

def get_topic_names_by_param():
    input_topic  = rospy.get_param('~input_topic','').split(' ')
    output_topic = rospy.get_param('~output_topic','').split(' ')
    wrap_node = rospy.get_param('~wrap_node',None)
    if wrap_node:
        node_in, node_out = get_topicname_by_node(wrap_node)
        node_in  = [t for t in node_in if not (t in output_topic)]
        node_out = [t for t in node_out if not (t in input_topic)]
        return input_topic+node_in, output_topic+node_out
    return input_topic, output_topic

#
# RTM <-> ROS Dataport Bridge Component
#
class RtmRosDataBridge(OpenRTM_aist.DataFlowComponentBase):
    def __init__ (self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        self.pubs = {}     # topicname: publisher
        self.subs = {}     # topicname: subscriber
        self.inports = {}  # topicname: inport,  dataobj
        self.outports = {} # topicname: outport, dataobj

        return

    def onInitialize(self):
        input_topic, output_topic = get_topic_names_by_param()
        rospy.loginfo('onInitialize Start')
        self.update_ports(input_topic, output_topic)
        rospy.loginfo('onInitialize Finished')
        return RTC.RTC_OK

    def update_ports(self, in_topic, out_topic):

        for topic in out_topic:
            port = topic.lstrip('/').replace('/','_')
            topic_type = idlman.topicinfo.get(topic, None)
            _data = idlman.get_rtmobj(topic_type)
            if topic in self.inports.keys() or not topic_type or not _data:
                rospy.loginfo('Failed to add InPort "%s"', port)
                continue
            rospy.loginfo('Add InPort "%s"[%s]', port, topic_type)
            _inport = OpenRTM_aist.InPort(port, _data)
            self.registerInPort(port, _inport)
            self.inports[topic] = (_inport, _data)
            self.add_pub(topic)

        for topic in in_topic:
            port = topic.lstrip('/').replace('/','_')
            topic_type = idlman.topicinfo.get(topic, None)
            _data = idlman.get_rtmobj(topic_type)
            if (topic in self.outports.keys()) or not topic_type or not _data:
                rospy.loginfo('Failed to add OutPort "%s"', port)
                continue
            rospy.loginfo('Add OutPort "%s"[%s]', port, topic_type)
            _outport = OpenRTM_aist.OutPort(port, _data)
            self.registerOutPort(port, _outport)
            self.outports[topic] = (_outport, _data)
            self.add_sub(topic)

        return

    # InPort -> Publisher
    def onExecute(self, ec_id):
        for topic in self.inports.keys():
            _inport, _ = self.inports[topic]
            if _inport.isNew():
                _data = _inport.read()
                msg = idlman.rtm2ros(_data)
                if msg:
                    self.pubs[topic].publish(msg)
        return RTC.RTC_OK

    def add_pub(self, topic):
        topic_type = idlman.topicinfo[topic]
        if(topic_type == None):
            rospy.loginfo('%s is not published yet', topic)
            return None
        topic_class, _, _ = rostopic.get_topic_class(topic)
        if(topic_class == None):
            rospy.loginfo('%s is not builded yet', topic_type)
            return None
        self.pubs[topic] = rospy.Publisher(topic, topic_class)
        return True

    # Subscriber -> OutPort
    def WriteData(self, topicname, rosdata):
        port, rtmdata = self.outports[topicname]
        idlman.ros2rtm(rosdata, rtmdata)
        port.write()

    def add_sub(self, topic):
        topic_type = idlman.topicinfo[topic]
        if(topic_type == None):
            rospy.loginfo('%s is not published yet', topic)
            return None
        topic_class, _, _ = rostopic.get_topic_class(topic)
        if(topic_class == None):
            rospy.loginfo('%s is not builded yet', topic_type)
            return None

        cb = self.gen_callback(topic)
        self.subs[topic] = rospy.Subscriber(topic, topic_class, cb)
        return True
    def gen_callback(self, topic):
        return (lambda m: self.WriteData(topic, m))

#
# topic(ROS) -> idl(RTM) converter methods
#   `rosmsg show` -> .idl -> python module
#
class RtmRosDataIdl:
    def __init__(self, idldir, idlfile='rosbridge.idl'):
        self.ignore_unbound_type = rospy.get_param('~ignore_unbound',True)
        self.idldir  = idldir
        self.idlfile = idlfile
        site.addsitedir(self.idldir)

        self.generated = []
        self.idltext = ''
        self.basictab = {
            'int8'   : 'char',
            'uint8'  : 'octet',
            'int16'  : 'short',
            'uint16' : 'unsigned short',
            'int32'  : 'long',
            'uint32' : 'unsigned long',
            'int64'  : 'long',          # ??
            'uint64' : 'unsigned long', # ??
            'float32': 'float',
            'float64': 'double',
            'string' : 'string',
            'bool'   : 'boolean',
            'char'   : 'char',          # deplicated
            'byte'   : 'octet',         # deplicated
            'time'   : 'RTC::Time',     # builtin
            'duration' : 'RTC::Time'}   # builtin
        self.dummyslot = 'dummy_padding_' # for empty message

        self.topicinfo = {} # topicname: msgtype, msgclass
        self.msg2obj = {}   # msgname: -> (rostype, rtmtype)

        return None

    def update_topicinfo(self,topics):
        new_topic_type = []
        for topic in topics:
            topic_type, _, _ = rostopic.get_topic_type(topic)
            if topic_type:
                new_topic_type += [topic_type]
                self.topicinfo[topic] = topic_type
        self.update_idl(new_topic_type)

    def rtm2ros(self, data, output=None):
        datatype = str(data).split('(')[0].split('.')[1]
        for msgtype in self.msg2obj.keys():
            if datatype == msgtype.replace('/','_'):
                output = self.msg2obj[msgtype][0]()
                break
        if not output: return None
        for slot in output.__slots__:
            var = getattr(data, slot)
            ovar = getattr(output, slot)
            typ = type(var)
            if typ in [bool, int, long, float, str]:
                arg = var
            elif typ == list or typ == tuple:
                if len(var)==0:
                    arg = []
                elif type(var[0]) in [bool, int, long, float, str]:
                    arg = var
                else:
                    arg = [self.rtm2ros(da) for da in list(var)]
            elif type(ovar) in [genpy.rostime.Time, genpy.rostime.Duration]:
                arg = genpy.rostime.Time(var.sec, var.nsec)
            else:
                arg = self.rtm2ros(var)
            setattr(output, slot, arg)
        return output
    def ros2rtm(self, data, output=None):
        args = []
        for slot in data.__slots__:
            var = getattr(data, slot)
            typ = type(var)
            if typ in [bool, int, long, float, str]:
                arg = var
            elif typ == list or typ == tuple:
                if len(var)==0:
                    arg = []
                elif type(var[0]) in [bool, int, long, float, str]:
                    arg = var
                else:
                    arg = [self.ros2rtm(da) for da in list(var)]
            elif typ in [genpy.rostime.Time, genpy.rostime.Duration]:
                arg = RTC.Time(var.secs, var.nsecs)
            else:
                arg = self.ros2rtm(var)
            if output:
                setattr(output, slot, arg)
            else:
                args += [arg]
        if args == []:
            args += [''] # for dummy slot in empty message
        if output:
            return output
        else:
            return self.msg2obj[data._type][1](*args)

    def get_rtmobj(self, msg):
        if not msg in self.msg2obj.keys():
            return None
        return self.ros2rtm(self.msg2obj[msg][0]())

    def update_idl(self, msgnames=[]):
        for msg in msgnames:
            self.gen_struct_text(msg)
        self.reload_msg_definition()

    def get_message_definition(self, msg):
        text = Popen(['rosmsg','show',msg], stdout=PIPE, stderr=PIPE).communicate()[0]
        text = ''.join([l+'\n' for l in text.split('\n')
                          if not ((len(l) == 0) or (l[0] in ' [') or ('=' in l))])
        if text == '':
            text = 'string %s\n' % self.dummyslot
        p = re.compile( '(^|\n)Header[ ]')
        text = p.sub( r'\1std_msgs/Header ', text)
        return text

    def gen_struct_text(self, msg):
        if msg in (self.generated + self.basictab.keys()):
            return True

        defmsg = self.get_message_definition(msg)

        # ignore unbound type mode
        if self.ignore_unbound_type and '[]' in defmsg:
            return False

        # add inner message
        for var in defmsg.split('\n'):
            if len(var.split(' ')) == 2:
                typ = var.split(' ')[0]
                if typ.find('[') != -1:
                    typ = typ[:typ.find('[')]
                if not self.gen_struct_text(typ):
                    return False

        rospy.loginfo('Generating IDL for "%s"', msg)
        self.generated.append(msg)

        msg = msg.replace('/','_')
        defmsg = defmsg.replace('/','_')

        # for embedded types, sequence/array types
        for key in self.basictab.keys():
            p = re.compile( '(^|\n)'+key+'((\[[\d]*\])?[ ]+[\w]+)')
            defmsg = p.sub( r'\1'+self.basictab[key]+r'\2', defmsg)
        p = re.compile( '(^|\n)([ \w]+)\[([\d]+)\][ ]([\w]+)')
        defmsg = p.sub( r'\1sequence<\2, \3> \4', defmsg)
        p = re.compile( '(^|\n)([ \w]+)\[\][ ]([\w]+)')
        defmsg = p.sub( r'\1sequence<\2> \3', defmsg)

        defmsg = defmsg.replace('\n',';\n')

        self.idltext += '\n' + ("struct %s {\n%s};" % (msg, defmsg))
        return True

    def reload_msg_definition(self):
        # generate idl module
        program = ' '.join(sys.argv)
        base = """// This IDL is generated by '%s'
#include "BasicDataType.idl"
module RTMROSDataBridge
{%s};
""" % (program, self.idltext)
        open(self.idldir+'/'+self.idlfile, 'w').write(base)

        genidl_bin = Popen(['rosrun','openrtm_aist','rtm-config','--idlc'], stdout=PIPE, stderr=PIPE).communicate()[0]
        rtmpkg = Popen(['rosrun','openrtm_aist','rtm-config','--prefix'], stdout=PIPE, stderr=PIPE).communicate()[0]
        genidl_bin = genidl_bin.rstrip('\n')
        rtmpkg = rtmpkg.rstrip('\n')

        call([genidl_bin, '-bpython', '-I%s/include/openrtm-1.1/rtm/idl'%rtmpkg, '-C'+self.idldir, self.idldir+'/'+self.idlfile])
        time.sleep(1) # ??

        # reload RTC data object
        global RTMROSDataBridge
        import RTMROSDataBridge
        reload(RTMROSDataBridge)
        reload(RTMROSDataBridge.rosbridge_idl)

        # register ros/rtm data object type
        for msg in self.generated:
            rtm_typ = 'RTMROSDataBridge.' + (msg.replace('/','_'))
            ros_typ_tuple = tuple(msg.split('/'))
            ros_typ = '%s.msg.%s' % ros_typ_tuple
            roslib.load_manifest(ros_typ_tuple[0])
            exec('import ' + ros_typ_tuple[0] + '.msg')
            self.msg2obj[msg] = (eval(ros_typ), eval(rtm_typ))
            rospy.loginfo('Data[%s] in RTC = "%s"', msg, self.get_rtmobj(msg))

        return True

#
def RTMROSDataBridgeInit(manager):
    nodename = rospy.get_param('~wrap_node','RTMROSDataBridge')
    compname = nodename.lstrip('/').replace('/','_')
    module_spec[module_spec.index('<implementation_id>')] = compname
    module_spec[module_spec.index('<type_name>')] = compname

    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    manager.registerFactory(profile,
                            RtmRosDataBridge,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent(compname)

if __name__ == '__main__':
    rospy.init_node('rtmrosbridge')
    idl_dir  = rospy.get_param('~idl_dir','/tmp')
    global idlman
    idlman = RtmRosDataIdl(idl_dir)

    input_topic, output_topic = get_topic_names_by_param()
    idlman.update_topicinfo(input_topic+output_topic)

    # RTC Manager initialize
    if OpenRTM_aist.version.openrtm_version == '1.0.0':
        # -o is not implemented (OpenRTM-Python-1.0.0)
        conf = roslib.rospack.rospackexec(['find',rospackage])+'/scripts/rtc.conf'
        sys.argv += ['-f',conf]

    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(RTMROSDataBridgeInit)
    mgr.activateManager()
    mgr.runManager()
