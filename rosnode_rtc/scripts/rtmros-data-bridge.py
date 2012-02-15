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

# rtm modules
import RTC
import OpenRTM_aist

module_spec = ["implementation_id", "RTMROSDataBridge",
               "type_name",         "RTMROSDataBridge",
               "description",       "Dataport ROS bridge component",
               "version",           "1.0",
               "vendor",            "Manabu Saito",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]

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
        input_topic  = rospy.get_param('~input_topic','').split(' ')
        output_topic = rospy.get_param('~output_topic','').split(' ')
        self.update_ports(input_topic, output_topic)
        rospy.loginfo('onInitialize Finished')
        return RTC.RTC_OK

    def update_ports(self, in_topic, out_topic):

        for topic in in_topic:
            if topic in self.outports.keys(): continue
            if not (topic in idlman.topicinfo.keys()): continue
            topic_type = idlman.topicinfo[topic]

            port = topic.lstrip('/').replace('/','_')
            _data = idlman.get_rtmobj(topic_type)
            _outport = OpenRTM_aist.OutPort(port, _data)
            self.registerOutPort(port, _outport)
            self.outports[topic] = (_outport, _data)
            self.add_sub(topic)
            rospy.loginfo('Add OutPort %s' % port)

        for topic in out_topic:
            if topic in self.inports.keys(): continue
            if not (topic in idlman.topicinfo.keys()): continue
            topic_type = idlman.topicinfo[topic]

            port = topic.lstrip('/').replace('/','_')
            _data = idlman.get_rtmobj(topic_type)
            _inport = OpenRTM_aist.InPort(port, _data)
            self.registerInPort(port, _inport)
            self.inports[topic] = (_inport, _data)
            self.add_pub(topic)
            rospy.loginfo('Add InPort %s' % port)

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
        for msgtype in self.msg2obj.keys():
            # TODO incomplete type estimation
            rosobj = self.msg2obj[msgtype][0]()
            rtmslot = [s for s in dir(data) if s[0]!='_']
            if set(rosobj.__slots__) == set(rtmslot):
                output = rosobj
                break
        if not output: return None
        for slot in output.__slots__:
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
                    arg = [self.rtm2ros(da) for da in list(var)]
            elif typ == RTC.Time:
                arg = roslib.rostime.Time(var.secs, var.nsecs)
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
            elif typ == roslib.rostime.Time:
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
        return self.ros2rtm(self.msg2obj[msg][0]())

    def update_idl(self, msgnames=[]):
        for msg in msgnames:
            self.add_msg_definition(msg)
            rospy.loginfo('Data[%s] in RTC = "%s"', msg, self.get_rtmobj(msg))

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
            return None

        defmsg = self.get_message_definition(msg)

        # add inner message
        for var in defmsg.split('\n'):
            if len(var.split(' ')) == 2:
                typ = var.split(' ')[0]
                if typ.find('[') != -1:
                    typ = typ[:typ.find('[')]
                self.gen_struct_text(typ)

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

    def add_msg_definition(self, msg):
        if not self.gen_struct_text(msg):
            return None

        # generate idl module
        program = ' '.join(sys.argv)
        base = """// This IDL is generated by '%s'
#include "BasicDataType.idl"
module RTMROSDataBridge
{%s};
""" % (program, self.idltext)
        open(self.idldir+'/'+self.idlfile, 'w').write(base)

        genidl_bin = Popen(['rosrun','openrtm','rtm-config','--idlc'], stdout=PIPE, stderr=PIPE).communicate()[0]
        rtmpkg = Popen(['rosrun','openrtm','rtm-config','--prefix'], stdout=PIPE, stderr=PIPE).communicate()[0]
        genidl_bin = genidl_bin.rstrip('\n')
        rtmpkg = rtmpkg.rstrip('\n')

        call([genidl_bin, '-bpython', '-I%s/include/rtm/idl'%rtmpkg, '-C'+self.idldir, self.idldir+'/'+self.idlfile])

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

        return True

#
def RTMROSDataBridgeInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    manager.registerFactory(profile,
                            RtmRosDataBridge,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent("RTMROSDataBridge")

if __name__ == '__main__':
    rospy.init_node('rtmrosbridge')
    idl_dir  = rospy.get_param('~idl_dir','/tmp')
    global idlman
    idlman = RtmRosDataIdl(idl_dir)
    input_topic  = rospy.get_param('~input_topic','').split(' ')
    output_topic = rospy.get_param('~output_topic','').split(' ')
    idlman.update_topicinfo(input_topic+output_topic)

    # RTC Manager initialize
    conf = roslib.rospack.rospackexec(['find',rospackage])+'/scripts/rtc.conf'
    mgr = OpenRTM_aist.Manager.init(sys.argv[0:1] + ['-f',conf])

    # -o is not implemented (OpenRTM-Python)
    #nameserver = os.environ['RTCTREE_NAMESERVERS']
    #sys.argv += ['-o','naming.formats:%n.rtc',
    #             '-o','corba.nameservers:'+nameserver+':2809',
    #             '-o','logger.file_name:/tmp/rtc%p.log']

    mgr.setModuleInitProc(RTMROSDataBridgeInit)
    mgr.activateManager()
    mgr.runManager()
