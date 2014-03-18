#!/usr/bin/env python

import roslib
roslib.load_manifest('openrtm_tools')

import sys
import os
import time
import optparse

from xml.dom.minidom import parse

import rtctree
from rtshell import rtcon
from rtshell import path
from rtshell import state_control_base
from rtshell import rts_exceptions

def alive_component(path):
    tree=rtctree.tree.RTCTree()
    if tree.has_path(path) and tree.is_component(path):
        node = tree.get_node(path)
        return node.plain_state_string
    else:
        return False

def wait_component(cmd_path):
    count=0
    path = rtctree.path.parse_path(cmd_path)[0]
    node = alive_component(path)
    while not node and count < 30:
        node = alive_component(path)
        print >>sys.stderr, "\033[33m[rtmlaunch] Wait for ",cmd_path," ",count,"/30\033[0m"
        count += 1
        time.sleep(1)
    if not node:
        raise rts_exceptions.NoSuchObjectError(cmd_path)
    return node

def check_connect(src_path, dest_path):
    tree=rtctree.tree.RTCTree()
    src_path, src_port = rtctree.path.parse_path(src_path)
    dest_path, dest_port = rtctree.path.parse_path(dest_path)
    src_node = tree.get_node(src_path)
    dest_node = tree.get_node(dest_path)
    port = src_node.get_port_by_name(src_port)
    for conn in port.connections:
        for name, p in conn.ports:
            tmp_dest_path, tmp_dest_port = rtctree.path.parse_path(name)
            if dest_path[-1] == tmp_dest_path[-1] and dest_port == tmp_dest_port:
                return True
    return False

def rtconnect(nameserver, tags):
    import re
    for tag in tags:

        # check if/unless attribute in rtconnect tags
        exec_flag = True
        if tag.attributes.get(u'if'):
            val = tag.attributes.get(u'if').value
            arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
            exec_flag =  get_flag_from_argv(arg)
        if tag.attributes.get(u'unless'):
            val = tag.attributes.get(u'unless').value
            arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
            exec_flag = not get_flag_from_argv(arg)
        if not exec_flag:
            continue

        source_path = nameserver+"/"+tag.attributes.get("from").value
        dest_path   = nameserver+"/"+tag.attributes.get("to").value
        source_path = re.sub("\$\(arg SIMULATOR_NAME\)",simulator,source_path);
        dest_path = re.sub("\$\(arg SIMULATOR_NAME\)",simulator,dest_path);
        # print >>sys.stderr, "[rtmlaunch] Connecting from %s to %s"%(source_path,dest_path)
        source_full_path = path.cmd_path_to_full_path(source_path)
        dest_full_path = path.cmd_path_to_full_path(dest_path)
        if tag.attributes.get("subscription_type") != None:
            sub_type = tag.attributes.get("subscription_type").value
            if not sub_type in ['flush','new','periodic']:
                print >>sys.stderr, sub_type+' is not a subscription type'
                continue
        else:
            sub_type = 'flush' # this is default value
        if sub_type == 'new':
            push_policy = 'all'
        # wait for proess
        try:
            wait_component(source_full_path)
            wait_component(dest_full_path)
            if check_connect(source_full_path,dest_full_path):
                continue
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] [ERROR] Could not Connect (', source_full_path, ',', dest_full_path, '): ', e,'\033[0m'
            return 1
        #print source_path, source_full_path, dest_path, dest_full_path;
        try:
            sub_type = str(sub_type)
            props = {'dataport.subscription_type': sub_type}
            if sub_type == 'new':
                props['dataport.publisher.push_policy'] = 'all'
            elif sub_type == 'periodic':
                props['dataport.publisher.push_policy'] = 'all'
                if tag.attributes.get("push_rate") != None:
                    props['dataport.push_rate'] = str(tag.attributes.get("push_rate").value)
                else:
                    props['dataport.push_rate'] = str('50.0')
            options = optparse.Values({'verbose': False, 'id': '', 'name': None, 'properties': props})
            print >>sys.stderr, "[rtmlaunch] Connected from",source_path
            print >>sys.stderr, "[rtmlaunch]             to",dest_path
            print >>sys.stderr, "[rtmlaunch]           with",options
            try :
                rtcon.connect_ports(source_path, source_full_path, dest_path, dest_full_path, options, tree=None)
            except Exception, e_1_1_0: # openrtm 1.1.0
                try:
                    rtcon.connect_ports([(source_path,source_full_path), (dest_path, dest_full_path)], options, tree=None)
                except Exception, e_1_0_0: # openrtm 1.0.0
                    print >>sys.stderr, '\033[31m[rtmlaunch] {0} did not work on both OpenRTM 1.0.0 and 1.1.0'.format(os.path.basename(sys.argv[1])),'\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]    OpenRTM 1.0.0 {0}'.format(e_1_0_0),'\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]    OpenRTM 1.1.0 {0}'.format(e_1_1_0),'\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]  This is very weird situation, Please check your network\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch] configuration with `ifconfig` on both robot and client side. \033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]  Having multiple network interface sometimes causes problem, \033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch] please see FAQ site http://www.openrtm.org/OpenRTM-aist/html/FAQ2FE38388E383A9E38396E383ABE382B7E383A5E383BCE38386E382A3E383B3E382B0.html#f2bc375d\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]            Issue related to this https://github.com/start-jsk/rtmros_hironx/issues/33\033[0m'
                    print >>sys.stderr, '\033[31m[rtmlaunch]            ~/.ros/log may contains usefully informations\033[0m'
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[1]), e),'\033[0m'
    return 0

def rtactivate(nameserver, tags):
    def activate_action(object, ec_index):
        object.activate_in_ec(ec_index)
    for tag in tags:
        
        # check if/unless attribute in rtactivate tags
        exec_flag = True
        if tag.attributes.get(u'if'):
            val = tag.attributes.get(u'if').value
            arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
            exec_flag =  get_flag_from_argv(arg)
        if tag.attributes.get(u'unless'):
            val = tag.attributes.get(u'unless').value
            arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
            exec_flag = not get_flag_from_argv(arg)
        if not exec_flag:
            continue

        cmd_path  = nameserver+"/"+tag.attributes.get("component").value
        full_path = path.cmd_path_to_full_path(cmd_path)
        # print >>sys.stderr, "[rtmlaunch] activate %s"%(full_path)
        try:
            state = wait_component(full_path)
            if state == 'Active':
                continue
            else:
                print >>sys.stderr, "[rtmlaunch] Activate <-",state,full_path
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] Could not Activate (', cmd_path, ') : ', e,'\033[0m'
            return 1
        try:
            options = optparse.Values({"ec_index": 0, 'verbose': False})
            try :
                state_control_base.alter_component_state(activate_action, cmd_path, full_path, options, None)
            except Exception, e: # openrtm 1.1.0
                state_control_base.alter_component_states(activate_action, [(cmd_path, full_path)], options, None)
        except Exception, e:
            print >>sys.stderr, '[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[0]), e)
            return 1
    return 0

def main():
    global simulator
    usage = '''Usage: %prog [launchfile]'''
    if len(sys.argv) <= 1:
        print >>sys.stderr, usage
        return 1
    fullpathname = sys.argv[1]
    print >>sys.stderr, "[rtmlaunch] starting... ",fullpathname
    try:
        parser = parse(fullpathname)
        nodes = parser.getElementsByTagName("launch")[0].childNodes
        remove_nodes = []
        for node in nodes:
            if node.nodeName == u'group':
                val = node.getAttributeNode(u'if').value
                arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
                if not get_flag_from_argv(arg):
                    remove_nodes.append(node)
        for remove_node in remove_nodes:
            nodes.remove(remove_node)
    except Exception,e:
        print e
        return 1

    if os.getenv("RTCTREE_NAMESERVERS") == None:
        print >>sys.stderr, "[rtmlaunch] RTCTREE_NAMESERVERS is not set, use localhost:15005"
        nameserver = "localhost:15005"
        os.environ["RTCTREE_NAMESERVERS"] = nameserver
    else:
        nameserver = os.getenv("RTCTREE_NAMESERVERS")

    simulator = os.getenv("SIMULATOR_NAME","Simulator")
    print >>sys.stderr, "[rtmlaunch] RTCTREE_NAMESERVERS", nameserver,  os.getenv("RTCTREE_NAMESERVERS")
    print >>sys.stderr, "[rtmlaunch] SIMULATOR_NAME", simulator
    while 1:
        print >>sys.stderr, "[rtmlaunch] check connection/activation"
        rtconnect(nameserver, parser.getElementsByTagName("rtconnect"))
        rtactivate(nameserver, parser.getElementsByTagName("rtactivate"))
        time.sleep(10)

def get_flag_from_argv(arg):
    for a in sys.argv:
        if arg in a: # If "USE_WALKING" is in argv
            return True if 'true' in a.split("=")[1] else False

import signal
def signal_handler(signum, frame):
    sigdict = dict((k, v) for v, k in signal.__dict__.iteritems() if v.startswith('SIG'))
    print >>sys.stderr, "\033[33m[rtmlaunch] Catch signal %r, exitting...\033[0m"%(sigdict[signum])
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()




