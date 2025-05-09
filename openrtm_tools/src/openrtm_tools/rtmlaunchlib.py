#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('openrtm_tools')

import sys
import os
import time
import optparse
import socket

from xml.dom.minidom import parse

import rtctree
from rtshell import rtcon
from rtshell import path
from rtshell import state_control_base
from rtshell import rts_exceptions

def alive_component(path, tree):
    if tree.has_path(path) and tree.is_component(path):
        node = tree.get_node(path)
        return node.plain_state_string
    else:
        tree._root._remove_all_children() # reflesh
        tree.load_servers_from_env()
        return False

def wait_component(cmd_path, tree):
    count=0
    path = rtctree.path.parse_path(cmd_path)[0]
    node = alive_component(path, tree)
    if not node:
        while count < 30:
            print("\033[33m[rtmlaunch] Wait for ",cmd_path," ",count,"/30\033[0m", file=sys.stderr)
            node = alive_component(path, tree)
            if node:
                return node
            count += 1
            time.sleep(1)
        raise rts_exceptions.NoSuchObjectError(cmd_path)
    return node

def check_connect(src_path, dest_path, tree):
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

def replace_arg_tag_by_env (input_path):
    import re
    ret_str = input_path
    if len(input_path.split("$(arg")) > 1:
        arg_str = input_path.split("$(arg")[1].split(")")[0]
        env_str = os.getenv(arg_str.replace(" ", ""))
        if env_str != None:
            # print >>sys.stderr, "[rtmlaunch] Replace arg tag in %s from [$(arg%s)] to [%s]"%(input_path, arg_str, env_str)
            ret_str = re.sub("\$\(arg"+arg_str+"\)",env_str,input_path)
    return ret_str

def rtconnect(nameserver, tags, tree):
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
        source_path = replace_arg_tag_by_env(source_path)
        dest_path = replace_arg_tag_by_env(dest_path)
        # print >>sys.stderr, "[rtmlaunch] Connecting from %s to %s"%(source_path,dest_path)
        source_full_path = path.cmd_path_to_full_path(source_path)
        dest_full_path = path.cmd_path_to_full_path(dest_path)
        if tag.attributes.get("subscription_type") != None:
            sub_type = tag.attributes.get("subscription_type").value
            sub_type = replace_arg_tag_by_env(sub_type);
            if not sub_type in ['flush','new','periodic']:
                print(sub_type+' is not a subscription type', file=sys.stderr)
                continue
        else:
            sub_type = 'flush' # this is default value
        if sub_type == 'new':
            push_policy = 'all'
        # wait for proess
        try:
            wait_component(source_full_path, tree)
            wait_component(dest_full_path, tree)
            if check_connect(source_full_path,dest_full_path, tree):
                continue
        except Exception as e:
            print('\033[31m[rtmlaunch] [ERROR] Could not Connect (', source_full_path, ',', dest_full_path, '): ', e,'\033[0m', file=sys.stderr)
            return 1
        #print source_path, source_full_path, dest_path, dest_full_path;
        try:
            sub_type = str(sub_type)
            props = {'dataport.subscription_type': sub_type}
            if tag.attributes.get("push_policy") != None:
                push_policy = replace_arg_tag_by_env(str(tag.attributes.get("push_policy").value));
            else:
                push_policy = 'all'
            if sub_type == 'new':
                props['dataport.publisher.push_policy'] = push_policy
            elif sub_type == 'periodic':
                props['dataport.publisher.push_policy'] = push_policy
                if tag.attributes.get("push_rate") != None:
                    props['dataport.push_rate'] = replace_arg_tag_by_env(str(tag.attributes.get("push_rate").value))
                else:
                    props['dataport.push_rate'] = str('50.0')
            if tag.attributes.get("buffer_length") != None:
                props['dataport.buffer.length'] = replace_arg_tag_by_env(str(tag.attributes.get("buffer_length").value))
            options = optparse.Values({'verbose': False, 'id': '', 'name': None, 'properties': props})
            print("[rtmlaunch] Connected from",source_path, file=sys.stderr)
            print("[rtmlaunch]             to",dest_path, file=sys.stderr)
            print("[rtmlaunch]           with",options, file=sys.stderr)
            try :
                rtcon.connect_ports(source_path, source_full_path, dest_path, dest_full_path, options, tree=tree)
            except Exception as e_1_1_0: # openrtm 1.1.0
                try:
                    rtcon.connect_ports([(source_path,source_full_path), (dest_path, dest_full_path)], options, tree=tree)
                except Exception as e_1_0_0: # openrtm 1.0.0
                    print('\033[31m[rtmlaunch] {0} did not work on both OpenRTM 1.0.0 and 1.1.0'.format(os.path.basename(sys.argv[1])),'\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]    OpenRTM 1.0.0 {0}'.format(e_1_0_0),'\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]    OpenRTM 1.1.0 {0}'.format(e_1_1_0),'\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]  This is very weird situation, Please check your network\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch] configuration with `ifconfig` on both robot and client side. \033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]  Having multiple network interface sometimes causes problem, \033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch] please see FAQ site http://www.openrtm.org/OpenRTM-aist/html/FAQ2FE38388E383A9E38396E383ABE382B7E383A5E383BCE38386E382A3E383B3E382B0.html#f2bc375d\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]            Issue related to this https://github.com/start-jsk/rtmros_hironx/issues/33\033[0m', file=sys.stderr)
                    print('\033[31m[rtmlaunch]            ~/.ros/log may contains usefully informations\033[0m', file=sys.stderr)
        except Exception as e:
            print('\033[31m[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[1]), e),'\033[0m', file=sys.stderr)
    return 0

def rtactivate(nameserver, tags, tree):
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
        full_path = replace_arg_tag_by_env(full_path)
        # print >>sys.stderr, "[rtmlaunch] activate %s"%(full_path)
        try:
            state = wait_component(full_path, tree)
            if state == 'Active':
                continue
            else:
                print("[rtmlaunch] Activate <-",state,full_path, file=sys.stderr)
        except Exception as e:
            print('\033[31m[rtmlaunch] Could not Activate (', cmd_path, ') : ', e,'\033[0m', file=sys.stderr)
            return 1
        try:
            options = optparse.Values({"ec_index": 0, 'verbose': False})
            try :
                state_control_base.alter_component_state(activate_action, cmd_path, full_path, options, tree=tree)
            except Exception as e: # openrtm 1.1.0
                state_control_base.alter_component_states(activate_action, [(cmd_path, full_path)], options, tree=tree)
        except Exception as e:
            print('[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[0]), e), file=sys.stderr)
            return 1
    return 0

def main():
    usage = '''Usage: %prog [launchfile]'''
    if len(sys.argv) <= 1:
        print(usage, file=sys.stderr)
        return 1
    fullpathname = sys.argv[1]
    print("[rtmlaunch] starting... ",fullpathname, file=sys.stderr)
    try:
        parser = parse(fullpathname)
        nodes = parser.getElementsByTagName("launch")[0].childNodes
        remove_nodes = []
        for node in nodes:
            if node.nodeName == u'group':
                if node.attributes.get(u'if'):
                    val = node.getAttributeNode(u'if').value
                    arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
                    if not get_flag_from_argv(arg):
                        remove_nodes.append(node)
                if node.attributes.get(u'unless'):
                    val = node.getAttributeNode(u'unless').value
                    arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
                    if get_flag_from_argv(arg):
                        remove_nodes.append(node)
                
        for remove_node in remove_nodes:
            nodes.remove(remove_node)
    except Exception as e:
        print(e)
        return 1

    if os.getenv("RTCTREE_NAMESERVERS") == None:
        print("[rtmlaunch] RTCTREE_NAMESERVERS is not set, use localhost:15005", file=sys.stderr)
        nameserver = "localhost:15005"
        os.environ["RTCTREE_NAMESERVERS"] = nameserver
    else:
        nameserver = os.getenv("RTCTREE_NAMESERVERS")

    print("\033[32m[rtmlaunch] RTCTREE_NAMESERVERS", nameserver,  os.getenv("RTCTREE_NAMESERVERS"), "\033[0m", file=sys.stderr)
    print("\033[32m[rtmlaunch] SIMULATOR_NAME", os.getenv("SIMULATOR_NAME","Simulator"), "\033[0m", file=sys.stderr)

    try:
        tree = rtctree.tree.RTCTree()
    except Exception as e:
        print("\033[31m[rtmlaunch] Could not start rtmlaunch.py, Caught exception (", e, ")\033[0m", file=sys.stderr)
        # check if host is connected
        try:
            hostname = nameserver.split(':')[0]
            ip_address = socket.gethostbyname(hostname)
        except Exception as e:
            print("\033[31m[rtmlaunch] .. Could not find IP address of ", hostname, ", Caught exception (", e, ")\033[0m", file=sys.stderr)
            print("\033[31m[rtmlaunch] .. Please check /etc/hosts or DNS setup\033[0m", file=sys.stderr)
            return 1
        # in this case, it is likely you forget to run name serveer
        print("\033[31m[rtmlaunch] .. Could not connect to NameServer at ", nameserver, ", Caught exception (", e, ")\033[0m", file=sys.stderr)
        print("\033[31m[rtmlaunch] .. Please make sure that you have NameServer running at %s/`\033[0m"%(nameserver), file=sys.stderr)
        print("\033[31m[rtmlaunch] .. You can check with `rtls %s/`\033[0m"%(nameserver), file=sys.stderr)
        return 1
    while 1:
        print("[rtmlaunch] check connection/activation", file=sys.stderr)
        rtconnect(nameserver, parser.getElementsByTagName("rtconnect"), tree)
        rtactivate(nameserver, parser.getElementsByTagName("rtactivate"), tree)
        time.sleep(10)
        tree.add_name_server(nameserver, [])
        if os.getenv("RTC_CONNECTION_CHECK_ONCE") and os.getenv("RTC_CONNECTION_CHECK_ONCE").lower() == "true":
            print("[rtmlaunch] break from rtmlaunch main loop.", file=sys.stderr)
            print("[rtmlaunch] If you check the rtc connection in the while loop, real-time loop becomes slow.", file=sys.stderr)
            break
def get_flag_from_argv(arg):
    for a in sys.argv:
        if arg in a: # If "USE_WALKING" is in argv
            return True if 'true' in a.split("=")[1] else False




