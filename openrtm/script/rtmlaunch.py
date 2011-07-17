#!/usr/bin/env python

import roslib
roslib.load_manifest('openrtm')

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
    return rtctree.tree.RTCTree().has_path(path) and rtctree.tree.RTCTree().is_component(path)

def wait_component(cmd_path):
    count=0
    path = rtctree.path.parse_path(cmd_path)[0]
    while not alive_component(path) and count < 30:
        print "Wait for ",cmd_path
        count += 1
        time.sleep(1)
    if not alive_component(path):
        raise rts_exceptions.NoSuchObjectError(cmd_path)

def rtconnect(nameserver, tags):
    import re
    for tag in tags:
        source_path = nameserver+"/"+tag.attributes.get("from").value
        dest_path   = nameserver+"/"+tag.attributes.get("to").value
        source_path = re.sub("\$\(arg ROBOT_NAME\)",robotname,source_path);
        dest_path = re.sub("\$\(arg ROBOT_NAME\)",robotname,dest_path);
        print >>sys.stderr, "Connect from ",source_path,"to",dest_path
        source_full_path = path.cmd_path_to_full_path(source_path)
        dest_full_path = path.cmd_path_to_full_path(dest_path)
        # wait for proess
        try:
            wait_component(source_full_path)
            wait_component(dest_full_path)
        except Exception, e:
            print >>sys.stderr, 'Could not Connect : ', e,' '
            return 1
        #print source_path, source_full_path, dest_path, dest_full_path;
        try:
            options = optparse.Values({'verbose': False, 'id': '', 'name': None, 'properties': {}})
            rtcon.connect_ports(source_path, source_full_path, dest_path, dest_full_path, options, tree=None)
        except Exception, e:
            print >>sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
    return 0

def rtactivate(nameserver, tags):
    def activate_action(object, ec_index):
        object.activate_in_ec(ec_index)
    for tag in tags:
        cmd_path  = nameserver+"/"+tag.attributes.get("component").value
        full_path = path.cmd_path_to_full_path(cmd_path)
        print "Activate ",full_path
        try:
            wait_component(full_path)
        except Exception, e:
            print >>sys.stderr, 'Could not Activate : ', e,' '
            return 1
        try:
            options = optparse.Values({"ec_index": 0, 'verbose': False})
            state_control_base.alter_component_state(activate_action, cmd_path, full_path, options, None)
        except Exception, e:
            print >>sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
            return 1
    return 0

def main():
    global robotname;
    usage = '''Usage: %prog [launchfile]'''
    if len(sys.argv) <= 1:
        print >>sys.stderr, usage
        return 1
    fullpathname = sys.argv[1]
    print fullpathname
    try:
        parser = parse(fullpathname)
    except Exception,e:
        print e
        return 1

    nameserver = os.getenv("RTCTREE_NAMESERVERS","localhost")
    robotname = os.getenv("ROBOT_NAME","")
    print robotname
    rtconnect(nameserver, parser.getElementsByTagName("rtconnect"))
    rtactivate(nameserver, parser.getElementsByTagName("rtactivate"))


if __name__ == '__main__':
    main()




