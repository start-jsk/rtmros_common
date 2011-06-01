#!/usr/bin/env python

import sys
import os
import optparse

from xml.dom.minidom import parse
from rtshell import rtcon
from rtshell import path
from rtshell import state_control_base

def rtconnect(nameserver, tags):
    for tag in tags:
        source_path = nameserver+"/"+tag.attributes.get("from").value
        dest_path   = nameserver+"/"+tag.attributes.get("to").value
        print "Connect from ",source_path,"to",dest_path
        source_full_path = path.cmd_path_to_full_path(source_path)
        dest_full_path = path.cmd_path_to_full_path(dest_path)
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
            options = optparse.Values({"ec_index": 0, 'verbose': False})
            state_control_base.alter_component_state(activate_action, cmd_path, full_path, options, None)
        except Exception, e:
            print >>sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
            return 1
        return 0

def main():
    usage = '''Usage: %prog [launchfile]'''
    print sys.argv
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

    nameserver = os.environ.get("RTCTREE_NAMESERVERS")
    rtconnect(nameserver, parser.getElementsByTagName("rtconnect"))
    rtactivate(nameserver, parser.getElementsByTagName("rtactivate"))


if __name__ == '__main__':
    main()




