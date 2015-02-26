#!/usr/bin/python

import sys; sys.path.insert(0,"/usr/lib/python{0}.{1}/dist-packages".format(sys.version_info[0],sys.version_info[1])) ## ros-groovy-rqt-top install psutil under /opt/groovy/lib/python.. which is not working...


import os,psutil,subprocess,socket,sys
from omniORB import CORBA
import CosNaming

def start_cosname(cosnames, port_number):
    p = None
    pid = None
    start_naming = True

    for p in psutil.process_iter():
        try:
            # find process using port_number
            if filter(lambda c: c.local_address[1] == port_number, p.get_connections()):
                print "\033[31m[rtmlaunch]", p.name, "is already started with port", port_number,"\033[0m"

                try:
                    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
                    nameserver = orb.resolve_initial_references("NameService")
                    rootnc = nameserver._narrow(CosNaming.NamingContext)
                    def findObject(name, kind="", rnc=None) :
                        nc = CosNaming.NameComponent(name, kind)
                        if not rnc: rnc = rootnc
                        return rnc.resolve([nc])
                    cxt = findObject(socket.gethostname(), "host_cxt")
                    obj = findObject("manager", "mgr", cxt)
                    start_naming = False
                except CosNaming.NamingContext.NotFound, ex:
                    # this is ok since host_cxt, manager is not bind
                    start_naming = False
                    pass
                except:
                    print "\033[31m[rtmlaunch] name server is unreachable so kill process\033[0m"
                    print "\033[31m[rtmlaunch] kill ", cosnames, " of pid", p.pid,"\033[0m"
                    p.terminate()
        except:
            pass

    if not start_naming :
        print "\033[31m[rtmlaunch] do not start", cosnames, ", exiting...\033[0m"
        exit(0)
    else:
        print "\033[34m[rtmlaunch] Start", cosnames, "at port", port_number, "\033[0m"
        logdir = "/tmp"
        hostname = socket.gethostname()
        try :
            os.remove(logdir+"/omninames-"+hostname+".log")
            os.remove(logdir+"/omninames-"+hostname+".bak")
        except:
            pass

        p = subprocess.Popen([cosnames,"-start", str(port_number), "-always", "-logdir", logdir])

        return p




