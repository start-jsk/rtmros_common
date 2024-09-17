#!/usr/bin/env python

from __future__ import print_function
import sys, socket, re, os

# run input/raw_input both python2 and python3
try:
   input = raw_input
except NameError:
    pass

from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

rootnc = None
nshost = None
nsport = None
# https://github.com/fkanehiro/hrpsys-base/blob/4f467e98671fb5ba67ce4350715ff9ab5cfe9fa0/python/rtm.py#L279
def initCORBA():
    global rootnc, orb, nshost, nsport

    # from omniorb's document
    # When CORBA::ORB_init() is called, the value for each configuration
    # parameter is searched for in the following order:
    #  Command line arguments
    #  Environment variables
    # so init_CORBA will follow this order
    # first check command line argument
    try:
        n = sys.argv.index('-ORBInitRef')
        (nshost, nsport) = re.match(
            'NameService=corbaloc:iiop:(\w+):(\d+)/NameService', sys.argv[n + 1]).group(1, 2)
    except:
        if not nshost:
            nshost = socket.gethostname()
        if not nsport:
            nsport = 15005

    print("\033[34m[check_name_server] Connect Nameserver with %s:%s\033[0m"%(nshost, nsport), file=sys.stderr)
    os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:%s:%s/NameService' % \
                               (nshost, nsport)

    try:
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
        nameserver = orb.resolve_initial_references("NameService")
        rootnc = nameserver._narrow(CosNaming.NamingContext)
        return None
    except CORBA.ORB.InvalidName:
        _, e, _ = sys.exc_info()
        print('\033[31m[check_name_server] [ERROR] Invalide Name (hostname=%s).\n\033[0m' % (nshost) +
                 'Make sure the hostname is correct.\n' + str(e), file=sys.stderr)
    except CORBA.TRANSIENT:
        _, e, _ = sys.exc_info()
        print('\033[31m[check_name_server] [ERROR] Connection Failed with the Nameserver (hostname=%s port=%s).\n\033[0m' % (nshost, nsport) +
                 '\033[31m[check_name_server] [ERROR] Make sure the hostname is correct and the Nameserver is running (%s).\033[0m' % (str(e)),
              file=sys.stderr)
    except Exception:
        _, e, _ = sys.exc_info()
        print('\033[31m[check_name_server] %s\033[0m' % (str(e)), file=sys.stderr)
    throw(Exception)


if __name__ == '__main__':
    try:
        initCORBA()
        input()
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception:
        _, e, _ = sys.exc_info()
        print('\033[31m================================================================================\033[0m', file=sys.stderr)
        print('\033[31m==                                                                            ==\033[0m', file=sys.stderr)
        print('\033[31m==                     Maybe forget to use `rtmlaunch` ?                      ==\033[0m', file=sys.stderr)
        print('\033[31m==                                                                            ==\033[0m', file=sys.stderr)
        print('\033[31m================================================================================\033[0m', file=sys.stderr)
        sys.exit(-1)

