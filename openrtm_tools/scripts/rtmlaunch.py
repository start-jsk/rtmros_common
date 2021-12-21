#!/usr/bin/env python

from openrtm_tools import rtmlaunchlib

import signal, sys
def signal_handler(signum, frame):
    sigdict = dict((k, v) for v, k in signal.__dict__.iteritems() if v.startswith('SIG'))
    print >>sys.stderr, "\033[33m[rtmlaunch] Catch signal %r, exitting...\033[0m"%(sigdict[signum])
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rtmlaunchlib.main()




