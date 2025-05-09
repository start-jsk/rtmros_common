#!/usr/bin/env python
from __future__ import print_function

from openrtm_tools import rtmlaunchlib

import signal, sys
def signal_handler(signum, frame):
    sigdict = dict((k, v) for v, k in signal.__dict__.items() if v.startswith('SIG'))
    print("\033[33m[rtmlaunch] Catch signal %r, exitting...\033[0m"%(sigdict[signum]), file=sys.stderr)
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rtmlaunchlib.main()




