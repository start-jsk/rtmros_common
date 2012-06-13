#!/bin/bash

export PATH=`rospack find openrtm`/bin:$PATH
export PYTHONPATH=`rospack find openrtm`/lib/`python -c 'import sys; print "python%s"%sys.version[:3]'`/site-packages:$PYTHONPATH
. `rospack find openrtm`/share/rtshell/shell_support

export RTCTREE_NAMESERVERS=localhost
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
