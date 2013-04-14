#!/bin/bash

export PATH=`rospack find openrtm`/bin:$PATH
export PYTHONPATH=`rospack find openrtm`/lib/`python -c 'import sys; print "python%s"%sys.version[:3]'`/site-packages:$PYTHONPATH
if [ -f `rospack find openrtm`/share/rtshell/shell_support ]; then
    . `rospack find openrtm`/share/rtshell/shell_support
else
    echo -e "Warning : Failed to load shell_support, try rosmake openrtm"
fi

export RTCTREE_NAMESERVERS=localhost
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
