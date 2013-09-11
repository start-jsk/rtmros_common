#!/bin/bash

export PATH=`rospack find openrtm_aist`/bin:$PATH
export PATH=`rospack find rtshell`/bin:$PATH
_PYTHONVER=`python -c 'import sys; print "python%s"%sys.version[:3]'`
export PYTHONPATH=`rospack find openrtm_aist_python`/lib/${_PYTHONVER}/site-packages:`rospack find rtshell`/lib/${_PYTHONVER}/site-packages:`rospack find rtctree`/lib/${_PYTHONVER}/site-packages:`rospack find rtsprofile`/lib/${_PYTHONVER}/site-packages:$PYTHONPATH
if [ -f `rospack find rtshell`/share/rtshell/shell_support ]; then
    source `rospack find rtshell`/share/rtshell/shell_support
else
    echo -e "Warning : Failed to load shell_support, try rosmake openrtm_tools"
fi

export RTCTREE_NAMESERVERS=localhost:15005

# enable set alias on non-interactive shell
shopt -s expand_aliases
alias rtmlaunch=`rospack find hrpsys_ros_bridge`/scripts/rtmlaunch
alias rtmtest=`rospack find hrpsys_ros_bridge`/scripts/rtmtest

# these completion settings are copied from rosbash
case "$CATKIN_SHELL" in
    "bash" )
        complete -F "_roscomplete_launch" -o filenames "rtmlaunch" ;;
    "zsh" )
        compctl -/g '*.(launch|test)' -x 'p[1]' -K "_roscomplete" -tx - 'p[2]' -K _roscomplete_launchfile -- + -x 'S[--]' -k "(--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params)" -- "rtmlaunch"
esac
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
