#!/bin/bash

if [ -e `rospack find rtshell`/bin ] ; then # if rosbuild
    _PYTHONVER=`python -c 'import sys; print "python%s"%sys.version[:3]'`
    if [ -e /etc/debian_version ]; then
	_PYTHON_DIR=lib/${_PYTHONVER}/dist-packages
    else
	_PYTHON_DIR=lib/${_PYTHONVER}/site-packages
    fi
    # export PATH=`rospack find openrtm_aist`/bin:$PATH
    export PYTHONPATH=`rospack find openrtm_aist_python`/${_PYTHON_DIR}:$PYTHONPATH
    export PATH=`rospack find rtshell`/bin:$PATH
    export PYTHONPATH=`rospack find rtshell`/${_PYTHON_DIR}:`rospack find rtctree`/${_PYTHON_DIR}:`rospack find rtsprofile`/${_PYTHON_DIR}:$PYTHONPATH

    if [ -f `rospack find rtshell`/share/rtshell/shell_support ]; then
        source `rospack find rtshell`/share/rtshell/shell_support
    else
        echo -e "Warning : Failed to load shell_support, try rosmake openrtm_tools"
    fi

    alias rtmlaunch=`rospack find openrtm_tools`/scripts/rtmlaunch
    alias rtmtest=`rospack find openrtm_tools`/scripts/rtmtest

else
    IFS=':' read -ra PREFIX_PATH <<< "$CMAKE_PREFIX_PATH"
    loop=true
    for dir in "${PREFIX_PATH[@]}"; do
        # process "$i"
        if [ $loop = true -a -e  ${dir}/share/rtshell/shell_support ] ; then
            #source ${dir}/share/rtshell/shell_support
            source ${dir}/share/rtshell/bash_completion
            loop=false
        fi
    done
    if [ loop = true ] ; then
        echo -e "Warning : Failed to load shell_support"
    fi
fi

export RTCTREE_NAMESERVERS=localhost:15005

# enable set alias on non-interactive shell
case "$CATKIN_SHELL" in
    "bash" )
        shopt -s expand_aliases
        ;;
esac

# these completion settings are copied from rosbash
case "$CATKIN_SHELL" in
    "bash" )
        complete -F "_roscomplete_launch" -o filenames "rtmlaunch" ;;
    "zsh" )
        compctl -/g '*.(launch|test)' -x 'p[1]' -K "_roscomplete" -tx - 'p[2]' -K _roscomplete_launchfile -- + -x 'S[--]' -k "(--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params)" -- "rtmlaunch"
esac
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
