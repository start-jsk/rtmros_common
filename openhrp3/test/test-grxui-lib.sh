#!/bin/bash -x

## parse gtest options
if [ $# -gt 0 ] ; then
    GETOPT=`getopt -l gtest_output:,start: -- dummy "$@"` ; [ $? != 0 ] && exit 1
    eval set -- "$GETOPT"
    while true
    do
	case $1 in
	    --gtest_output)  TEST_OUTPUT=`echo $2|sed s/^xml\://`     ; shift 2
		;;
	    --start)  FILENAME=$2     ; shift 2
		;;
	    --)  shift; break;
		;;
	esac
    done
    if [ "$TEST_OUTPUT" != "" ] ; then
	touch $TEST_OUTPUT # for dummy gtest output
    fi
fi

## grxui functions
TEST_DIR=`rospack find openhrp3`/test
function wait-grxui {
    while :; do
	rosrun openhrp3 check-online-viewer.py
	if [ $? == 0 ] ; then
	    break;
	fi;
	echo ";; Wait for GRXUI of $filename" 1>&2;
	sleep 1;
    done
}

# install cnee http://blog.livedoor.jp/vine_user/archives/51738792.html, use xnee-3.10.tar.gz
function start-capture-grxui {
    local filename=$1
    WINID=`xdotool search --name \Eclipse\ SDK`;
    xdotool windowraise $WINID; xdotool windowmove  $WINID 320 0; xdotool windowfocus $WINID;
    # start simulator
    xdotool key alt+g; xdotool key Down; xdotool key Down; xdotool key Down; xdotool key Down; xdotool key Down; xdotool key Return
    # capture image
    sleep 5;
    for winname in "Time is up" "Extend Time" "Simulation Finished"
    do
	winid=`xdotool search --name "$winname"`
	if [ "$winid" != "" ]; then
	    xdotool windowfocus $winid; xdotool key alt+F4
	fi
    done
    xdotool windowfocus $(xdotool search --name  Eclipse\ SDK\ )
    import -window Eclipse\ SDK\  $filename  1>&2
    # done
    xdotool windowfocus $(xdotool search --name  Eclipse\ SDK\ ) && xdotool key alt+F4
}

if [ "$FILENAME" != "" ]; then
    wait-grxui
    start-capture-grxui $FILENAME
fi
