#!/bin/bash -x

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
    # start simulator
    cnee --replay --time 5 -fcr -f $TEST_DIR/cnee-grxui-start.xns  1>&2
    # capture image
    sleep 5;
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    import -window Eclipse\ SDK\  $filename  1>&2
    # done
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-quit.xns    1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
}

