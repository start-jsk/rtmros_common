#!/bin/bash -x

# install cnee http://blog.livedoor.jp/vine_user/archives/51738792.html, use xnee-3.10.tar.gz
TEST_DIR=`rospack find openhrp3`/test

function check-sample-project {
    local filename=$1
    rm -f $TEST_DIR/project-`basename $filename .xml`.png
    # start openhrp3
    rosrun openhrp3 openhrp-shutdown-servers
    rosrun openrtm rtm-naming-restart &
    # kill eclipse to avoid double start
    ps -C eclipse && pkill -KILL eclipse
    PID=`ps aux | grep /usr/lib/eclipse//plugins/org.eclipse.equinox.launcher_1.0.201.R35x_v20090715.jar | grep -v grep | awk '{print $2}'`
    if [ ! "$PID" == "" ]; then kill -KILL $PID; fi
    #
    echo "start $filename" 1>&2
    rosrun openhrp3 grxui.sh $filename > /dev/null &
    while :; do
	rosrun openhrp3 check-online-viewer.py
	if [ $? == 0 ] ; then
	    break;
	fi;
	echo ";; Wait for GRXUI of $filename" 1>&2;
	sleep 1;
    done
    # start simulator
    cnee --replay --time 5 -fcr -f $TEST_DIR/cnee-grxui-start.xns  1>&2
    # capture image
    sleep 5;
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    import -window Eclipse\ SDK\  $TEST_DIR/project-`basename $filename .xml`.png  1>&2
    # done
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-quit.xns    1>&2
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns  1>&2
    # make sure to kill eclipse
    ps $! && kill -KILL $!
}

PROJECT_FILE=$1
GETOPT=`getopt -q -l gtest_output: -- "$@"` ; [ $? != 0 ] && exit 1
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)  TEST_OUTPUT=`echo $2|sed s/^xml\://`     ; shift 2
	  ;;
  --)  shift; break
	  ;;
  esac
done

touch $TEST_OUTPUT # for dummy gtest output
check-sample-project $PROJECT_FILE





