#!/bin/bash -x

. `rospack find openhrp3`/test/test-grxui-lib.sh

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
    wait-grxui
    start-capture-grxui $TEST_DIR/project-`basename $filename .xml`.png
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





