#!/bin/bash

# install cnee http://blog.livedoor.jp/vine_user/archives/51738792.html, use xnee-3.10.tar.gz
TEST_DIR=`rospack find openhrp3`/test
function check-sample-project {
    local filename=$1
    local robotname=$2
    if [ "$robotname" == "" ] ; then robotname="floor"; fi
    echo $robotname
    rm -f $TEST_DIR/project-`basename $filename .xml`.png
    # start openhrp3
    rosrun openhrp3 grxui.sh $filename > /dev/null &
    while :; do
	./check-online-viewer.py $robotname
	if [ $? == 0 ] ; then
	    break;
	fi;
	echo ";; Wait for GRXUIof $filename";
	sleep 1;
    done
    # start simulator
    cnee --replay --time 5 -f $TEST_DIR/cnee-grxui-start.xns
    # capture image
    sleep 5; import -window Eclipse\ SDK\  $TEST_DIR/project-`basename $filename .xml`.png
    # done
    cnee --replay --time 1 -f $TEST_DIR/cnee-grxui-return.xns
    cnee --replay --time 1 -f $TEST_DIR/cnee-grxui-return.xns
    cnee --replay --time 1 -f $TEST_DIR/cnee-grxui-quit.xns
    cnee --replay --time 1 -f $TEST_DIR/cnee-grxui-return.xns
}

SHARE_DIR=`rospack find openhrp3`/share
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/FallingBoxes.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/ODESample_Crash.xml "longfloor"
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PA10Sample.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PD_HGtest.xml "box"
#check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PathPlanner.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/Sample.xml "box"
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleHG.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleLF.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SamplePD.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SamplePD_HG.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleRH2.xml "longfloor"
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleSV.xml "longfloor"
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleSV_RangeSensor.xml "longfloor"
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SpringJoint.xml


