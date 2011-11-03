#!/bin/bash

TEST_DIR=`rospack find openhrp3`/test
function check-sample-project {
    local filename=$1
    rm -f $TEST_DIR/project-`basename $filename .xml`.png
    rosrun openhrp3 grxui.sh $filename &
    cnee --replay --time 20 -f $TEST_DIR/cnee-grxui-start.xns
    sleep 5; import -window Eclipse\ SDK\  $TEST_DIR/project-`basename $filename .xml`.png
    cnee --replay --time 5 -f $TEST_DIR/cnee-grxui-quit.xns
}

SHARE_DIR=`rospack find openhrp3`/share
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/Sample.xml
exit
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/FallingBoxes.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/ODESample_Crash.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PA10Sample.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PD_HGtest.xml
#check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/PathPlanner.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/Sample.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleHG.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleLF.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SamplePD.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SamplePD_HG.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleRH2.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleRobot_inHouse.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleSV.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SampleSV_RangeSensor.xml
check-sample-project $SHARE_DIR/OpenHRP-3.1/sample/project/SpringJoint.xml


