#!/bin/bash

touch $HOME/.ros/test_results/openhrp3/TEST-sample-project.xml

# install cnee http://blog.livedoor.jp/vine_user/archives/51738792.html, use xnee-3.10.tar.gz
TEST_DIR=`rospack find openhrp3`/test

cat <<EOF > $TEST_DIR/sample-projects.rst
OpenHPR3 examples
=================
EOF

function check-sample-project {
    local filename=$1
    local robotname=$2
    cat <<EOF >> $TEST_DIR/sample-projects.rst

`basename $filename .xml`
-------------------------

.. code-block:: bash

  roscd openhrp3/OpenHRP-3.1/sample/project/
  rosrun openhrp3 grxui.sh `basename $filename`

.. image :: project-`basename $filename .xml`.png
    :width: 500pt

Download \``basename $filename`\`_ file

.. _\``basename $filename`\`: ../../share/OpenHRP-3.1/sample/project/`basename $filename`

-------------------
EOF
    if [ "$robotname" == "" ] ; then robotname="floor"; fi
    rm -f $TEST_DIR/project-`basename $filename .xml`.png
    # start openhrp3
    rosrun openhrp3 openhrp-shutdown-servers
    rosrun openrtm rtm-naming-restart &
    rosrun openhrp3 grxui.sh $filename > /dev/null &
    while :; do
	rosrun openhrp3 check-online-viewer.py $robotname
	if [ $? == 0 ] ; then
	    break;
	fi;
	echo ";; Wait for GRXUI of $filename";
	sleep 1;
    done
    # start simulator
    cnee --replay --time 5 -fcr -f $TEST_DIR/cnee-grxui-start.xns
    # capture image
    sleep 5;
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns
    cnee --replay --time 0 -fcr -f $TEST_DIR/cnee-grxui-return.xns
    import -window Eclipse\ SDK\  $TEST_DIR/project-`basename $filename .xml`.png
    # done
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-quit.xns
    cnee --replay --time 1 -fcr -f $TEST_DIR/cnee-grxui-return.xns
    # make sure to kill eclipse
    ps $! && kill -KILL $!
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


sphinx-build  -b html -d `rospack find openhrp3`/build/doctrees $TEST_DIR `rospack find openhrp3`build/html


