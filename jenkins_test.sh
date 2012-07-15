#!/bin/bash

function error {
    rosrun rosunit clean_junit_xml.py
    echo "source $ROS_WORKSPACE/setup.bash"
    exit 1
}
rm -fr $ROS_HOME/test_results/
trap error ERR

function test-grxui {
    local package=$1
    trap error ERR
    (cd `rospack find $package`; make test || (sudo /etc/init.d/omniorb4-nameserver stop; rosrun openhrp openhrp3-shutdown-servers; make test) )
    (cd `rospack find $package`; rosrun rosdoc rosdoc $package 2>&1 | tee build/rosdoc.log; [ `grep "WARNING: cannot copy" build/rosdoc.log | wc -l` -gt 0 ] && exit 1 || true) # we assume .static does not exist... warning

    (cd `rospack find $package`; copy build/index.rst .; copy build/conf.py .;svn add --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 index.rst conf.py; svn commit --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 -m "update index.rst,conf.py by Jenkins" index.rst conf.py)
    if [ -n "$WORKSPACE" ]; then # only for jenkins to copy to results
	rm -fr $WORKSPACE/$package-example
	cp -r `rospack find $package`/doc/$package/html $WORKSPACE/$package-example
    fi
}

# do test
set -x
rm -fr `rospack find openhrp3`/workspace/
sudo /etc/init.d/omniorb4-nameserver stop
(rosrun openhrp3 openhrp-shutdown-servers; exit 0)
test-grxui openhrp3
test-grxui hrpsys

# resize eclipse window size
#sed -i 's/height="[0-9]*" width="[0-9]*"/height="723" width="506"/' `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
#head `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
# do test
test-grxui hrpsys_ros_bridge

# done
rosrun rosunit clean_junit_xml.py


