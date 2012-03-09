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
    rosmake --status-rate=0 --test-only $package
    (cd `rospack find $package`; rosrun rosdoc rosdoc $package 2>&1 | tee build/rosdoc.log; [ `grep WARNING build/rosdoc.log | wc -l` -gt 1 ] && exit 1) # we assume .static does not exist... warning

    if [ -n "$WORKSPACE" ]; then # only for jenkins to copy to results
	rm -fr $WORKSPACE/$package-example
	cp -r `rospack find $package`/doc/$package/html $WORKSPACE/$package-example
    fi
}

# do test
rm -fr `rospack find openhrp3`/workspace/
test-grxui openhrp3
test-grxui hrpsys

# resize eclipse window size
#sed -i 's/height="[0-9]*" width="[0-9]*"/height="723" width="506"/' `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
#head `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
# do test
test-grxui hrpsys_ros_bridge

# done
rosrun rosunit clean_junit_xml.py


