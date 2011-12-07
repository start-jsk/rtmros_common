#!/bin/bash

trap 'rosrun rosunit clean_junit_xml.py; exit 1' ERR

function test-grxui {
    local package=$1
    rosmake --status-rate=0 --test-only $package
    (cd `rospack find $package`; rosrun rosdoc rosdoc $package)

    if [ -n "$WORKSPACE" ]; then # only for jenkins to copy to results
	rm -fr $WORKSPACE/$package-example
	cp -r `rospack find $package`/doc/$package/html $WORKSPACE/$package-example
    fi
}

# do test
test-grxui openhrp3
test-grxui hrpsys

# resize eclipse window size
sed -i 's/height="[0-9]*" width="[0-9]*"/height="723" width="506"/' `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
head `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
# do test
test-grxui hrpsys_ros_bridge

# done
rosrun rosunit clean_junit_xml.py


