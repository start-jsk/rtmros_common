#!/bin/bash

trap 'rosrun rosunit clean_junit_xml.py; exit 1' ERR

function test-grxui {
    local package=$1
    rosmake --status-rate=0 --test-only $package
    (cd `rospack find $dir`; rosrun rosdoc rosdoc $package)

#    rm -fr $WORKSPACE/$package-example
#    cp -r `rospack find $dir`/doc/$package/html $WORKSPACE/$dir-example
}

# do test
test-grxui openhrp3
test-grxui hrpsys

# resize eclipse window size
sed -i 's/height="768" width="1024"/height="723" width="506"/' `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workbench.xml
# do test
test-grxui hrpsys hrpsys_ros_bridge

# done
rosrun rosunit clean_junit_xml.py


