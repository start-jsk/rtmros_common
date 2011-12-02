#!/bin/bash

trap 'rosrun rosunit clean_junit_xml.py; exit 1' ERR

TEST_PACKAGES='openhrp3 hrpsys hrpsys_ros_bridge'

## do test
rosclean check
for dir in $TEST_PACKAGES; do
  rosmake --status-rate=0 --test-only $dir
  (cd `rospack find $dir`; rosrun rosdoc rosdoc $dir)
done
rosrun rosunit clean_junit_xml.py

# copy results
for dir in $TEST_PACKAGES; do
  rm -fr $WORKSPACE/$dir-example
  cp -r `rospack find $dir`/doc/$dir/html $WORKSPACE/$dir-example
done
