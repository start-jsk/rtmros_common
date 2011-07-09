#!/bin/bash

ROBOT_NAME=`rosrun openhrp3 extract-robotname $1`
if [ $? != 0 ]; then exit 1; fi
ROBOT_NAME=$ROBOT_NAME rosrun openrtm rtmlaunch.py $2

