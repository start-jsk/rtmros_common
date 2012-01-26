#!/bin/bash

ROBOT_NAME=`rosrun openhrp3 extract-robotname $1` -ORBInitRef NameService=corbaloc:iiop:$RTCTREE_NAMESERVERS:2809/NameService
if [ $? != 0 ]; then exit 1; fi
ROBOT_NAME=$ROBOT_NAME RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS rosrun openrtm rtmlaunch.py $2

