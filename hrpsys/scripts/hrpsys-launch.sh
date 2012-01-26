#!/bin/bash

ROBOT_NAME=`rosrun openhrp3 extract-robotname -ORBInitRef NameService=corbaloc:iiop:$RTCTREE_NAMESERVERS:2809/NameService $1`
ROBOT_NAME=$ROBOT_NAME RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS rosrun openrtm rtmlaunch.py $2

