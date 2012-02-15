#!/bin/bash

port=$1
data=$2

sleep 5
xterm -e rtinject -p /tmp -m RTMROSDataBridge -n 10 -c "${data}" /localhost/RTMROSDataBridge0.rtc:${port}