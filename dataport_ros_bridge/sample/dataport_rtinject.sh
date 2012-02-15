#!/bin/bash

port=$1
data=$2

echo "sleep 5 seconds, then rtinject 10 times"
sleep 5
rtinject -p /tmp -m RTMROSDataBridge -n 10 -c "${data}" /localhost/RTMROSDataBridge0.rtc:${port}