#!/bin/bash

port=$1
data=$2

waitsec=10

echo "sleep ${waitsec} seconds, then rtinject 10 times"
sleep ${waitsec}
rtinject -p /tmp -m RTMROSDataBridge -n 10 -c "${data}" /localhost/RTMROSDataBridge0.rtc:${port}