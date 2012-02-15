#!/bin/bash -i

port=$1

waitsec=10

echo "sleep ${waitsec} seconds, then rtprint"
sleep ${waitsec}
rosrun openrtm rtprint -p /tmp -m RTMROSDataBridge /localhost/RTMROSDataBridge0.rtc:${port}
