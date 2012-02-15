#!/bin/bash

port=$1

echo "sleep 5 seconds, then rtprint"
sleep 5
rtprint -p /tmp -m RTMROSDataBridge /localhost/RTMROSDataBridge0.rtc:${port}