#!/bin/bash

port=$1

sleep 5
xterm -e rtprint -m RTMROSDataBridge -p /tmp /localhost/RTMROSDataBridge0.rtc:${port}