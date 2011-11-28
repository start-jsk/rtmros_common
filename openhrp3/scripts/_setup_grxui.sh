#!/bin/bash

rosrun openhrp3 grxui.sh &
RET=1
while [ $RET == 1 ]; do
  xdotool search --name Eclipse\ SDK
  RET=$?
  echo "wait for eclipse"
  sleep 1
done
sleep 10
xdotool mousemove --clearmodifiers 863 132
xdotool mousedown --clearmodifiers 1
xdotool mouseup --clearmodifiers 1
sleep 1
xdotool key --clearmodifiers alt+f key --clearmodifiers x  key --clearmodifiers Return
