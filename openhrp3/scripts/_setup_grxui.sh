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
sleep 3
xdotool search --name Eclipse\ SDK key --clearmodifiers alt+f
xdotool search --name Eclipse\ SDK key --clearmodifiers x
xdotool search --name Eclipse\ SDK key --clearmodifiers Return
rosrun openhrp3 openhrp-shutdown-servers
for child in $(ps -o pid,command -axwww | awk "{if ( /eclipse.equinox.launcher/ && ! /grep/){ print \$1}}"); do

    echo "kill $child"
    kill -KILL $child
done
