#!/bin/bash

rosrun openhrp3 eclipse.sh &
RET=1
COUNT=0
while [ $RET == 1 -a $COUNT -lt 30 ]; do
  xdotool search --name Eclipse\ 
  RET=$?
  COUNT=`expr 1 + $COUNT`
  echo "wait for eclipse ... $COUNT"
  sleep 1
  if [ "`ps -auxwww | grep eclipse.equinox.launcher | grep -v grep`" == "" ]; then
    exit 1
  fi
done

sleep 10
xdotool search --name Eclipse\  key --clearmodifiers alt+w
xdotool search --name Eclipse\  key --clearmodifiers c
xdotool search --name Eclipse\  key --clearmodifiers Return
sleep 3
xdotool search --name Eclipse\  key --clearmodifiers alt+f
xdotool search --name Eclipse\  key --clearmodifiers x
xdotool search --name Eclipse\  key --clearmodifiers Return
rosrun openhrp3 openhrp-shutdown-servers
for child in $(ps -o pid,command -axwww | awk "{if ( /eclipse.equinox.launcher/ && ! /grep/){ print \$1}}"); do

    echo "kill $child"
    kill -KILL $child
done
