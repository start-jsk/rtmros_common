#!/bin/bash -i

port=$1

while [ 0 -eq `rtcat ${port} | grep "+DataOutPort" | wc -l` ]
do
    echo "sleep"
    sleep 1
done
echo "start listening ${port}"
rosrun rtshell rtprint -p /tmp -m RTMROSDataBridge ${port}
