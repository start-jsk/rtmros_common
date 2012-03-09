#!/bin/bash -i

for i in 3 2 1
do
    echo "rosservice call /bridge/set_value ${i}"
    rosservice call /bridge/set_value ${i}
done

sleep 10
