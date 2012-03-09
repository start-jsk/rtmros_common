#!/bin/bash -i

echo "rosservice call /bridge/get_value"
rosservice call /bridge/get_value
sleep 10
