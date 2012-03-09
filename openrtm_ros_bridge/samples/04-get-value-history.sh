#!/bin/bash -i

echo "rosservice call /bridge/get_value_history"
rosservice call /bridge/get_value_history
sleep 10
