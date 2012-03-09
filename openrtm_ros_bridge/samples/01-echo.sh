#!/bin/bash -i

echo "rosservice call /bridge/echo 'hello, this is echo sample'"
rosservice call /bridge/echo 'hello, this is echo sample'
sleep 10
