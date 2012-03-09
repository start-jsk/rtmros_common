#!/bin/bash -i

rosservice call /bridge/echo 'hello, this is echo sample'
sleep 10
