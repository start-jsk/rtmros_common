#!/bin/bash

openhrp-controller-bridge --server-name Darwin \
    --periodic-rate Darwin\(Robot\)0:1.0 \
    --periodic-rate HGcontroller0:1.0 \
    --in-port qRef:JOINT_VALUE \
    --in-port dqRef:JOINT_VELOCITY \
    --in-port ddqRef:JOINT_ACCELERATION \
    --out-port q:JOINT_VALUE \
    --out-port pr:WAIST:ABS_TRANSFORM \
    --out-port camera:0:COLOR_IMAGE:0.1 \
    --connection qRef:HGcontroller0:qOut \
    --connection dqRef:HGcontroller0:dqOut \
    --connection ddqRef:HGcontroller0:ddqOut








