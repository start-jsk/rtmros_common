#!/bin/bash

openhrp-controller-bridge --server-name SampleRobot \
    --periodic-rate SampleRobot\(Robot\)0:1.0 \
    --periodic-rate HGcontroller0:1.0 \
    --in-port qRef:JOINT_VALUE \
    --in-port dqRef:JOINT_VELOCITY \
    --in-port ddqRef:JOINT_ACCELERATION \
    --out-port q:JOINT_VALUE \
    --out-port pr:WAIST:ABS_TRANSFORM \
    --out-port left-eye:0:COLOR_IMAGE:0.1 \
    --out-port right-eye:1:COLOR_IMAGE:0.1 \
    --connection qRef:HGcontroller0:qOut \
    --connection dqRef:HGcontroller0:dqOut \
    --connection ddqRef:HGcontroller0:ddqOut







