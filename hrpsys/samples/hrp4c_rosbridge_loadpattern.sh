#!/bin/bash -x
rosservice call /SequencePlayerServiceROSBridgeComp/loadPattern "`rospack find hrpsys`/share/hrpsys/samples/HRP-4C/data/walk2m" 1
rosservice call /SequencePlayerServiceROSBridgeComp/waitInterpolation

