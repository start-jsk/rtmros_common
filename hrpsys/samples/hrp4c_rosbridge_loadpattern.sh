#!/bin/bash -x
rosservice call /loadPattern "`rospack find hrpsys`/share/hrpsys/samples/HRP-4C/data/walk2m" 1
rosservice call /waitInterpolation

