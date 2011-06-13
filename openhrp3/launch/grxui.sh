#!/bin/sh
if [ "$1" != "" ]; then
    export INITIAL_PROJECT=$1
fi
export JYTHON_LIB=/usr/share/jython/Lib/:`rospack find hrpsys`/share/hrpsys/jython:`rospack find hrpsys`/share/hrpsys/jar

sudo pkill -KILL omniNames
rtm-naming
rosrun openhrp3 eclipse.sh \
	-application com.generalrobotix.ui.grxui.applications \
        -perspective com.generalrobotix.ui.grxui.GrxUIPerspectiveFactory
