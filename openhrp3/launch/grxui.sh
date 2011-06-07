#!/bin/sh
if [ "$1" != "" ]; then
    export INITIAL_PROJECT=$1
fi

rosrun openhrp3 eclipse.sh \
	-application com.generalrobotix.ui.grxui.applications \
        -perspective com.generalrobotix.ui.grxui.GrxUIPerspectiveFactory
