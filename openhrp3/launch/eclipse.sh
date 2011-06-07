#!/bin/sh

export PROJECT_DIR=`rospack find openhrp3`/share/OpenHRP-3.1/sample/project
export PATH=`rospack find openhrp3`/bin:$PATH
export LD_LIBRARY_PATH=`rospack find openhrp3`/lib:$LD_LIBRARY_PATH

# java3d
export CLASSPATH=.:$CLASSPATH:`rospack find openhrp3`/lib/ext/j3dcore.jar:`rospack find openhrp3`/lib/ext/j3dutils.jar:`rospack find openhrp3`/lib/ext/vecmath.jar
export LD_LIBRARY_PATH=`rospack find openhrp3`/lib/amd64:$LD_LIBRARY_PATH

#
export LANG=C
export GDK_NATIVE_WINDOWS=1
echo "initialProjectFile=${INITIAL_PROJECT}" >> `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.core.runtime/.settings/com.generalrobotix.ui.grxui.prefs
eclipse -data `rospack find openhrp3`/workspace -consolelog -clean $@


