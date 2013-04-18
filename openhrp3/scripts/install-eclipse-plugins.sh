#!/bin/sh

export PROJECT_DIR=`rospack find openhrp3`/share/OpenHRP-3.1/sample/project
export PATH=`rospack find openhrp3`/bin:$PATH
export LD_LIBRARY_PATH=`rospack find openhrp3`/lib:$LD_LIBRARY_PATH

# java3d
export CLASSPATH=.:$CLASSPATH:`rospack find openhrp3`/lib/ext/j3dcore.jar:`rospack find openhrp3`/lib/ext/j3dutils.jar:`rospack find openhrp3`/lib/ext/vecmath.jar
export LD_LIBRARY_PATH=`rospack find openhrp3`/lib/amd64:$LD_LIBRARY_PATH

#
export LANG=C
export LC_CTYPE="en_US.UTF-8"
export GDK_NATIVE_WINDOWS=1
export ORBgiopMaxMsgSize=2147483648

export eclipse_root=/usr/share/eclipse/

#
mkdir -p `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.core.runtime/.settings/
{
cat <<EOF
eclipse.preferences.version=1
GrxUI.version=1.0
initialProjectFile=$INITIAL_PROJECT
JYTHON_LIB=$JYTHON_LIB
EOF
} > `rospack find openhrp3`/workspace/.metadata/.plugins/org.eclipse.core.runtime/.settings/com.generalrobotix.ui.grxui.prefs

install_eclipse_plugin () {
	/usr/bin/eclipse -data `rospack find openhrp3`/workspace \
	-consolelog \
	-clean -nosplash -application org.eclipse.equinox.p2.director \
	-repository http://download.eclipse.org/releases/indigo/,$1 \
	-installIU ${2}
}

install_eclipse_plugin_from_url () {
	echo Downloading ${2} ...
	install_eclipse_plugin ${1} ${2}
}

install_eclipse_plugin_from_archive () {
	echo Installing ${2} from archive ...
	if [ ! -e ${1} ]; then
		echo "----------"
		echo "ERROR: ${1} does not exist!"
		echo "run 'rosmake hrpsys' and try again."
		exit 1
	fi
	install_eclipse_plugin jar:file:${1}!/ ${2}
}

## depreciated
#ECLIPSE="java -jar /usr/lib/eclipse/plugins/org.eclipse.equinox.launcher_1.2.0.dist.jar -application org.eclipse.update.core.standaloneUpdate -command install"
#$ECLIPSE -featureId org.eclipse.gef -version 3.6.2.v20110128-0100-777B381A4Bz06565376E32322 -from http://download.eclipse.org/tools/gef/updates/releases/


install_eclipse_plugin_from_url http://download.eclipse.org/tools/gef/updates/releases/ org.eclipse.gef.sdk.feature.group/3.6.2.v20110128-0100-7G7R77A5WNcHQDbhX8JWOYLOSeRJ,org.eclipse.xsd.doc.feature.group/2.5.0.v200906151043,org.eclipse.xsd.sdk.feature.group/2.5.0.v200906151043,org.eclipse.xsd.source.feature.group/2.5.0.v200906151043
install_eclipse_plugin_from_url http://download.eclipse.org/modeling/emf/updates/releases/ org.eclipse.emf.all.feature.group/2.5.0.v200906151043,org.eclipse.emf.doc.feature.group/2.5.0.v200906151043,org.eclipse.emf.examples.feature.group/2.5.0.v200906151043,org.eclipse.emf.examples.source.feature.group/2.5.0.v200906151043,org.eclipse.emf.sdk.feature.group/2.5.0.v200906151043,org.eclipse.emf.source.feature.group/2.5.0.v200906151043

install_eclipse_plugin_from_archive `rospack find openhrp3`/build/rtmtools_eclipse.zip jp.go.asit.rtm.rtmtools.feature.group/1.0.0.201106111836
install_eclipse_plugin_from_archive `rospack find openhrp3`/build/java3declipse-20090302.zip java3d.feature.group/1.5.1,jogl.feature.group/1.1.2,org.java3declipse.core.feature.group/1.1.0
install_eclipse_plugin_from_archive `rospack find openhrp3`/build/grxui_eclipse.zip grxui.feature.group/1.0.0.201106111845
install_eclipse_plugin_from_archive `rospack find hrpsys`/build/robothardware_eclipse.zip RobotHardwareRTC.feature.group/1.0.0.201106130144

echo "Installation Completed."

exit

インストールされているものを見るには下のコマンドを実行する
$ /usr/lib/eclipse/eclipse -nosplash -application org.eclipse.equinox.p2.director -lir
RobotHardwareRTC.feature.group/1.0.0.201106130144
grxui.feature.group/1.0.0.201106111845
java3d.feature.group/1.5.1
jogl.feature.group/1.1.2
jp.go.asit.rtm.rtmtools.feature.group/1.0.0.201106111836
org.eclipse.cvs.feature.group/1.3.100.dist-7B78FHl9exZ_
org.eclipse.cvs.source.feature.group/1.3.100.dist-7B78FHl9exZ_
org.eclipse.emf.all.feature.group/2.5.0.v200906151043
org.eclipse.emf.codegen.ecore.feature.group/2.5.0.v200906151043
org.eclipse.emf.codegen.ecore.ui.feature.group/2.5.0.v200906151043
org.eclipse.emf.codegen.feature.group/2.5.0.v200906151043
org.eclipse.emf.codegen.ui.feature.group/2.4.0.v200906151043
org.eclipse.emf.common.feature.group/2.5.0.v200906151043
org.eclipse.emf.common.ui.feature.group/2.5.0.v200906151043
org.eclipse.emf.converter.feature.group/2.5.0.v200906151043
org.eclipse.emf.databinding.edit.feature.group/1.1.0.v200906151043
org.eclipse.emf.databinding.feature.group/1.1.0.v200906151043
org.eclipse.emf.doc.feature.group/2.5.0.v200906151043
org.eclipse.emf.ecore.edit.feature.group/2.5.0.v200906151043
org.eclipse.emf.ecore.editor.feature.group/2.5.0.v200906151043
org.eclipse.emf.ecore.feature.group/2.5.0.v200906151043
org.eclipse.emf.edit.feature.group/2.5.0.v200906151043
org.eclipse.emf.edit.ui.feature.group/2.5.0.v200906151043
org.eclipse.emf.examples.feature.group/2.5.0.v200906151043
org.eclipse.emf.examples.source.feature.group/2.5.0.v200906151043
org.eclipse.emf.feature.group/2.5.0.v200906151043
org.eclipse.emf.mapping.ecore.editor.feature.group/2.5.0.v200906151043
org.eclipse.emf.mapping.ecore.feature.group/2.5.0.v200906151043
org.eclipse.emf.mapping.feature.group/2.5.0.v200906151043
org.eclipse.emf.mapping.ui.feature.group/2.5.0.v200906151043
org.eclipse.emf.sdk.feature.group/2.5.0.v200906151043
org.eclipse.emf.source.feature.group/2.5.0.v200906151043
org.eclipse.equinox.p2.rcp.feature.source.feature.group/1.0.1.dist-782EqAUkcWGYdGMiW9DK982713B5
org.eclipse.gef.sdk.feature.group/3.6.2.v20110128-0100-7G7R77A5WNcHQDbhX8JWOYLOSeRJ
org.eclipse.help.source.feature.group/1.3.0.dist-7g7lFG9F858FGc7V7L9QGS1
org.eclipse.jdt.feature.group/3.7.2.dist-7z8gFcuFFZOlyA8Br73543A7B15B
org.eclipse.pde.feature.group/3.7.2.dist-7b7rFUPF9fnFH
org.eclipse.pde.source.feature.group/3.7.2.dist-7b7rFUPF9fnFH
org.eclipse.platform.ide/3.7.0.I20110613-1736
org.eclipse.xsd.doc.feature.group/2.5.0.v200906151043
org.eclipse.xsd.ecore.converter.feature.group/2.5.0.v200906151043
org.eclipse.xsd.edit.feature.group/2.5.0.v200906151043
org.eclipse.xsd.editor.feature.group/2.5.0.v200906151043
org.eclipse.xsd.feature.group/2.5.0.v200906151043
org.eclipse.xsd.mapping.editor.feature.group/2.5.0.v200906151043
org.eclipse.xsd.mapping.feature.group/2.5.0.v200906151043
org.eclipse.xsd.sdk.feature.group/2.5.0.v200906151043
org.eclipse.xsd.source.feature.group/2.5.0.v200906151043
org.java3declipse.core.feature.group/1.1.0
Operation completed in 815 ms.

初期状態から違うプラグインは以下。
上に書いてあるのはインストール元。

http://download.eclipse.org/modeling/emf/updates/releases/
org.eclipse.emf.all.feature.group/2.5.0.v200906151043
org.eclipse.emf.doc.feature.group/2.5.0.v200906151043
org.eclipse.emf.examples.feature.group/2.5.0.v200906151043
org.eclipse.emf.examples.source.feature.group/2.5.0.v200906151043
org.eclipse.emf.sdk.feature.group/2.5.0.v200906151043
org.eclipse.emf.source.feature.group/2.5.0.v200906151043

http://download.eclipse.org/tools/gef/updates/releases/
org.eclipse.gef.sdk.feature.group/3.6.2.v20110128-0100-7G7R77A5WNcHQDbhX8JWOYLOSeRJ
org.eclipse.xsd.doc.feature.group/2.5.0.v200906151043
org.eclipse.xsd.sdk.feature.group/2.5.0.v200906151043
org.eclipse.xsd.source.feature.group/2.5.0.v200906151043

jar:file:`rospack find hrpsys`/build/robothardware_eclipse.zip!/
RobotHardwareRTC.feature.group/1.0.0.201106130144

jar:file:`rospack find openhrp3`/build/grxui_eclipse.zip!/
grxui.feature.group/1.0.0.201106111845

jar:file:`rospack find openhrp3`/build/rtmtools_eclipse.zip!/
jp.go.asit.rtm.rtmtools.feature.group/1.0.0.201106111836

jar:file:`rospack find openhrp3`/build/java3declipse-20090302.zip!/
java3d.feature.group/1.5.1
jogl.feature.group/1.1.2
org.java3declipse.core.feature.group/1.1.0

