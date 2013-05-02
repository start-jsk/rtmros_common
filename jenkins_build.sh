#!/bin/bash


function usage {
    echo >&2 "usage: $0"
    echo >&2 "          [-h|--help] print this message"
    echo >&2 "          [-t|--target] set compile package"
    exit 0
}


# command line parse
OPT=`getopt -o t:h -l help,target: -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
	-h|--help) usage ;;
	-t|--target) TARGET=$2; shift 2; break;;
	--) shift; break;;
	*) echo "Unknown option($1)"; usage;;
    esac
done

wget 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosbuild?format=raw' -O /tmp/jsk.rosbuild
. /tmp/jsk.rosbuild -e

function install-rtm-ros-robotics {
    install-pkg  'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosinstall?format=raw' http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall
}

function compile-rtm-ros-robotics {
    compile-pkg euscollada rtmros_common openhrp3 hrpsys_ros_bridge mrobot_ros_bridge RS003 openrtm_ros_bridge
}

set -x

setup-ros
apt-get-ros-package
install-rtm-ros-robotics
if ( [ "$TARGET" = "" ] ) then
    echo "compile-rtm-ros-robotics"
    compile-rtm-ros-robotics
else
    echo "compile-pkg $TARGET"
    compile-pkg $TARGET
fi

