#!/bin/bash

function usage {
    echo >&2 "usage: $0"
    echo >&2 "          [-w|--workspace] workspace to set openrtm/openhrp/hrpsys repository,"
    echo >&2 "          [-p|--package] download only selected package,"
    exit 0
}

function error {
    usage
    exit 1
}
trap error ERR

# command line parse
OPT=`getopt -o hw:p: -l help,workspace:,package: -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
	-h|--help) usage ;;
	-w|--workspace) WORKSPACE=$2; shift 2;;
	-p|--package) PACKAGE=$2; shift 2;;
	--) shift; break;;
	*) echo "Unknown option($1)"; usage;;
    esac
done

if [ ! -e $WORKSPACE/src/.rosinstall ]; then
    echo "Your workspace is not initialized yet, please run
mkdir -p $WORKSPACE/src; wstool init $WORKSPACE/src
"
    exit 1
fi

if [ "$CI" != "true" ] ; then
    for pkg in rtshell rtctree rtsprofile openrtm_aist openrtm_aist_python; do
        cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/tork-a/$pkg-release
    local-name: $pkg
EOF
    done
fi

if [ "$IS_EUSLISP_TRAVIS_TEST" != "true" ] ; then
#    wstool remove openhrp3
    # update HISTORY-{en,ja}.txt, CMakeLists.txt(OPENHRP_VERSION) and ask kanehiro-san for set tag
    cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/fkanehiro/openhrp3
    local-name: openhrp3
EOF
fi

# update package.xml, CMakeLists.txt(CMAKE_PACKAGE_VERSION) and ask kanehiro-san for set tag
cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/fkanehiro/hrpsys-base
    local-name: hrpsys
EOF

if [ "${ROS_DISTRO}" == "hydro" ] ; then
    # As .travis.yml forcely upgrade PCRE to avoid failure in building hrpsys with hydro, hrpsys_state_publisher dies:
    # https://github.com/start-jsk/rtmros_common/pull/1077#issuecomment-552102475
    # To avoid this, the following PRs are required:
    # https://github.com/ros/robot_model/pull/105, https://github.com/ros/robot_model/pull/106, https://github.com/ros/robot_model/pull/108
    cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: https://github.com/pazeshun/robot_model.git
    local-name: robot_model
    version: for-hydro-with-new-pcre
EOF
fi

wstool merge /tmp/rosinstall.$$ -t $WORKSPACE/src
wstool info -t $WORKSPACE/src
wstool update --abort-changed-uris -t $WORKSPACE/src $PACKAGE

if [ -e $WORKSPACE/src/openhrp3/CMakeLists.txt ]; then
    echo "Updaging openhrp3 CMakeLists.txt"
    sed -i 's@option(ENABLE_DOXYGEN "Use Doxygen" ON)@option(ENABLE_DOXYGEN "Use Doxygen" OFF)@' $WORKSPACE/src/openhrp3/CMakeLists.txt
fi

if [ -e $WORKSPACE/src/hrpsys/CMakeLists.txt ]; then
    echo "Updaging hrpsys CMakeLists.txt"
    sed -i 's@option(ENABLE_DOXYGEN "Use Doxygen" ON)@option(ENABLE_DOXYGEN "Use Doxygen" OFF)@' $WORKSPACE/src/hrpsys/CMakeLists.txt
fi

echo "*** Done"
