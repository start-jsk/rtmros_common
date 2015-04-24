#!/bin/bash

function usage {
    echo >&2 "usage: $0"
    echo >&2 "          [-w|--workspace] workspace to set openrtm/openhrp/hrpsys repository,"
    exit 0
}

function error {
    usage
    exit 1
}
trap error ERR

# command line parse
OPT=`getopt -o hw: -l help,workspace: -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
	-h|--help) usage ;;
	-w|--workspace) WORKSPACE=$2; shift 2;;
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

# update HISTORY-{en,ja}.txt, CMakeLists.txt(OPENHRP_VERSION) and ask kanehiro-san for set tag
cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/fkanehiro/openhrp3
    local-name: openhrp3
EOF

# update package.xml, CMakeLists.txt(CMAKE_PACKAGE_VERSION) and ask kanehiro-san for set tag
cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/fkanehiro/hrpsys-base
    local-name: hrpsys
EOF

wstool merge /tmp/rosinstall.$$ -t $WORKSPACE/src
wstool info -t $WORKSPACE/src
wstool update --abort-changed-uris -t $WORKSPACE/src

echo "Updaging openhrp3 CMakeLists.txt"
sed -i 's@option(ENABLE_DOXYGEN "Use Doxygen" ON)@option(ENABLE_DOXYGEN "Use Doxygen" OFF)@' $WORKSPACE/src/openhrp3/CMakeLists.txt

echo "Updaging hrpsys CMakeLists.txt"
sed -i 's@option(ENABLE_DOXYGEN "Use Doxygen" ON)@option(ENABLE_DOXYGEN "Use Doxygen" OFF)@' $WORKSPACE/src/hrpsys/CMakeLists.txt

echo "*** Done"
