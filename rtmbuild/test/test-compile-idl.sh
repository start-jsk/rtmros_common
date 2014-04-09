#!/bin/bash

trap 'exit 1' ERR
set -x
set -e

#CATKIN_DIR=/tmp/test_compile_idl_$$
CATKIN_DIR=/tmp/test_compile_idl
rm -fr ${CATKIN_DIR}
mkdir -p ${CATKIN_DIR}/src/rtmbuild_test/idl
cp `rospack find rtmbuild`/test/test-compile-idl.mak ${CATKIN_DIR}/src/rtmbuild_test/Makefile
cp `rospack find rtmbuild`/test/test-compile-idl.cmake ${CATKIN_DIR}/src/rtmbuild_test/CMakeLists.txt
cp `rospack find rtmbuild`/test/test-compile-idl.package.xml   ${CATKIN_DIR}/src/rtmbuild_test/package.xml
cp `rospack find rtmbuild`/test/test-compile-idl.manifest.xml   ${CATKIN_DIR}/src/rtmbuild_test/manifest.xml
cp `rospack find rtmbuild`/test/test-compile-idl.test  ${CATKIN_DIR}/src/rtmbuild_test/test-compile-idl.test

## add PKG_CONFIG_PATH for rosbuild enviornment
if [ -e `rospack find openrtm_aist`/lib/pkgconfig ]; then
    export PKG_CONFIG_PATH=`rospack find openrtm_aist`/lib/pkgconfig:$PKG_CONFIG_PATH
fi

cp `pkg-config  openrtm-aist --variable=data_prefix`/openrtm-1.1/example/src/SimpleService/MyService.idl ${CATKIN_DIR}/src/rtmbuild_test/idl/
if [ -e `rospack find rtmbuild`/build ]; then ## rosbuild
    echo "rosbuild"
    cd ${CATKIN_DIR}/src/rtmbuild_test
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    rosmake rtmbuild_test
else
    cd ${CATKIN_DIR}
    catkin_make
    source ${CATKIN_DIR}/devel/setup.bash
fi
rosrun rtmbuild_test MyServiceROSBridgeComp -o "corba.nameservers:localhost:9999" -o "naming.formats:%n.rtc" -o "logger.file_name:/tmp/rtc%p.log" _service_port:=myservice0 &
PID=$!
rosrun rtmbuild test-compile-idl.py $@
kill -HUP $PID

