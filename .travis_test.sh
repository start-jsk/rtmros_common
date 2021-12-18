#!/bin/bash

set -x

function error {
    if [ -e ${HOME}/.ros/rosmake/ ]; then find ${HOME}/.ros/rosmake/ -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
    if [ -e ${HOME}/.ros/test_results ]; then find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
    for file in ${HOME}/.ros/log/rostest-*; do echo "=== $file ==="; cat $file; done
    exit 1
}

function apt-get-install {
    sudo -E apt-get -y -qq install $@ | pv -i 60 | grep -v -e '\(Preparing to unpack\|Unpacking\|Selecting previously unselected package\)'
    [[ ${PIPESTATUS[0]} == 0 ]] || error
}
trap error ERR

# install fundamental packages
sudo -E apt-get -y -qq update
sudo -E apt-get -y -qq install pv
apt-get-install apt-utils build-essential curl git lsb-release wget

# MongoDB hack
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get remove --purge -qq -y mongodb mongodb-10gen || echo "ok"; fi
if [ $HAVE_MONGO_DB == 0 ]; then apt-get-install mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"; fi # default actions

### before_install: # Use this to prepare the system to install prerequisites or dependencies
# Define some config vars
export CI_SOURCE_PATH=$(pwd)
export REPOSITORY_NAME=${PWD##*/}
if [ ! "$ROS_PARALLEL_JOBS" ]; then export ROS_PARALLEL_JOBS="-j8 -l8";  fi
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -y --force-yes -q -qq dpkg # https://github.com/travis-ci/travis-ci/issues/9361#issuecomment-408431262 dpkg-deb: error: archive has premature member 'control.tar.xz' before 'control.tar.gz' #9361
apt-get-install python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-rosbash

if [ "$EXTRA_DEB" ]; then apt-get-install $EXTRA_DEB;  fi
###
pkg=$TEST_PACKAGE
apt-get-install ros-$ROS_DISTRO-$pkg

apt-get-install ros-$ROS_DISTRO-rqt-robot-dashboard

#
source /opt/ros/$ROS_DISTRO/setup.bash

# set hrpsys (hrpsys wrapper for ROS, just for compile and test)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/rtmros_common

if [ "$TEST_TYPE" == "work_with_downstream" ]; then
    echo "
            #
            # check newer version of hrpsys works on current rtmros_common deb package
            # [rtmros_common:new] <-> [rtmros_<robot>:old]
            "
    sudo dpkg -r --force-depends ros-$ROS_DISTRO-hrpsys-ros-bridge
    sudo dpkg -r --force-depends ros-$ROS_DISTRO-hrpsys-tools
    sudo dpkg -r --force-depends ros-$ROS_DISTRO-rtmbuild
    # https://github.com/start-jsk/rtmros_hironx/issues/287
    sudo sed -i s@test_tf_and_controller@_test_tf_and_controller@ /opt/ros/$ROS_DISTRO/share/hironx_ros_bridge/test/test_hironx_ros_bridge.py
    catkin_make $ROS_PARALLEL_JOBS | grep -v '^-- \(Up-to-date\|Installing\):' | grep -v 'Generating \(Python\|C++\) code from' | grep -v '^Compiling .*.py ...$' | uniq
    catkin_make install $ROS_PARALLEL_JOBS | grep -v '^-- \(Up-to-date\|Installing\):' | grep -v 'Generating \(Python\|C++\) code from' | grep -v '^Compiling .*.py ...$' | uniq
else
    echo "
            #
            # check rtmros_<robot> compiled on newer version of rtmros_common works with deb version of rtmros_common
            # [rtmros_common:old] <-> [rtmros_<robot>:new] + [rtmros_common:new]
            "
            # set up sorce code of downstream package
    cd src
    wstool init
    wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y
    wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y
    wstool update
    cd ..
    # do not install rtmros_common because we want to use them
    find src
    for _pkg in hrpsys_ros_bridge hrpsys_tools rtmbuild; do
        if [ -e src/rtmros_common/${_pkg}/CMakeLists.txt ] ; then
            sed -i "1imacro(dummy_install)\nmessage(\"install(\${ARGN})\")\nendmacro()" src/rtmros_common/${_pkg}/CMakeLists.txt
            sed -i "s@install(@dummy_install(@g" src/rtmros_common/${_pkg}/CMakeLists.txt
        fi
        if [ -e src/rtmros_common/${_pkg}/catkin.cmake ] ; then
            sed -i "s@install(@dummy_install(@g" src/rtmros_common/${_pkg}/catkin.cmake
        fi
    done
    catkin_make $ROS_PARALLEL_JOBS --only-pkg-with-deps `echo $pkg | sed s/-/_/g` | grep -v '^-- \(Up-to-date\|Installing\):' | grep -v 'Generating \(Python\|C++\) code from' | grep -v '^Compiling .*.py ...$' | uniq
    catkin_make install $ROS_PARALLEL_JOBS | grep -v '^-- \(Up-to-date\|Installing\):' | grep -v 'Generating \(Python\|C++\) code from' | grep -v '^Compiling .*.py ...$' | uniq
    # make sure to kill test
    for _pkg in hrpsys_ros_bridge hrpsys_tools rtmbuild; do
	rm -fr install/lib/${_pkg}
	rm -fr install/share/${_pkg}
	rm -fr install/lib/python2.7/dist-packages/${_pkg}*
    done

fi

rospack profile
source install/setup.bash

export EXIT_STATUS=0;
pkg_path=`rospack find \`echo $pkg | sed s/-/_/g\``
if [ "`find $pkg_path/test -iname '*.test'`" == "" ]; then
    echo "[$pkg] No tests ware found!!!"
else
    find $pkg_path/test -iname "*.test" -print0 | xargs -0 -n1 rostest || export EXIT_STATUS=$?;
fi

#if [ -e build ]; then find build -name LastTest.log -exec echo "==== {} ====" \; -exec cat {} \;  ; fi
if [ $EXIT_STATUS == 0 ] ; then
    return 0
else
    if [ -e ${HOME}/.ros/test_results ]; then find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
    exit 1
fi

