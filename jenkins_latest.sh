#!/bin/bash

trap 'exit 1' ERR

export LC_ALL=C
sudo apt-get install python-svn

. $WORKSPACE/ros/groovy/setup.sh

function rtmros_common_merge {
    # hrpsys
    hrpsys_revision=`python -c "import pysvn; print pysvn.Client().info('\`rospack find hrpsys\`/build/hrpsys-base-source').revision.number"`

    # openhrp3
    openhrp3_revision=`python -c "import pysvn; print pysvn.Client().info('\`rospack find openhrp3\`/build/OpenHRP-3').revision.number"`

    #
    echo ";;   hrpsys revision : $hrpsys_revision"
    echo ";; openrhp3 revision : $openhrp3_revision"


    # download and merge
    latest_uri=https://rtm-ros-robotics.googlecode.com/svn/tags/latest/rtmros_common
    if [ "$WORKSPACE" == "" ]; then # if not jenkins
        export WORKSPACE=$HOME
    fi
    latest=$WORKSPACE/rtmros_common-latest
    target=$WORKSPACE/ros/groovy/rtm-ros-robotics/rtmros_common

    ## rtmros_common
    rm -fr rtmros_common-latest ; svn co $latest_uri rtmros_common-latest;

    latest_revision=`python -c "import pysvn; print pysvn.Client().info('$latest').commit_revision.number"`
    target_revision=`python -c "import pysvn; print pysvn.Client().info('$target').commit_revision.number"`
    echo ";; latest revision : $latest_revision"
    echo ";; target revision : $target_revision"


    (cd $latest; svn merge --accept theirs-full -r 0:$target_revision $target)
    export ROS_PACKAGE_PATH=$latest:$ROS_PACKAGE_PATH
    sed -i s/^SVN_REVISION=-r.*$/SVN_REVISION=-r$hrpsys_revision/ `rospack find hrpsys`/Makefile.hrpsys-base
    sed -i s/^SVN_REVISION=-r.*$/SVN_REVISION=-r$openhrp3_revision/ `rospack find openhrp3`/Makefile.openhrp3
}

function rtmros_common_commit {
    cat <<EOF >  latest.commit.msg
Add latest tag for $target_revision by Jenkins
Update from previous latest version($latest_revision) are....

`svn log -r $latest_revision:$target_revision $target`
EOF

    /usr/bin/svn commit --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 -F latest.commit.msg $latest

#if [ `\svn diff --diff-cmd /usr/bin/diff -x "--normal " $latest_uri https://rtm-ros-robotics.googlecode.com/svn/trunk/rtmros_common | grep "^>" | grep -v "@REVISION" | wc -l` != 0 ] ; then
#    \svn diff $latest_uri https://rtm-ros-robotics.googlecode.com/svn/trunk/rtmros_common
#fi
}

function rosinstall_merge {
    ##
    ## update agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall
    ##
    latest_revision=`python -c "import pysvn; print pysvn.Client().info('$latest').commit_revision.number"`

    target_ros_install_dir=$WORKSPACE/ros/groovy/
    latest_ros_install_dir=$WORKSPACE/agentsystem_ros_tutorials
    #
    # get latest .rosinstall
    rm -fr ${latest_ros_install_dir}
    svn co -N https://rtm-ros-robotics.googlecode.com/svn/tags/latest/agentsystem_ros_tutorials/
    #
    # copy backup
    cp ${target_ros_install_dir}/.rosinstall  ${target_ros_install_dir}/.rosinstall.bak

    # generate versioned rosinstall
    rosinstall ${target_ros_install_dir} --generate-versioned-rosinstall=${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs || true

    # set rtm-ros-robotics from trunk to latest
    sed -i 's#https://rtm-ros-robotics.googlecode.com/svn/trunk/rtmros_common#https://rtm-ros-robotics.googlecode.com/svn/tags/latest/rtmros_common#g' ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs
    sed -e "/rtm-ros-robotics/{
N
s#\(https://rtm-ros-robotics.googlecode.com/svn/tags/latest/rtmros_common',\n    version: -r\)[0-9]*#\1${latest_revision}#
}
#" ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs > ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs.tmp
    mv ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs.tmp ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs

    # conmbine other and vcs .rosinstall
    cat ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs > ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall
}

function rosinstall_commit {
    # commit
    /usr/bin/svn commit --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 -m "update versioned rosinstall file for latest tag r${latest_revision}" ${latest_ros_install_dir}
}

rtmros_common_merge
rtmros_common_commit
rosinstall_merge
rosinstall_commit


