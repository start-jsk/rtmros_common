#!/bin/bash

trap 'exit 1' ERR

export LC_ALL=C
#sudo apt-get install python-svn

export WORKSPACE=/home/k-okada/tmp/rtm-ros-robotics/trunk/rosbuild_ws
. $WORKSPACE/ros/groovy/setup.sh

function rtmros_common_merge {
    # hrpsys
    hrpsys_revision=`python -c "import pysvn; print pysvn.Client().info('\`rospack find hrpsys\`/build/hrpsys-base-source').revision.number"`

    # openhrp3
    openhrp3_revision=`python -c "import pysvn; print pysvn.Client().info('\`rospack find openhrp3\`/build/OpenHRP-3.1').revision.number"`

    #
    echo ";;   hrpsys revision : $hrpsys_revision"
    echo ";; openrhp3 revision : $openhrp3_revision"


    # download and merge
    latest_uri=https://rtm-ros-robotics.googlecode.com/svn/tags/latest
    if [ "$WORKSPACE" == "" ]; then # if not jenkins
        export WORKSPACE=$HOME
    fi
    latest=$WORKSPACE/rtm-ros-robotics-latest
    target=$WORKSPACE/ros/groovy/rtm-ros-robotics

    ## rtmros_common
    rm -fr $latest ; svn co $latest_uri $latest

    latest_revision=`python -c "import pysvn; print pysvn.Client().info('$latest').commit_revision.number"`
    target_revision=`python -c "import pysvn; print pysvn.Client().info('$target/rtmros_common').commit_revision.number"`
    echo ";; latest revision : $latest_revision"
    echo ";; target revision : $target_revision"

    for dir in openrtm_common rtmros_common rtmros_tutorials rtmros_hironx rtmros_gazebo openrtm_apps; do
        (cd $latest/$dir; svn merge --accept theirs-full -r 0:$target_revision $target/$dir)
    done
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
    latest_revision=`python -c "import pysvn; print pysvn.Client().info('$latest/rtmros_common').commit_revision.number"`

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
    sed -i 's#https://rtm-ros-robotics.googlecode.com/svn/trunk/#http://rtm-ros-robotics.googlecode.com/svn/tags/latest/#g' ${latest_ros_install_dir}/rtm-ros-robotics.rosinstall.vcs
    # remove version from rtmros_common
    sed -e "/rtm-ros-robotics/{
N
s#\(http://rtm-ros-robotics.googlecode.com/svn/tags/latest/.*'\),\n    version: -r[0-9]*#\1#
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


