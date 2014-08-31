================================================================================================================================================================
rtmros_common  [![Build Status](https://travis-ci.org/start-jsk/rtmros_common.png)](https://travis-ci.org/start-jsk/rtmros_common)
================================================================================================================================================================

A package suite that provides all the capabilities for the ROS users to connect to the robots that run on RT Middleware or RTM-based controllers.

.. contents::

Install
========

This document explains how to use and how to contribute to rtm-ros-robotics softwares ([openrtm_aist_core](https://github.com/start-jsk/openrtm_aist_core), [openhrp3](https://github.com/start-jsk/openhrp3), [hrpsys](https://github.com/start-jsk/hrpsys), [rtshell_core](https://github.com/start-jsk/rtshell_core), [rtmros_common](https://github.com/start-jsk/rtmros_common), [rtmros_hironx](https://github.com/start-jsk/rtmros_hironx), [rtmros_tutorial](https://github.com/start-jsk/rtmros_turorial), [rtmros_gazebo](https://github.com/start-jsk/rtmros_gazebo)). The instruction uses `rtmros_common` repository as an example, but also works for other rtm-ros-robotics repositories.

1. Install software

 rtm-ros-robotics software is distributed as ros-debian packages, if you already uses ROS system, install the software as follows:
 - `sudo apt-get install ros-$ROS_DISTRO-rtmros-common`
 
 If you did not installed ROS sysem, please follow [this instruction](http://wiki.ros.org/hydro/Installation/Ubuntu).
 - ``sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -a` main" > /etc/apt/sources.list.d/ros-latest.list'``
 - `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`
 - `sudo apt-get update`
 - `sudo apt-get update ros-hydro-base` # you may use ros-groovy-base if you want
 - `sudo rosdep init`
 - `rosdep update`
 - `source /opt/ros/hydro/setup.bash` # it is better to source ROS environment everytime terminal starts (`echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc`)

2. Compile from source code

 You may have two choice, one is to compile all rtm-ros-robotics source code, other is to compile target repository.
 First, create catkin workspace
 - `mkdir -p ~/catkin_ws/src`
 - `cd ~/catkin_ws/src`
 - `catkin_init_workspace`
 - `wstool init .`
 
 If compile all source code
 - `wstool merge https://raw.github.com/start-jsk/rtmros_common/master/.rosinstall -y`
 
 Else if compile only targe repository
 - `wstool set rtm-ros-robotics/rtmros_common https://github.com/start-jsk/rtmros_common --git -y`
 
 Both methods needs following procedures.
 - `wstool update `
 - `cd ..`
 - `source /opt/ros/hydro/setup.bash`
 - `rosdep install -v -r --from-paths src --ignore-src --rosdistro hydro -y`
 - `catkin_make`

3. Contributes to rtm-ros-robotics projects.

 - First fork the target repository on GitHub
 - Move to the package direcotry
 - `source ~/catkin_ws/devel/setup.bash`
 - `roscd rtmros_common`
 -  create branch for your fix
 - `git checkout -b your_awesome_code_branch`
 -  write awesome code and commit to local repo
 -   ... wite code....
 - `git commit -m "detailed description of what you did"`
 -  Add your forked repository as upstream
 - `git remote add experimental https://github.com/<your github user name>/rtmros_common`
 - `git push experimental your_awesome_code_branch`
 - Submit a pull request on GitHub to the repository
 - Please check travic-ci status after sending your pull request.

4. Uses other forked repository *before* merged into master.

You don't need to wait for the maintainers to merge some pull requests by others
before you use them.

 - Adding other's remote repository to your git remote
 - `git remote add <awesome-fork> https://github.com/<user>/rtmros_common`
 - Fetch branches from the remote.
 - `git fetch <awesome-fork>`
 - Merge those remote branch into your current branch
 - `git merge <awesome-fork>/<branch-name>`

For maintainers
===============

Tweak to release into Groovy
--------------------------------

While in current design the package depends on `pr2_controllers_msgs <http://wiki.ros.org/pr2_controllers_msgs>`_ that's catkinized from ROS hydro onward and not available in ROS groovy, some hacks allow the package not to separate `branches` (regardless it's good or not, doing so is the decision as of March 2014). This requires another hack during release process using `bloom` as follows:

 1. Run `bloom <http://wiki.ros.org/bloom>`_ as normal. E.g. `$ bloom-release --rosdistro groovy --track groovy rtmros_common`.
 2. Once `bloom` halts and its command prompt starts waiting on the same terminal (the output might look as following), edit `package.xml` to comment out lines for declaring build and run depend on `pr2-controllers`.

  ::

    $ bloom-release --rosdistro groovy --track groovy rtmros_common
    :
    >>> Resolve any conflicts and when you have resolved this problem run 'git am --resolved' and then exit the shell using 'exit 0'. <<<
        To abort use 'exit 1'
    (bloom)emacs package.xml 

  Modify `package.xml` as:

  ::

    :  
    <!-- <build_depend>pr2_controllers</build_depend> -->
    :
    <!-- <run_depend>pr2_controllers</run_depend> -->
    :

 3. Run `git add`, `$ git am --skip`, `git commit` like below, and `exit 0` respectively. If all succeeds then `bloom` resumes.

  ::

    (bloom)git add package.xml
    (bloom)git am --skip
    Resolve operation not in progress, we are not resuming.
    (bloom)git commit -m "3rd trial commentout pr2 pkg" -a
    [release/groovy/hrpsys_ros_bridge 1c05bbc] 3rd trial commentout pr2 pkg
     1 file changed, 1 insertion(+), 1 deletion(-)
    (bloom)git status
    # On branch release/groovy/hrpsys_ros_bridge
    # Your branch is ahead of 'origin/release/groovy/hrpsys_ros_bridge' by 8 commits.
    #
    nothing to commit (working directory clean)
    (bloom)exit 0
    exit
     [git-bloom-patch import]: User reports that conflicts have been resolved, continuing.
     [git-bloom-patch import]: Applied 2 patches
    :
    (bloom continues)
