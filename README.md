rtmros_common  [![Build Status](https://travis-ci.org/start-jsk/rtmros_common.png)](https://travis-ci.org/start-jsk/rtmros_common)
-------------

A package suite that provides all the capabilities for the ROS users to connect to the robots that run on RT Middleware or RTM-based controllers.


--------------

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


