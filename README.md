# rtmros_common  [![Build Status](https://travis-ci.org/start-jsk/rtmros_common.png)](https://travis-ci.org/start-jsk/rtmros_common)

A package suite that provides all the capabilities for the ROS users to connect to the robots that run on RT Middleware or RTM-based controllers.

## Install

This document explains how to use and how to contribute to rtm-ros-robotics software ([openrtm_aist](https://github.com/tork-a/openrtm_aist-release), [openhrp3](https://github.com/fkanehiro/openhrp3), [hrpsys](https://github.com/fkanehiro/hrpsys-base), [rtshell](https://github.com/tork-a/rtshell-release), [rtctree](https://github.com/tork-a/rtctree-release), [rtsprofile](https://github.com/tork-a/rtsprofile-release), [rtmros_common](https://github.com/start-jsk/rtmros_common), [rtmros_hironx](https://github.com/start-jsk/rtmros_hironx), [rtmros_tutorials](https://github.com/start-jsk/rtmros_tutorials), [rtmros_gazebo](https://github.com/start-jsk/rtmros_gazebo)). The instruction uses `rtmros_common` repository as an example, but also works for other rtm-ros-robotics repositories.

1. Install software

 rtm-ros-robotics software is distributed as ros-debian packages, if you already use ROS system, install the software as follows:
 - `sudo apt-get install ros-${ROS_DISTRO}-rtmros-common`

 **NOTE** Currently, those packages are not released to `melodic` distribution. Please compile all rtm-ros-robotics source code by following the section below ("2. Compile from source code").

 If you did not install ROS system, please follow [this instruction](http://wiki.ros.org/ROS/Installation).
 - ``sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -a` main" > /etc/apt/sources.list.d/ros-latest.list'``
 - `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`
 - `sudo apt-get update`
 - `sudo apt-get update ros-${ROS_DISTRO}-base`
 - `sudo rosdep init`
 - `rosdep update`
 - `source /opt/ros/${ROS_DISTRO}/setup.bash` # it is better to source ROS environment every time terminal starts (`echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc`)

2. Compile from source code

 You may have two choices, one is to compile target repository (rtmros_common) only, the other is to compile all rtm-ros-robotics source code.
 First, create catkin workspace and add rtmros_common:
 - `mkdir -p ~/catkin_ws/src`
 - `cd ~/catkin_ws/src`
 - `wstool init .`
 - `wstool set rtmros_common https://github.com/start-jsk/rtmros_common --git -y`

 If you want to compile all source code:
 - (If the target repository is not `rtmros_common`) `wstool set rtmros_common https://github.com/start-jsk/rtmros_common --git -y`
 - (On `kinetic` distribution) `sudo apt-get install libopencv-dev`
   - Skipping this may cause [runtime error](https://github.com/start-jsk/rtmros_common/pull/1091#issuecomment-611457804)
 - `wstool merge https://raw.githubusercontent.com/start-jsk/rtmros_common/master/.travis.rosinstall -y`
 - `wstool merge https://raw.githubusercontent.com/start-jsk/rtmros_common/master/.travis.rosinstall.${ROS_DISTRO} -y`
   - Please ignore `Unable to download URL` error on `indigo` distribution

 Both methods need following procedures:
 - `wstool update`
 - `cd ..`
 - `source /opt/ros/${ROS_DISTRO}/setup.bash`
 - `rosdep install -r --from-paths src --ignore-src -y`
 - `sudo apt-get install python-catkin-tools`
 - `catkin build`

3. Contribute to rtm-ros-robotics projects.

 - First fork the target repository on GitHub
 - Move to the package directory
   - `source ~/catkin_ws/devel/setup.bash`
   - `roscd rtmros_common`
 - Create branch for your fix
   - `git checkout -b your_awesome_code_branch`
 - Write awesome code and commit to local repo
   - ...Write code....
   - `git commit -m "detailed description of what you did"`
 - Add your forked repository as upstream
   - `git remote add experimental https://github.com/<your github user name>/rtmros_common`
 - `git push experimental your_awesome_code_branch`
 - Submit a pull request on GitHub to the repository
 - Please check travis-ci status after sending your pull request.

4. Use other forked repository *before* merged into master.

You don't need to wait for the maintainers to merge some pull requests by others
before you use them.

 - Adding other's remote repository to your git remote
   - `git remote add <awesome-fork> https://github.com/<user>/rtmros_common`
 - Fetch branches from the remote.
   - `git fetch <awesome-fork>`
 - Merge those remote branch into your current branch
   - `git merge <awesome-fork>/<branch-name>`

## For maintainers

### Tweak to release into Groovy

While in current design the package depends on `pr2_controllers_msgs <http://wiki.ros.org/pr2_controllers_msgs>`_ that's catkinized from ROS hydro onward and not available in ROS groovy, some hacks allow the package not to separate `branches` (regardless it's good or not, doing so is the decision as of March 2014). This requires another hack during release process using `bloom` as follows:

 1. Run `bloom <http://wiki.ros.org/bloom>`_ as normal. E.g. `$ bloom-release --rosdistro groovy --track groovy rtmros_common`.
 2. Once `bloom` halts and its command prompt starts waiting on the same terminal (the output might look as following), edit `package.xml` to comment out lines for declaring build and run depend on `pr2-controllers`.

    ```
    $ bloom-release --rosdistro groovy --track groovy rtmros_common
    :
    >>> Resolve any conflicts and when you have resolved this problem run 'git am --resolved' and then exit the shell using 'exit 0'. <<<
        To abort use 'exit 1'
    (bloom)emacs package.xml
    ```

    Modify `package.xml` as:

    ```
    :
    <!-- <build_depend>pr2_controllers</build_depend> -->
    :
    <!-- <run_depend>pr2_controllers</run_depend> -->
    :
    ```

 3. Run `git add`, `$ git am --skip`, `git commit` like below, and `exit 0` respectively. If all succeeds then `bloom` resumes.

    ```
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
    ```
