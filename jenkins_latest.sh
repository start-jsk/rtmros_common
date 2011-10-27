BUILD_NUMBER=`grep number $HOME/jobs/agentsystem/lastSuccessful/build.xml | sed 's/[^0-9]//g'`
. $HOME/jobs/agentsystem/workspace/rtm-ros-robotics-$BUILD_NUMBER/setup.sh


# hrpsys
hrpsys_revision=`python -c "import pysvn; print pysvn.Client().info('\`rospack find hrpsys\`/build/hrpsys-base').revision.number"`
sed -i s/#SVN_REVISION=-r@REVISION@/SVN_REVISION=-r$hrpsys_revision/ `rospack find hrpsys`/Makefile.hrpsys-base

# openhrp3
openhrp3_revision=`LANG=C hg -R \`rospack find openhrp3\`/build/openhrp-aist-grx tip | head -1 | cut -d\: -f3`
sed -i s/#HG_REVISION=@REVISION@/HG_REVISION=$openhrp3_revision/ `rospack find openhrp3`/Makefile.openhrp-aist-grx

# add latest tag
revision=`python -c "import pysvn; print pysvn.Client().info('$target').commit_revision.number"`
latest=https://rtm-ros-robotics.googlecode.com/svn/tags/latest
target=$HOME/jobs/agentsystem/workspace/rtm-ros-robotics-$BUILD_NUMBER/rtmros_common
svn rm --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 -m "Delete Latest stable Tag (Tagged by Jenkins)" $latest/rtmros_common
svn cp --non-interactive --username rtmrosrobotics.testing@gmail.com --password XC6HC3Jy2FG3 -m　"Latest Stable Tag (Tagged by Jenkins). Revision $revision" $target $latest/rtmros_common
