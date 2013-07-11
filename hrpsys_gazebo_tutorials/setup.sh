pkgdir=`rospack find hrpsys_gazebo_general`
tpkgdir=`rospack find hrpsys_gazebo_tutorials`

if [ -e ${pkgdir} -a -e ${tpkgdir} ]; then
    for dname in `find ${tpkgdir}/models -mindepth 1 -maxdepth 1 -type d -regex '.*[^\.svn]'`; do export ROS_PACKAGE_PATH=${dname}:$ROS_PACKAGE_PATH; done
#    export ROS_PACKAGE_PATH=${pkgdir}/ros:$ROS_PACKAGE_PATH
    export GAZEBO_RESOURCE_PATH=${tpkgdir}/worlds:$GAZEBO_RESOURCE_PATH
    export GAZEBO_MODEL_PATH=${tpkgdir}/models:$GAZEBO_MODEL_PATH
    export GAZEBO_PLUGIN_PATH=${pkgdir}/plugins:$GAZEBO_PLUGIN_PATH
fi
