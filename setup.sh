#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
(cd $DIR;

[ -e openrtm_aist ] || git clone http://github.com/tork-a/openrtm_aist-release  openrtm_aist
[ -e openrtm_aist_python ] || git clone http://github.com/tork-a/openrtm_aist_python-release  openrtm_aist_python
[ -e rtshell ] || git clone http://github.com/tork-a/rtshell-release rtshell
[ -e rtsprofile ] || git clone http://github.com/tork-a/rtsprofile-release rtsprofile
[ -e rtctree ] || git clone http://github.com/tork-a/rtctree-release rtctree
[ -e openhrp3 ] || git clone http://github.com/fkanehiro/openhrp3 openhrp3
[ -e hrpsys ] || git clone http://github.com/fkanehiro/hrpsys-base hrpsys
)

