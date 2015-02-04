#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
(cd $DIR;

for pkg in rtshell rtctree rtsprofile openrtm_aist openrtm_aist_python; do
    [ ! -e $pkg ] && git clone http://github.com/tork-a/$pkg-release $pkg
done

# update HISTORY-{en,ja}.txt, CMakeLists.txt(OPENHRP_VERSION) and ask kanehiro-san for set tag
if [ ! -e openhrp3 ]; then
    git clone http://github.com/fkanehiro/openhrp3 openhrp3
fi

# update package.xml, CMakeLists.txt(CMAKE_PACKAGE_VERSION) and ask kanehiro-san for set tag
if [ ! -e hrpsys ]; then
    git clone http://github.com/fkanehiro/hrpsys-base hrpsys
fi

)
