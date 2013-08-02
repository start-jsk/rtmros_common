# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosnode_rtc)

find_package(catkin REQUIRED COMPONENTS openrtm_tools)

catkin_package()

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN ".svn" EXCLUDE)