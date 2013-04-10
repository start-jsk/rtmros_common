#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(target hrpIo)

#rosbuild_find_ros_package(hrpsys)
execute_process(
  COMMAND rospack find hrpsys
  OUTPUT_VARIABLE hrpsys_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND rospack find osrf_msgs
  OUTPUT_VARIABLE osrf_msgs_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND rospack find atlas_msgs
  OUTPUT_VARIABLE atlas_msgs_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(${hrpsys_PACKAGE_PATH}/include;${osrf_msgs_PACKAGE_PATH}/msg_gen/cpp/include;${atlas_msgs_PACKAGE_PATH}/msg_gen/cpp/include)
link_directories("/opt/ros/${ROS_DISTRO}/lib")
add_library(${target} SHARED iob.cpp)
target_link_libraries(${target} rt roscpp rostime)
target_link_libraries(${target} rt)


install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)



