cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(rtmbuild)
include(${rtmbuild_PACKAGE_PATH}/cmake/rtmbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()
set(CollisionDetector.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/ModelLoader.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/World.hh)
set(Controller.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/OpenHRPCommon.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/DynamicsSimulator.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/ViewSimulator.hh)
set(DynamicsSimulator.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/CollisionDetector.hh)
set(ModelLoader.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/OpenHRPCommon.hh)
set(OnlineViewer.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/OpenHRPCommon.hh)
set(PathPlanner.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/OpenHRPCommon.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/CollisionDetector.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/DynamicsSimulator.hh
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/ModelLoader.hh)
set(ViewSimulator.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/World.hh)
set(World.idl_depends
  ${PROJECT_SOURCE_DIR}/idl_gen/cpp/openhrp3/idl/ModelLoader.hh)
rtmbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 
set(OPENHRP3_PRIVATE_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libhrpCorbaStubSkel-3.1.a)
set(OPENHRP3_LIBRARY_DIRS ${PROJECT_SOURCE_DIR})
set(OPENHRP3_LIBRARIES hrpModel-3.1 libUtil-3.1)
rtmbuild_genidl()

rtmbuild_add_executable(JoystickController share/OpenHRP-3.1/sample/JoystickControl/Controller/JoystickControllerComp.cpp share/OpenHRP-3.1/sample/JoystickControl/Controller/JoystickController.cpp)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
rtmbuild_add_executable(openhrp-scheduler src/openhrp-scheduler.cpp)
include_directories(${LIBXML2_INCLUDE_DIRS})
target_link_libraries(openhrp-scheduler ${LIBXML2_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR}/include/OpenHRP-3.1)
rtmbuild_add_executable(extract-robotname src/extract-modelfile.cpp)
rtmbuild_add_executable(extract-modelpath src/extract-modelfile.cpp)
target_link_libraries(extract-robotname ${LIBXML2_LIBRARIES})
target_link_libraries(extract-modelpath ${LIBXML2_LIBRARIES})
rosbuild_add_compile_flags(extract-robotname -DROBOTNAME)
rosbuild_add_compile_flags(extract-modelpath -DMODELPATH)

rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)


rosbuild_add_rostest(launch/falling-box.launch)
rosbuild_add_rostest(launch/pa10.launch)
rosbuild_add_rostest(launch/sample-vehicle.launch)
rosbuild_add_rostest(launch/samplerobot-inhouse.launch)
rosbuild_add_rostest(launch/samplerobot-pickupbox.launch)
rosbuild_add_rostest(launch/samplerobot-walk.launch)
