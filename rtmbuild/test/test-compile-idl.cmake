cmake_minimum_required(VERSION 2.8.3)
project(rtmbuild_test)

# call catkin depends
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp)

# initialize rtmbuild
rtmbuild_init()

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

