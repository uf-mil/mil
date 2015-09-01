cmake_minimum_required(VERSION 2.8.3)
project(sub8_rdi_dvl)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)