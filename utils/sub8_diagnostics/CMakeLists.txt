cmake_minimum_required(VERSION 2.8.3)
project(sub8_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mil_tools
  sub8_videoray_m5_thruster
)

# Any tools used to diagnose the core functionality of the sub should be put here
catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS
  mil_tools
  sub8_videoray_m5_thruster
 DEPENDS
)

