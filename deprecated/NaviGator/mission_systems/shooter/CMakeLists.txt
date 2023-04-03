cmake_minimum_required(VERSION 3.0.2)
project(navigator_shooter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  navigator_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
)
