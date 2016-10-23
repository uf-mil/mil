cmake_minimum_required(VERSION 2.8.3)
project(navigator_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rosgraph_msgs
  navigator_msgs
)

catkin_python_setup()

catkin_package()
