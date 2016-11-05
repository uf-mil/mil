cmake_minimum_required(VERSION 2.8.3)
project(navigator_scan_the_code)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  navigator_msgs
)

catkin_python_setup()
catkin_package()
