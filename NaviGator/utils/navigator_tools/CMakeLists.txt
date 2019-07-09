cmake_minimum_required(VERSION 2.8.3)
project(navigator_tools)
find_package(catkin
  REQUIRED COMPONENTS
    rospy
    roslib
    std_msgs
    nav_msgs
    geometry_msgs
    tf
)
catkin_python_setup()

catkin_package()
