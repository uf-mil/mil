cmake_minimum_required(VERSION 3.0.2)
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
