cmake_minimum_required(VERSION 3.0.2)
project(sabertooth2x12)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()
