cmake_minimum_required(VERSION 2.8.3)
project(mil_usb_to_can)
find_package(catkin REQUIRED COMPONENTS
  rospy
  serial
)
catkin_python_setup()
catkin_package()
