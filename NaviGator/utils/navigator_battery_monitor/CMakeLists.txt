cmake_minimum_required(VERSION 3.0.2)
project(navigator_battery_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()
