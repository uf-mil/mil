cmake_minimum_required(VERSION 3.0.2)
project(navigator_robotx_comms)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  rospy
  ros_alarms
  tf
  geometry_msgs
  std_msgs
  nav_msgs
  navigator_msgs
  mil_tools
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/robotx_comms.test)
endif()

catkin_python_setup()

catkin_package()
