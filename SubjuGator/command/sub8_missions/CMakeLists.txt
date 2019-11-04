cmake_minimum_required(VERSION 2.8.3)
project(sub8_missions)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sub8_msgs
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)