cmake_minimum_required(VERSION 3.0.2)
project(subjugator_missions)

find_package(catkin REQUIRED COMPONENTS
  rospy
  subjugator_msgs
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
