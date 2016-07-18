cmake_minimum_required(VERSION 2.8.3)
project(sub8_hydrophones)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sub8_sonar
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
