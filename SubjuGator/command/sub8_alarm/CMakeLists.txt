cmake_minimum_required(VERSION 2.8.3)

project(sub8_alarm)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roscpp
)

set(catkin_LIBRARIES
  ${catkin_LIBRARIES}
)

catkin_python_setup()

catkin_package(
  # CATKIN_DEPENDS roscpp sub8_msgs roslib
  # DEPENDS system_lib
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
)

