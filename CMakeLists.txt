cmake_minimum_required(VERSION 2.8.3)
project(ros_alarms)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  roscpp
  rospy
)

catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Alarm.msg
  Alarms.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AlarmGet.srv
  AlarmSet.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  ros_alarms
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ros_alarms
  CATKIN_DEPENDS
    roscpp
    rospy
    message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(ros_alarms
   src/${PROJECT_NAME}/broadcaster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_options(${PROJECT_NAME} PRIVATE -g -Wall -std=c++11)

