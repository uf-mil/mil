cmake_minimum_required(VERSION 2.8.3)
project(navigator_alarm)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wno-reorder -std=c++11 ")

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
    roscpp
    roslib
    navigator_msgs
)

set(catkin_LIBRARIES
  ${catkin_LIBRARIES}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp navigator_msgs roslib
  DEPENDS system_lib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(
  ${PROJECT_NAME}
  navigator_alarm/alarm_helpers.cc
)

add_dependencies(
  ${PROJECT_NAME}
  navigator_msgs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test_alarms/test_alarm_helpers.py)
  add_rostest(test_alarms/alarm_integration.test)
endif()

# Add gtest for C++ alarms
if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(navigator_alarm_integration_cpp_tests
  	test_alarms/cpp/alarm_integration_cpp.test
    test_alarms/cpp/test_alarm_helpers.cc
    navigator_alarm/alarm_helpers.cc
  )

target_link_libraries(
  	navigator_alarm_integration_cpp_tests
  	${catkin_LIBRARIES}
  	${PROJECT_NAME}
  )
endif()
