cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(sub8_alarm)

find_package(catkin REQUIRED COMPONENTS
  rospy rostest roscpp roslib sub8_msgs
)

set(catkin_LIBRARIES
  ${catkin_LIBRARIES}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp sub8_msgs roslib
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Build a shared lib for C++ alarm_helpers
add_library(
  ${PROJECT_NAME}
  sub8_alarm/alarm_helpers.cc
)

add_dependencies(
  ${PROJECT_NAME}
  sub8_msgs_generate_msgs_cpp
)

# Export headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test_alarms/test_alarm_helpers.py)
  add_rostest(test_alarms/alarm_integration.test)
endif()

# Gtest for C++ alarms
if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(alarm_integration_cpp_tests 
  	test_alarms/cpp/alarm_integration_cpp.test
    test_alarms/cpp/test_alarm_helpers.cc
    sub8_alarm/alarm_helpers.cc
  )

  target_link_libraries(
  	alarm_integration_cpp_tests 
  	${catkin_LIBRARIES}
  	${PROJECT_NAME}
  )
endif()