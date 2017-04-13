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
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
set(ros_alarms_srcs
 src/${PROJECT_NAME}/broadcaster.cpp
 src/${PROJECT_NAME}/alarm_proxy.cpp
)
add_library(${PROJECT_NAME} ${ros_alarms_srcs})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_options(${PROJECT_NAME} PRIVATE -g -Wall -std=c++11)
add_dependencies(ros_alarms ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # The test driver with your main function
  # must be listed last (before the source files)
  add_rostest_gtest(${PROJECT_NAME}_cpp_tests
    test/ros_alarms_cpp.test
    test/heartbeat_monitor_test.cpp
    test/broadcaster_test.cpp
    test/listener_test.cpp
    test/alarm_proxy_test.cpp
    test/integration_test_driver.cpp
    ${ros_alarms_srcs}
  )
  add_rostest(test/ros_alarms_python.test)
  set_target_properties(${PROJECT_NAME}_cpp_tests PROPERTIES COMPILE_FLAGS "-std=c++11")
  target_include_directories(${PROJECT_NAME}_cpp_tests PUBLIC include)
  target_link_libraries(${PROJECT_NAME}_cpp_tests ${catkin_LIBRARIES})
endif()
