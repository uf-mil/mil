cmake_minimum_required(VERSION 2.8.3)
project(sub8_alarm)

find_package(catkin REQUIRED COMPONENTS
  rospy rostest
)

catkin_python_setup()
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test_alarms/test_alarm_helpers.py)
    add_rostest(test_alarms/alarm_integration.test)
endif()