cmake_minimum_required(VERSION 3.0.2)
project(subjugator_missions)

find_package(catkin REQUIRED COMPONENTS
  rospy
  subjugator_msgs
  roscpp
  mil_missions
  mil_tools
  ros_alarms
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(start_gate_test
                    test/missions.test
                    test/start_gate_test.cpp)
  target_link_libraries(start_gate_test ${catkin_LIBRARIES})
endif()
