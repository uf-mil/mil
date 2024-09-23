cmake_minimum_required(VERSION 3.0.2)
project(navigator_ball_launcher)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
