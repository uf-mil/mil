cmake_minimum_required(VERSION 2.8.3)
project(navigator_kill_board)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/kill_board.test)
endif()

