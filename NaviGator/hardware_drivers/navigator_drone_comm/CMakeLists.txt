cmake_minimum_required(VERSION 3.0.2)
project(navigator_drone_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/simulated.test)
endif()
