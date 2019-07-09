cmake_minimum_required(VERSION 2.8.3)
project(navigator_thrust_mapper)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  python_module
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
