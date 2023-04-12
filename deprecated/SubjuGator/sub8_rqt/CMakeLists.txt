cmake_minimum_required(VERSION 3.0.2)
project(sub8_rqt)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_python_setup()
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
