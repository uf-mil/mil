cmake_minimum_required(VERSION 2.8.3)
project(navigator_keyboard_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	navigator_msg_multiplexer
	nav_msgs
)

catkin_package(
CATKIN_DEPENDS
	std_msgs
	nav_msgs
	geometry_msgs
	roscpp
	navigator_msg_multiplexer
)

catkin_python_setup()
