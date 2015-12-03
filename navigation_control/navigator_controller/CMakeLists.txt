cmake_minimum_required(VERSION 2.8.3)
project(navigator_controller)

find_package(catkin REQUIRED
	geometry_msgs
	std_msgs
	nav_msgs
	tf
	)

catkin_package()
