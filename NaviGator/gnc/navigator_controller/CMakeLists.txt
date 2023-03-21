cmake_minimum_required(VERSION 3.0.2)
project(navigator_controller)

find_package(catkin REQUIRED
	geometry_msgs
	std_msgs
	nav_msgs
	tf
	)

catkin_package()
