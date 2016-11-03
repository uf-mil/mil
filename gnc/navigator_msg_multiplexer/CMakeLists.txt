cmake_minimum_required(VERSION 2.8.3)
project(navigator_msg_multiplexer)

find_package(catkin REQUIRED COMPONENTS
	rospy
)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/OgridConfig.cfg
)

catkin_package()