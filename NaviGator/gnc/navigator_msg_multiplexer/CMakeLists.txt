cmake_minimum_required(VERSION 3.0.2)
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
