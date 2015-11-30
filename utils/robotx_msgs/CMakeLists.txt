cmake_minimum_required(VERSION 2.8.3)
project(robotx_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	std_msgs 
	message_generation 
	geometry_msgs
	)

add_message_files(
  FILES
  Acceleration.msg
  PoseTwist.msg
  Coordinates.msg
  PoseTwistStamped.msg
)

generate_messages(
	DEPENDENCIES 
	std_msgs 
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	std_msgs 
	geometry_msgs
    message_runtime
)


