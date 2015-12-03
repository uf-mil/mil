cmake_minimum_required(VERSION 2.8.3)
project(navigator_msg_multiplexer)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	message_generation
	)

 add_service_files(
   FILES
   wrench_arbiter.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package()
