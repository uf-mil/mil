cmake_minimum_required(VERSION 2.8.3)
project(navigator_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	geometry_msgs
	)

add_message_files(
  DIRECTORY msg
  FILES
  Acceleration.msg
  PoseTwist.msg
  Coordinates.msg
  PoseTwistStamped.msg
  Point.msg
  Points.msg
  Waypoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ShooterManual.srv
  VisionRequest.srv
  WrenchSelect.srv
  StartGate.srv
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


