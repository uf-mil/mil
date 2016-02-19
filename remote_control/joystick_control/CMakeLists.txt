cmake_minimum_required(VERSION 2.8.3)
project(joystick_control)
set(CMAKE_XX_FLAGS "-std=c++11")

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


add_definitions(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_control_node src/joystick_wrench.cpp)

target_link_libraries(joystick_control_node
  ${catkin_LIBRARIES}
)

