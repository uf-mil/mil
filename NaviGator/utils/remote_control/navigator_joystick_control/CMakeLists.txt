cmake_minimum_required(VERSION 3.5)
project(navigator_emergency_controller)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find_package(catkin REQUIRED COMPONENTS
# 	roscpp
# 	std_msgs
# 	geometry_msgs
# 	navigator_msg_multiplexer
# 	nav_msgs
# )

find_package(ament_cmake REQUIRED)
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigator_msg_multiplexer REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# catkin_package(
# CATKIN_DEPENDS
#	std_msgs
#	nav_msgs
#	geometry_msgs
#	roscpp
# 	navigator_msg_multiplexer
# )

ament_export_dependencies(std_msgs nav_msgs geometry_msgs roscpp navigator_msg_multiplexer)
ament_package()
