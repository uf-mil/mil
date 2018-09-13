cmake_minimum_required(VERSION 2.8.3)
project(navigator_lidar_oa)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  navigator_msgs
  navigator_tools
  interactive_markers
  mil_msgs
  dynamic_reconfigure
#  message_generation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidartest
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf tf2
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar src/lidar.cpp src/lidarParams.cpp)
target_link_libraries(lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar lidar_generate_messages_cpp navigator_msgs_generate_messages_cpp mil_msgs_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})

#add_executable(actionClient src/actionClient.cpp)
#target_link_libraries(actionClient ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(actionClient lidar_generate_messages_cpp navigator_msgs_generate_messages_cpp mil_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
