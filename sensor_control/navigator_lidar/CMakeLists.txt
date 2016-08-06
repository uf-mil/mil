cmake_minimum_required(VERSION 2.8.3)
project(navigator_lidar)


find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	sensor_msgs
	navigator_msgs
	)

catkin_package(
	CATKIN_DEPENDS 
	pcl_ros
	roscpp
	sensor_msgs
	message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extraction src/cluster_extraction.cpp)

add_dependencies(cluster_extraction navigator_msgs_generate_messages)

target_link_libraries(
	cluster_extraction
 	${catkin_LIBRARIES}
)
