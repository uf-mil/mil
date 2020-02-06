cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_object_detection_and_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  mil_msgs
  interactive_markers
  eigen_conversions
  pcl_ros
  cv_bridge
  dynamic_reconfigure
  mil_bounds
  roslaunch
)

###################################################
# 				     BASE CODE 					  #
###################################################

generate_dynamic_reconfigure_options(
  cfg/PCODAR.cfg
)

# C++ Flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# PCL stuff
find_package(PCL 1.2 REQUIRED)

# OPENCV stuff
find_package(OpenCV 3.0 REQUIRED)


catkin_package(INCLUDE_DIRS include
               LIBRARIES pcodar
               CATKIN_DEPENDS roscpp sensor_msgs std_msgs mil_msgs tf tf2 eigen_conversions pcl_ros)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Create Library
add_library(pcodar
  src/input_cloud_filter.cpp
  src/marker_manager.cpp
  src/object_associator.cpp
  src/object.cpp
  src/object_detector.cpp
  src/object_map.cpp
  src/ogrid_manager.cpp
  src/pcodar_controller.cpp
  src/persistent_cloud_filter.cpp
  src/point_cloud_builder.cpp
)

# Link all the relevant libs
target_link_libraries(pcodar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(pcodar ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# executable that actually runs everything
add_executable(pcodar_node nodes/main.cpp)
target_link_libraries(pcodar_node pcodar)
add_dependencies(pcodar_node pcodar)

###################################################
# 				  LAUNCH CODE 					  #
###################################################

roslaunch_add_file_check(launch/pcodar.launch)
