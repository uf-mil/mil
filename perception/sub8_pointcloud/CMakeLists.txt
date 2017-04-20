cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(sub8_pointcloud)

find_package(catkin REQUIRED COMPONENTS

  roscpp
  rospy
  image_transport
  cv_bridge
  std_msgs
  message_generation
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_msgs
  tf2_py
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  mil_blueview_driver
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pointcloud_ogrid_lib
)



find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_library(pointcloud_ogrid_lib src/OGridGen.cpp src/Classification.cpp)
target_link_libraries(pointcloud_ogrid_lib
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${roslib_LIBRARIES}
)

include_directories(include ${roslib_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(ogrid_generator src/OGridGen.cpp)
add_dependencies(ogrid_generator pointcloud_ogrid_lib)
add_dependencies(ogrid_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(ogrid_generator pointcloud_ogrid_lib ${catkin_LIBRARIES})
