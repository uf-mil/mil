cmake_minimum_required(VERSION 2.8.3)
project(mil_vision)

# set c++11 as default, overide with target_compile_options()
# if needed for specific nodes (cough..cough... PCL)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -pedantic -Wall -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    image_transport
    image_geometry
    cv_bridge
    message_runtime
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    pcl_ros
    tf
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    mil_tools
    mil_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED date_time filesystem)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    mil_vision_lib
  CATKIN_DEPENDS
    roscpp
    rospy
    image_transport
    image_geometry
    cv_bridge
    message_runtime
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    pcl_ros
    tf
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    mil_tools
    mil_msgs
  DEPENDS
)

# COMMMON: Don't repeat anything here for individual targets
include_directories(
    include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_dependencies(${catkin_EXPORTED_TARGETS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(mil_vision_lib
  src/mil_vision_lib/cv_utils.cc
  src/mil_vision_lib/image_filtering.cpp
  src/mil_vision_lib/active_contours.cpp
  src/mil_vision_lib/colorizer/pcd_colorizer.cpp
  src/mil_vision_lib/colorizer/single_cloud_processor.cpp
  src/mil_vision_lib/colorizer/camera_observer.cpp
  src/mil_vision_lib/colorizer/color_observation.cpp
)
target_include_directories(mil_vision_lib PUBLIC include/mil_vision_lib)

# Everything else here doesn't need to explicitly link
link_libraries(mil_vision_lib)

##########################################
# ALL EXECUTABLES SHOULD COME AFTER THIS #
#########################################

add_executable(camera_stream_demo src/camera_stream_demo.cpp)

add_executable(camera_lidar_transformer src/camera_lidar_transformer.cpp)
add_dependencies(camera_lidar_transformer ${catkin_EXPORTED_TARGETS})
