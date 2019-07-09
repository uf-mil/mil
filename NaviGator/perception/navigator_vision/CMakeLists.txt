cmake_minimum_required(VERSION 2.8.3)
project(navigator_vision)

# SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
# set c++11 as default, overide with set_target_properties()
# if needed for specific nodes (cough..cough... PCL)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -pedantic -Wall -std=c++11 ")

find_package(Eigen)
find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    message_generation
    navigator_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    tf
    pcl_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    mil_vision
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED date_time filesystem)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    navigator_msgs
    navigator_tools
    pcl_ros
  DEPENDS
    system_lib
    image_transport
    image_geometry
    cv_bridge
    navigator_msgs
)

include_directories(
    include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(underwater_shape_identification_vision
  nodes/underwater_shape_identification_vision.cpp
  src/missions/underwater_shape_identification.cpp
)
add_dependencies(underwater_shape_identification_vision ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  underwater_shape_identification_vision
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(shape_identification
  nodes/shape_identification/main.cpp
  nodes/shape_identification/DockShapeVision.cpp
  nodes/shape_identification/GrayscaleContour/GrayscaleContour.cpp
  nodes/shape_identification/TrackedShape.cpp
  nodes/shape_identification/ShapeTracker.cpp
)
add_dependencies(shape_identification
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(shape_identification
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(scan_the_code_marshall
  nodes/scan_the_code_marshall.cpp
)
add_dependencies(scan_the_code_marshall
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scan_the_code_marshall
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

