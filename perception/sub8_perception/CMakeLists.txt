cmake_minimum_required(VERSION 2.8.3)
project(sub8_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -rdynamic -Wall -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    sub8_msgs
    message_generation
    std_msgs
    geometry_msgs
    mil_vision
    tf2
    tf2_ros
    tf2_eigen
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sub8_vision_lib
    sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    sub8_msgs
    message_runtime
    std_msgs
    geometry_msgs
  DEPENDS
    image_transport
    image_geometry
    cv_bridge
    sub8_msgs
)

include_directories(
    include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(sub8_vision_lib
    # src/sub8_vision_lib/align.cpp
    # src/sub8_vision_lib/cv_param_helpers.cpp
    src/sub8_vision_lib/visualization.cpp
    # src/sub8_vision_lib/object_finder.cpp
)

target_link_libraries(
  sub8_vision_lib
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(
  torpedos_cpp
    nodes/torpedo_board.cpp
)

add_dependencies(
  torpedos_cpp
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  torpedos_cpp
    sub8_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  sub8_start_gate
    nodes/start_gate_node.cpp
    src/sub8_perception/start_gate.cpp
    src/sub8_vision_lib/stereo_base.cpp
)
add_dependencies(
  sub8_start_gate
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  sub8_start_gate
    sub8_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(test)
