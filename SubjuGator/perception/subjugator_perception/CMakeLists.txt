cmake_minimum_required(VERSION 3.0.2)
project(subjugator_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -rdynamic -Wall -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    subjugator_msgs
    message_generation
    std_msgs
    geometry_msgs
    mil_vision
    tf2
    tf2_ros
    tf2_eigen
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/VampireIdentifier.cfg
  cfg/HSVCalibration.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    subjugator_vision_lib
    sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    subjugator_msgs
    message_runtime
    std_msgs
    geometry_msgs
  DEPENDS
    image_transport
    image_geometry
    cv_bridge
    subjugator_msgs
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

add_library(subjugator_vision_lib
    # src/subjugator_vision_lib/align.cpp
    # src/subjugator_vision_lib/cv_param_helpers.cpp
    src/subjugator_vision_lib/visualization.cpp
    # src/subjugator_vision_lib/object_finder.cpp
)

target_link_libraries(
  subjugator_vision_lib
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
   subjugator_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  torpedos_cpp
    subjugator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  subjugator_start_gate
    nodes/start_gate_node.cpp
    src/subjugator_perception/start_gate.cpp
    src/subjugator_vision_lib/stereo_base.cpp
)
add_dependencies(
  subjugator_start_gate
   subjugator_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  subjugator_start_gate
    subjugator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(test)
