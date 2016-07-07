cmake_minimum_required(VERSION 2.8.3)
project(sub8_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -rdynamic -Wall")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    sub8_build_tools
    sub8_msgs
    message_generation
    std_msgs
    geometry_msgs
)

catkin_python_setup()

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sub8_vision_lib
  CATKIN_DEPENDS
    roscpp
    rospy
    sub8_msgs
    message_runtime
    std_msgs
    geometry_msgs
  DEPENDS
    system_lib
    image_transport
    image_geometry
    cv_bridge
    sub8_build_tools
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

add_library(sub8_vision_lib SHARED
    src/sub8_vision_lib/align.cc
    src/sub8_vision_lib/cv_utils.cc
    src/sub8_vision_lib/cv_param_helpers.cc
    src/sub8_vision_lib/visualization.cc
    src/sub8_vision_lib/object_finder.cc
)

target_link_libraries(
  sub8_vision_lib
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    bfd
    dl
)

install(TARGETS sub8_vision_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

file(GLOB BUOY_SOURCES
    "src/sub8_perception/*.cc"
)

add_executable(
  torpedos_cpp
    nodes/torpedo_board.cc
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
  perception_server
    nodes/perception_server.cc
)

add_dependencies(
  perception_server
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  perception_server
    sub8_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  pcl_buoy
    nodes/buoy_node.cc
    ${BUOY_SOURCES}
)

add_dependencies(
  pcl_buoy
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  pcl_buoy
    sub8_vision_lib
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
    bfd
    dl
)

install(
  TARGETS
)
