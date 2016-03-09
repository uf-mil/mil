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
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sub8_pcl
  CATKIN_DEPENDS
    roscpp
    rospy
    sub8_msgs
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

add_library(sub8_pcl SHARED
    src/sub8_pcl/align.cc
    src/sub8_pcl/cv_utils.cc
    src/sub8_pcl/visualization.cc
)

target_link_libraries(
  sub8_pcl
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    bfd
    dl
)

install(TARGETS sub8_pcl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

add_executable(
  pcl_buoy
    nodes/buoy.cc
)

add_dependencies(
  pcl_buoy
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  pcl_buoy
    sub8_pcl
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