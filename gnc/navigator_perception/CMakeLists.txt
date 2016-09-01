cmake_minimum_required(VERSION 2.8.3)
project(navigator_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -rdynamic -Wall -std=c++11 ")
set(CMAKE_BUILD_TYPE Debug)

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

# find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    navigator_vision_lib
    sparsestereo
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
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)


add_library(
    navigator_vision_lib
    helper/model.cc 
    src/navigator_vision_lib/visualization.cc
    src/navigator_vision_lib/cv_utils.cc
    helper/stereomodelfitter.cc
)

target_link_libraries(
  navigator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


install(TARGETS navigator_vision_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

MESSAGE( STATUS "Include search path: " ${INCLUDE_DIRECTORIES} )

add_executable(
  scan_the_code
    scan_the_code_detector.cc
)

add_dependencies(
  scan_the_code
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  scan_the_code
    navigator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

install(
  TARGETS
)
