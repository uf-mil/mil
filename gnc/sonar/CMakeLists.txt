cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
project(blue_view_driver)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  image_transport
  cv_bridge
  bvtsdkpkg
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
    BlueViewPing.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include 
  LIBRARIES
    sonar_tools
  CATKIN_DEPENDS
    bvtsdkpkg
    roslib
	image_transport
	cv_bridge
	message_runtime
  DEPENDS 
	OpenCV
)

include_directories(include ${bvtsdkpkg_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})

add_library(sonar_tools
			src/BVWrapper.cpp)

target_link_libraries(sonar_tools 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${roslib_LIBRARIES}
  ${bvtsdkpkg_LIBRARIES}
)

#add_executable(test_sdk test/test_sdk.cpp)

#target_link_libraries(test_sdk
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#	${bvtsdkpkg_LIBRARIES}
#	sonar_tools
#)

add_executable(blue_view_driver src/BlueViewRosDriver.cpp)
add_dependencies(blue_view_driver sonar_tools)

target_link_libraries(blue_view_driver sonar_tools ${catkin_LIBRARIES})
