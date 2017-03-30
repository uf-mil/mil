cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(blueview_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  cv_bridge
  bvtsdkpkg
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

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
    blueview_tools
  CATKIN_DEPENDS
    bvtsdkpkg
	image_transport
	cv_bridge
	message_runtime
)

include_directories(include ${bvtsdkpkg_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})

add_library(blueview_tools
			src/BVWrapper.cpp)
target_link_libraries(blueview_tools
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${roslib_LIBRARIES}
  ${bvtsdkpkg_LIBRARIES}
)

add_executable(blueview_driver src/BlueViewRosDriver.cpp)
add_dependencies(blueview_driver blueview_tools)
add_dependencies(blueview_driver blueview_driver_generate_messages ${catkin_EXPORTED_TARGETS})
target_link_libraries(blueview_driver blueview_tools ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(blueview_driver_test
    test/blueview_driver.test
    test/blueview_driver_test.cpp
  )
  target_link_libraries(blueview_driver_test ${catkin_LIBRARIES})
endif()
