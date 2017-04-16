cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(mil_blueview_driver)

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
    mil_blueview_tools
  CATKIN_DEPENDS
    image_transport
    cv_bridge
    message_runtime
  DEPENDS
    bvtsdkpkg
)

include_directories(
  include
  ${bvtsdkpkg_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
)
link_libraries(
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${roslib_LIBRARIES}
  ${bvtsdkpkg_LIBRARIES}
)

add_library(mil_blueview_tools src/blueview_wrapper.cpp)
add_dependencies(mil_blueview_tools ${PROJECT_NAME}_generate_messages ${catkin_EXPORTED_TARGETS})

add_executable(blueview_driver src/blueview_ros_driver.cpp)
add_dependencies(blueview_driver mil_blueview_tools ${catkin_EXPORTED_TARGETS})
target_link_libraries(blueview_driver mil_blueview_tools)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(blueview_driver_test
    test/blueview_driver.test
    test/blueview_driver_test.cpp
  )
endif()
