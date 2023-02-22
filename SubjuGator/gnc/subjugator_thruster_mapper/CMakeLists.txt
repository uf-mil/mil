cmake_minimum_required(VERSION 2.8.3)
project(subjugator_thruster_mapper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  foreach(T
      test/test_subjugator_mapper.test)
     add_rostest(${T})
  endforeach()
endif()
