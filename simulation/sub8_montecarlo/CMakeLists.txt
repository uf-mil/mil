cmake_minimum_required(VERSION 2.8.3)
project(sub8_montecarlo)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)
catkin_python_setup()

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
if(CATKIN_ENABLE_TESTING)
  foreach(T
      montecarlo_tests/montecarlo_pd_controller.test)
      add_rostest(${T})
  endforeach()
endif()
