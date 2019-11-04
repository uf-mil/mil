cmake_minimum_required(VERSION 2.8.3)
project(navigator_missions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/import_missions.test)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
