cmake_minimum_required(VERSION 3.0.2)
project(sub8_videoray_m5_thruster)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()

