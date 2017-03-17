cmake_minimum_required(VERSION 2.8.3)
project(sub8_ros_tools)
find_package(catkin REQUIRED COMPONENTS rostest std_msgs)
catkin_python_setup()
catkin_package()

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test_ros_tools)
endif()

