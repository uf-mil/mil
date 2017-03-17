cmake_minimum_required(VERSION 2.8.3)
project(mil_dev_tools)
find_package(catkin REQUIRED COMPONENTS 
    rostest
    std_msgs
    rospy
    roscpp
)

catkin_package()

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()

catkin_python_setup()

