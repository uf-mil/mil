
cmake_minimum_required(VERSION 2.8.3)
project(navigator_sim_rendering)

find_package(catkin REQUIRED COMPONENTS tf rospy sensor_msgs)
catkin_python_setup()

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS tf rospy sensor_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


