cmake_minimum_required(VERSION 2.8.3)
project(adis16400_imu)
find_package(catkin REQUIRED COMPONENTS sensor_msgs geometry_msgs nodelet roscpp mil_tools)
catkin_package(
    DEPENDS
    CATKIN_DEPENDS sensor_msgs geometry_msgs nodelet roscpp mil_tools
    INCLUDE_DIRS include
    LIBRARIES
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(adis16400_imu_nodelet src/nodelet.cpp)
target_link_libraries(adis16400_imu_nodelet ${catkin_LIBRARIES})
add_dependencies(adis16400_imu_nodelet ${catkin_EXPORTED_TARGETS})
set_target_properties(adis16400_imu_nodelet PROPERTIES COMPILE_FLAGS -std=c++11)
