cmake_minimum_required(VERSION 3.0.2)
project(depth_driver)
find_package(catkin REQUIRED COMPONENTS nodelet roscpp mil_msgs mil_tools)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS nodelet roscpp mil_msgs mil_tools
    INCLUDE_DIRS include
    LIBRARIES
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(depth_driver_nodelet src/nodelet.cpp)
target_link_libraries(depth_driver_nodelet ${catkin_LIBRARIES})
add_dependencies(depth_driver_nodelet ${catkin_EXPORTED_TARGETS})
set_target_properties(depth_driver_nodelet PROPERTIES COMPILE_FLAGS -std=c++11)
install(PROGRAMS scripts/fake_depth DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
