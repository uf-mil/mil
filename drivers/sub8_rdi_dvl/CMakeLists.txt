cmake_minimum_required(VERSION 2.8.3)
project(rdi_explorer_dvl)
find_package(catkin REQUIRED COMPONENTS nodelet roscpp mil_msgs mil_tools)
catkin_package(
    DEPENDS
    CATKIN_DEPENDS nodelet roscpp mil_msgs mil_tools
    INCLUDE_DIRS include
    LIBRARIES
)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} $(mil_tools_INCLUDE_DIRS))
add_library(rdi_explorer_dvl_nodelet src/nodelet.cpp)
target_link_libraries(rdi_explorer_dvl_nodelet ${catkin_LIBRARIES})
add_dependencies(rdi_explorer_dvl_nodelet ${catkin_EXPORTED_TARGETS})
set_target_properties(rdi_explorer_dvl_nodelet PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

install(PROGRAMS scripts/fake_dvl DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
