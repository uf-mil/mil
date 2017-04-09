cmake_minimum_required(VERSION 2.8.3)
project(magnetic_hardsoft_compensation)
find_package(catkin REQUIRED COMPONENTS tf geometry_msgs nodelet roscpp mil_msgs eigen_conversions)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS tf geometry_msgs nodelet roscpp mil_msgs eigen_conversions
    INCLUDE_DIRS
    LIBRARIES
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(magnetic_hardsoft_compensation_nodelet src/nodelet.cpp)
target_link_libraries(magnetic_hardsoft_compensation_nodelet ${catkin_LIBRARIES})
add_dependencies(magnetic_hardsoft_compensation_nodelet ${catkin_EXPORTED_TARGETS})
set_target_properties(magnetic_hardsoft_compensation_nodelet PROPERTIES COMPILE_FLAGS -std=c++11)

install(PROGRAMS scripts/generate_config DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
